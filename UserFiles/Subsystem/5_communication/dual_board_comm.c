//
// 双板通信模块实现
// Created by pomelo on 2025/12/5
// 双板通信部分ai写得多，有bug问ai（
//
// 架构说明：
// - CAN1: 各板控制各自的电机（云台板控制 GM6020，底盘板控制 M3508）
// - CAN2: 双板通信（IMU 数据、控制命令、心跳）
//
// 通信协议：
// 1. 云台板 -> 底盘板: 底盘目标转速 (0x510)
// 2. 底盘板 -> 云台板: IMU 数据 (0x501, 0x502)
// 3. 双向心跳: 云台板 (0x520), 底盘板 (0x521)
//

#include "dual_board_comm.h"
#include "bsp_can.h"
#include "chassis_imu.h"
#include "cmsis_os.h"
#include "robot_config.h"
#include <string.h>

/*====================内部变量====================*/

// 心跳状态（直接使用 volatile）
static volatile uint32_t gimbal_heartbeat_tick = 0;  // 云台板心跳时间戳
static volatile uint32_t chassis_heartbeat_tick = 0; // 底盘板心跳时间戳
static volatile uint16_t gimbal_heartbeat_seq = 0;
static volatile uint16_t chassis_heartbeat_seq = 0;

// 底盘命令（使用双缓冲）
static Chassis_Cmd_t chassis_cmd_buf[2];
static volatile uint8_t cmd_write_idx = 0;
static volatile uint8_t cmd_ready = 0;
static Chassis_Cmd_t chassis_cmd_copy;

// 发送相关
static volatile uint16_t heartbeat_seq = 0;
static volatile uint32_t last_heartbeat_send_tick = 0;

// 外部 CAN 句柄
extern CAN_HandleTypeDef hcan2;

/*====================外部接口实现====================*/

void DualBoard_Comm_Init(void) {
  gimbal_heartbeat_tick = 0;
  chassis_heartbeat_tick = 0;
  gimbal_heartbeat_seq = 0;
  chassis_heartbeat_seq = 0;
  cmd_write_idx = 0;
  cmd_ready = 0;
  memset(chassis_cmd_buf, 0, sizeof(chassis_cmd_buf));
  memset(&chassis_cmd_copy, 0, sizeof(chassis_cmd_copy));
  heartbeat_seq = 0;
  last_heartbeat_send_tick = 0;
}

void DualBoard_Comm_Update(void) {
  // 同步底盘命令（如果有新数据）
  if (cmd_ready) {
    taskENTER_CRITICAL();
    uint8_t read_idx = cmd_write_idx;
    cmd_write_idx = 1 - cmd_write_idx;
    cmd_ready = 0;
    taskEXIT_CRITICAL();

    memcpy(&chassis_cmd_copy, &chassis_cmd_buf[read_idx],
           sizeof(Chassis_Cmd_t));
  }

  // 心跳超时检查不需要特殊同步，因为只读取 volatile 时间戳
  // 超时判断在 DualBoard_Is_Peer_Online() 中实时进行
}

void DualBoard_Send_Heartbeat(void) {
  uint32_t current_tick = HAL_GetTick();

  // 控制发送频率
  if (current_tick - last_heartbeat_send_tick < HEARTBEAT_INTERVAL_MS) {
    return;
  }
  last_heartbeat_send_tick = current_tick;

  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8] = {0};
  uint32_t mailbox;

#if CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL
  tx_header.StdId = CAN_MSG_HEARTBEAT_GIMBAL;
#else
  tx_header.StdId = CAN_MSG_HEARTBEAT_CHASSIS;
#endif

  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;

  // 打包心跳数据
  uint16_t seq = heartbeat_seq;
  tx_data[0] = (seq >> 8) & 0xFF;
  tx_data[1] = seq & 0xFF;
  tx_data[2] = 0;                          // 错误码
  tx_data[3] = DualBoard_Is_Peer_Online(); // 通信状态
  // 字节 4-7: 时间戳
  tx_data[4] = (current_tick >> 24) & 0xFF;
  tx_data[5] = (current_tick >> 16) & 0xFF;
  tx_data[6] = (current_tick >> 8) & 0xFF;
  tx_data[7] = current_tick & 0xFF;

  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &mailbox);
  heartbeat_seq++;
}

void DualBoard_Send_Chassis_Cmd(int16_t *wheel_rpm, uint8_t mode) {
#if CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];
  uint32_t mailbox;

  tx_header.StdId = CAN_MSG_GIMBAL_TO_CHASSIS;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;

  // 打包 4 个轮子的 RPM (每个 int16)
  // 由于 8 字节只能放 4 个 int16，刚好够用
  tx_data[0] = (wheel_rpm[0] >> 8) & 0xFF;
  tx_data[1] = wheel_rpm[0] & 0xFF;
  tx_data[2] = (wheel_rpm[1] >> 8) & 0xFF;
  tx_data[3] = wheel_rpm[1] & 0xFF;
  tx_data[4] = (wheel_rpm[2] >> 8) & 0xFF;
  tx_data[5] = wheel_rpm[2] & 0xFF;
  tx_data[6] = (wheel_rpm[3] >> 8) & 0xFF;
  tx_data[7] = wheel_rpm[3] & 0xFF;

  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &mailbox);
#else
  (void)wheel_rpm;
  (void)mode;
#endif
}

const Chassis_Cmd_t *DualBoard_Get_Chassis_Cmd(void) {
  return &chassis_cmd_copy;
}

uint8_t DualBoard_Is_Peer_Online(void) {
  // 实时计算在线状态（基于 volatile 时间戳）
  uint32_t current_tick = HAL_GetTick();

#if CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL
  // 云台板检查底盘板心跳
  uint32_t last_tick = chassis_heartbeat_tick;
  if (last_tick == 0)
    return 0; // 从未收到过心跳
  return (current_tick - last_tick) < HEARTBEAT_TIMEOUT_MS;
#else
  // 底盘板检查云台板心跳
  uint32_t last_tick = gimbal_heartbeat_tick;
  if (last_tick == 0)
    return 0; // 从未收到过心跳
  return (current_tick - last_tick) < HEARTBEAT_TIMEOUT_MS;
#endif
}

void DualBoard_CAN_Callback(uint32_t can_id, uint8_t *data) {
  // 此函数在 CAN 中断上下文中调用

  switch (can_id) {
  case CAN_MSG_HEARTBEAT_GIMBAL:
    // 底盘板收到云台板心跳 - 直接更新 volatile 时间戳
    gimbal_heartbeat_tick = HAL_GetTick();
    gimbal_heartbeat_seq = (uint16_t)((data[0] << 8) | data[1]);
    break;

  case CAN_MSG_HEARTBEAT_CHASSIS:
    // 云台板收到底盘板心跳 - 直接更新 volatile 时间戳
    chassis_heartbeat_tick = HAL_GetTick();
    chassis_heartbeat_seq = (uint16_t)((data[0] << 8) | data[1]);
    break;

  case CAN_MSG_GIMBAL_TO_CHASSIS: {
    // 底盘板收到控制命令 - 使用双缓冲
    Chassis_Cmd_t *buf = &chassis_cmd_buf[cmd_write_idx];
    buf->wheel_rpm[0] = (int16_t)((data[0] << 8) | data[1]);
    buf->wheel_rpm[1] = (int16_t)((data[2] << 8) | data[3]);
    buf->wheel_rpm[2] = (int16_t)((data[4] << 8) | data[5]);
    buf->wheel_rpm[3] = (int16_t)((data[6] << 8) | data[7]);
    buf->timestamp = HAL_GetTick();
    cmd_ready = 1;
    break;
  }

  case CAN_MSG_CHASSIS_IMU_1:
  case CAN_MSG_CHASSIS_IMU_2:
    // 转发给 Chassis IMU 模块处理
    Chassis_IMU_CAN_Callback(can_id, data);
    break;

  default:
    break;
  }
}
