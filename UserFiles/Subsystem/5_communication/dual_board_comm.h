//
// Created by pomelo on 2025/12/5
//

#ifndef __DUAL_BOARD_COMM_H
#define __DUAL_BOARD_COMM_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "robot_config.h"

/**
 * @brief 底盘控制命令结构体（云台板 -> 底盘板）
 */
typedef struct {
  int16_t wheel_rpm[4]; // 四个轮子目标转速 (RPM)
  uint8_t mode;         // 底盘模式
  uint8_t reserved;     // 保留
  uint32_t timestamp;   // 时间戳
} Chassis_Cmd_t;

/**
 * @brief 心跳数据结构体
 */
typedef struct {
  uint32_t last_recv_tick; // 最后接收时间
  uint8_t is_connected;    // 连接状态
  uint8_t error_code;      // 错误码
  uint16_t seq_num;        // 序列号
} Heartbeat_Status_t;

/**
 * @brief 双板通信状态
 */
typedef struct {
  Heartbeat_Status_t gimbal_heartbeat;  // 云台板心跳状态（底盘板使用）
  Heartbeat_Status_t chassis_heartbeat; // 底盘板心跳状态（云台板使用）
  Chassis_Cmd_t chassis_cmd;            // 底盘控制命令
  uint8_t comm_ok;                      // 通信是否正常
} DualBoard_Comm_t;

/**
 * @brief 双板通信初始化
 */
void DualBoard_Comm_Init(void);

/**
 * @brief 双板通信更新（检查心跳超时）
 * @note 在主循环中周期性调用
 */
void DualBoard_Comm_Update(void);

/**
 * @brief 发送心跳包
 * @note 由各板周期性调用（建议 50ms）
 */
void DualBoard_Send_Heartbeat(void);

/**
 * @brief 发送底盘控制命令（云台板调用）
 * @param wheel_rpm 四个轮子目标转速数组
 * @param mode 底盘模式
 */
void DualBoard_Send_Chassis_Cmd(int16_t *wheel_rpm, uint8_t mode);

/**
 * @brief 获取底盘控制命令（底盘板调用）
 * @return 底盘控制命令指针
 */
const Chassis_Cmd_t *DualBoard_Get_Chassis_Cmd(void);

/**
 * @brief 检查对方板是否在线
 * @return 1: 在线, 0: 离线
 */
uint8_t DualBoard_Is_Peer_Online(void);

/**
 * @brief 获取通信状态
 */
const DualBoard_Comm_t *DualBoard_Get_Status(void);

/**
 * @brief CAN 接收回调（由 bsp_can.c 调用）
 * @param can_id CAN 消息 ID
 * @param data 8 字节数据
 */
void DualBoard_CAN_Callback(uint32_t can_id, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif // __DUAL_BOARD_COMM_H
