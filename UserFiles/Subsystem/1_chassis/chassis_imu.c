//
// 底盘 IMU 模块实现
// Created by pomelo on 2025/12/4
//
// - TYPE 0: 本地硬件 IMU（第二个 BMI088））
// - TYPE 1: CAN 接收模式（从底盘主控板接收）
//
// 帧1 (0x501): Yaw角度(int16, 0.01°) + Yaw角速度(int16, 0.01rad/s) + 保留
// 帧2 (0x502): Pitch(int16) + Roll(int16) + 状态(uint8) + 保留
//

#include "chassis_imu.h"
#include "IMU.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "task_user.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/*====================内部变量====================*/

// 中断接收缓冲（volatile，中断写入）
static volatile float imu_rx_yaw = 0.0f;
static volatile float imu_rx_gyro_z = 0.0f;
static volatile float imu_rx_accel_x = 0.0f;
static volatile float imu_rx_accel_y = 0.0f;
static volatile float imu_rx_pitch = 0.0f;
static volatile float imu_rx_roll = 0.0f;
static volatile float imu_rx_gyro_x = 0.0f;
static volatile uint8_t imu_rx_calibrated = 0;
static volatile uint8_t imu_data_ready = 0;

// 任务使用的数据副本
static Chassis_IMU_Data_t chassis_imu_data;

// 外部时间戳
extern volatile uint32_t nowtime;

float chassis_omega_z;

// CAN 通信相关
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/*====================CAN 协议定义====================*/

// 数据缩放因子
#define ANGLE_SCALE 100.0f // 角度: 0.01° 精度
#define GYRO_SCALE 1000.0f // 角速度: 0.001 rad/s 精度

// CAN 帧接收标志（volatile 因为在中断中修改）
static volatile uint8_t frame1_received = 0;
static volatile uint8_t frame2_received = 0;
static volatile uint32_t last_can_tick = 0;

/*====================CAN 接收模式====================*/

#if CHASSIS_IMU_TYPE == 1

/**
 * @brief CAN 接收模式初始化
 */
static int8_t Chassis_IMU_Init_CAN(void) {
  memset(&chassis_imu_data, 0, sizeof(chassis_imu_data));
  chassis_imu_data.is_initialized = 1;
  chassis_imu_data.is_calibrated = 0;
  imu_data_ready = 0;
  frame1_received = 0;
  frame2_received = 0;
  last_can_tick = 0;
  return 0;
}

/**
 * @brief CAN 接收模式更新（同步数据 + 检查超时）
 */
static void Chassis_IMU_Update_CAN(void) {
  // 从中断缓冲区同步数据到任务副本
  if (imu_data_ready) {
    // 使用临界区保护，直接复制 volatile 变量
    taskENTER_CRITICAL();
    chassis_imu_data.yaw = imu_rx_yaw;
    chassis_omega_z = imu_rx_gyro_z * -10.0f;
    chassis_imu_data.gyro_z = imu_rx_gyro_z;
    chassis_imu_data.accel_x = imu_rx_accel_x;
    chassis_imu_data.accel_y = imu_rx_accel_y;
    chassis_imu_data.pitch = imu_rx_pitch;
    chassis_imu_data.roll = imu_rx_roll;
    chassis_imu_data.gyro_x = imu_rx_gyro_x;
    chassis_imu_data.is_calibrated = imu_rx_calibrated;
    imu_data_ready = 0;
    taskEXIT_CRITICAL();
  }

  // 检查通信超时
  uint32_t current_tick = nowtime / 10;     // 0.1ms -> ms
  uint32_t saved_last_tick = last_can_tick; // 读取 volatile

  if (saved_last_tick > 0) {
    uint32_t elapsed;
    if (current_tick >= saved_last_tick) {
      elapsed = current_tick - saved_last_tick;
    } else {
      elapsed = current_tick + (0xFFFFFFFF - saved_last_tick);
    }

    if (elapsed > CHASSIS_IMU_CAN_TIMEOUT) {
      // 通信超时
      chassis_imu_data.is_calibrated = 0;
    }
  }
}

/**
 * @brief 处理 CAN 帧1: Yaw 角度 + Yaw 角速度（中断上下文）
 */
static void Parse_CAN_Frame1(uint8_t *data) {
  // 字节 0-1: Yaw 角度 (int16, 缩放 100)
  int16_t yaw_raw = (int16_t)((data[0] << 8) | data[1]);
  imu_rx_yaw = (float)yaw_raw / ANGLE_SCALE;

  // 字节 2-3: Yaw 角速度 (int16, 缩放 1000)
  int16_t gyro_z_raw = (int16_t)((data[2] << 8) | data[3]);
  imu_rx_gyro_z = (float)gyro_z_raw / GYRO_SCALE;

  // 字节 4-5: 加速度 X (int16, 缩放 1000, 单位 m/s²)
  int16_t accel_x_raw = (int16_t)((data[4] << 8) | data[5]);
  imu_rx_accel_x = (float)accel_x_raw / GYRO_SCALE;

  // 字节 6-7: 加速度 Y
  int16_t accel_y_raw = (int16_t)((data[6] << 8) | data[7]);
  imu_rx_accel_y = (float)accel_y_raw / GYRO_SCALE;

  frame1_received = 1;
  imu_data_ready = 1;
}

/**
 * @brief 处理 CAN 帧2: Pitch + Roll + 状态（中断上下文）
 */
static void Parse_CAN_Frame2(uint8_t *data) {
  // 字节 0-1: Pitch 角度 (int16, 缩放 100)
  int16_t pitch_raw = (int16_t)((data[0] << 8) | data[1]);
  imu_rx_pitch = (float)pitch_raw / ANGLE_SCALE;

  // 字节 2-3: Roll 角度 (int16, 缩放 100)
  int16_t roll_raw = (int16_t)((data[2] << 8) | data[3]);
  imu_rx_roll = (float)roll_raw / ANGLE_SCALE;

  // 字节 4: 状态标志
  imu_rx_calibrated = data[4] & 0x01;

  // 字节 5-6: 陀螺仪 X
  int16_t gyro_x_raw = (int16_t)((data[5] << 8) | data[6]);
  imu_rx_gyro_x = (float)gyro_x_raw / GYRO_SCALE;

  frame2_received = 1;
  imu_data_ready = 1;
}

#endif // CHASSIS_IMU_TYPE == 1

/*====================底盘板发送功能====================*/

/**
 * @brief 底盘板发送 IMU 数据到 CAN
 * @note 底盘板需要在 1kHz 任务中调用此函数
 */
void Chassis_IMU_CAN_Transmit(void) {
#if CURRENT_BOARD_ROLE == BOARD_ROLE_CHASSIS
  // 获取本地 IMU 数据
  float angles[3];
  angles[0] = g_imu_yaw;
  angles[1] = g_imu_pitch;
  angles[2] = g_imu_roll;

  // 获取陀螺仪原始数据（用于角速度）
  float imu_raw[7];
  extern void IMU_TT_getgyro(float *zsjganda);
  IMU_TT_getgyro(imu_raw);
  imu_raw[5] = g_rawomega_yaw; // 这里返回的是使用分向量校准过的速度

  // 准备 CAN 帧1
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];
  uint32_t mailbox;

  tx_header.StdId = CAN_MSG_CHASSIS_IMU_1;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;

  // 打包帧1: Yaw + Gyro_Z + Accel_X + Accel_Y
  int16_t yaw_raw = (int16_t)(angles[0] * ANGLE_SCALE);
  int16_t gyro_z_raw = (int16_t)(imu_raw[5] * GYRO_SCALE);         // deg/s
  int16_t accel_x_raw = (int16_t)(imu_raw[0] * 9.8f * GYRO_SCALE); // g -> m/s²
  int16_t accel_y_raw = (int16_t)(imu_raw[1] * 9.8f * GYRO_SCALE);

  tx_data[0] = (yaw_raw >> 8) & 0xFF;
  tx_data[1] = yaw_raw & 0xFF;
  tx_data[2] = (gyro_z_raw >> 8) & 0xFF;
  tx_data[3] = gyro_z_raw & 0xFF;
  tx_data[4] = (accel_x_raw >> 8) & 0xFF;
  tx_data[5] = accel_x_raw & 0xFF;
  tx_data[6] = (accel_y_raw >> 8) & 0xFF;
  tx_data[7] = accel_y_raw & 0xFF;

  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &mailbox);

  // 准备 CAN 帧2
  tx_header.StdId = CAN_MSG_CHASSIS_IMU_2;

  int16_t pitch_raw = (int16_t)(angles[1] * ANGLE_SCALE);
  int16_t roll_raw = (int16_t)(angles[2] * ANGLE_SCALE);
  int16_t gyro_x_raw = (int16_t)(imu_raw[3] * (M_PI / 180.0f) * GYRO_SCALE);

  tx_data[0] = (pitch_raw >> 8) & 0xFF;
  tx_data[1] = pitch_raw & 0xFF;
  tx_data[2] = (roll_raw >> 8) & 0xFF;
  tx_data[3] = roll_raw & 0xFF;
  tx_data[4] = 0x01; // 状态: calibrated
  tx_data[5] = (gyro_x_raw >> 8) & 0xFF;
  tx_data[6] = gyro_x_raw & 0xFF;
  tx_data[7] = 0x00; // 保留

  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &mailbox);
#endif
}

/*====================外部接口实现====================*/

int8_t Chassis_IMU_Init(void) {
#if !CHASSIS_IMU_ENABLED
  return 0;
#endif

#if CHASSIS_IMU_TYPE == 1
  return Chassis_IMU_Init_CAN();
#else
  return -1;
#endif
}

void Chassis_IMU_Update(void) {
#if !CHASSIS_IMU_ENABLED
  return;
#endif

  if (!chassis_imu_data.is_initialized)
    return;

#if CHASSIS_IMU_TYPE == 1
  Chassis_IMU_Update_CAN();
#endif
}

const Chassis_IMU_Data_t *Chassis_IMU_Get_Data(void) {
  return &chassis_imu_data;
}

float Chassis_IMU_Get_Yaw(void) { return chassis_imu_data.yaw; }

float Chassis_IMU_Get_Yaw_Rate(void) { return chassis_imu_data.gyro_z; }

void Chassis_IMU_Get_Accel(float *ax, float *ay) {
  if (ax)
    *ax = chassis_imu_data.accel_x;
  if (ay)
    *ay = chassis_imu_data.accel_y;
}

void Chassis_IMU_Reset_Yaw(void) { chassis_imu_data.yaw = 0.0f; }

/**
 * @brief CAN 接收回调（中断上下文）
 */
void Chassis_IMU_CAN_Callback(uint32_t can_id, uint8_t *data) {
#if CHASSIS_IMU_TYPE == 1
  if (can_id == CAN_MSG_CHASSIS_IMU_1) {
    Parse_CAN_Frame1(data);
    last_can_tick = nowtime / 10;
  } else if (can_id == CAN_MSG_CHASSIS_IMU_2) {
    Parse_CAN_Frame2(data);
  }
#else
  (void)can_id;
  (void)data;
#endif
}

uint8_t Chassis_IMU_CAN_Is_Connected(void) {
#if CHASSIS_IMU_TYPE == 1
  return chassis_imu_data.is_calibrated && frame1_received && frame2_received;
#else
  return 1;
#endif
}
