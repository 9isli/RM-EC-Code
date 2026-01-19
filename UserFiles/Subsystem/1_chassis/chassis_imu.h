//
// Created by pomelo on 2025/12/4
//

#ifndef __CHASSIS_IMU_H
#define __CHASSIS_IMU_H

#include "main.h"
#include "robot_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  // 欧拉角（世界坐标系，单位：度）
  float yaw;   // 偏航角 [-180, 180]
  float pitch; // 俯仰角 [-90, 90]
  float roll;  // 滚转角 [-180, 180]

  // 角速度（单位：rad/s）
  float gyro_x; // 绕 X 轴角速度
  float gyro_y; // 绕 Y 轴角速度
  float gyro_z; // 绕 Z 轴角速度（底盘自转角速度）

  // 加速度（单位：m/s²）
  float accel_x;
  float accel_y;
  float accel_z;

  // 状态标志
  uint8_t is_initialized;    // 是否已初始化
  uint8_t is_calibrated;     // 是否已校准
  uint32_t last_update_tick; // 最后更新时间戳
} Chassis_IMU_Data_t;

extern float chassis_omega_z;

/**
 * @brief 底盘 IMU 初始化
 * @return 0 成功，非 0 失败
 */
int8_t Chassis_IMU_Init(void);

/**
 * @brief 底盘 IMU 数据更新
 * @note 需要周期性调用（建议 1kHz）
 */
void Chassis_IMU_Update(void);

/**
 * @brief 获取底盘 IMU 数据指针
 */
const Chassis_IMU_Data_t *Chassis_IMU_Get_Data(void);

/**
 * @brief 获取底盘 Yaw 角度（世界坐标系）
 * @return Yaw 角度（度）
 */
float Chassis_IMU_Get_Yaw(void);

/**
 * @brief 获取底盘 Yaw 角速度
 * @return Yaw 角速度（rad/s）
 */
float Chassis_IMU_Get_Yaw_Rate(void);

/**
 * @brief 获取底盘加速度（用于打滑检测）
 * @param[out] ax X 轴加速度 (m/s²)
 * @param[out] ay Y 轴加速度 (m/s²)
 */
void Chassis_IMU_Get_Accel(float *ax, float *ay);

/**
 * @brief 重置底盘 IMU Yaw 角度为零
 */
void Chassis_IMU_Reset_Yaw(void);

/**
 * @brief CAN 接收回调（由 bsp_can.c 调用）
 * @param can_id CAN 消息 ID
 * @param data 8 字节数据
 */
void Chassis_IMU_CAN_Callback(uint32_t can_id, uint8_t *data);

/**
 * @brief 发送本地 IMU 数据到 CAN（底盘板调用）
 * @note 底盘主控板需要周期性调用此函数（建议 1kHz）
 */
void Chassis_IMU_CAN_Transmit(void);

/**
 * @brief 检查 CAN 通信是否正常
 * @return 1: 正常, 0: 超时
 */
uint8_t Chassis_IMU_CAN_Is_Connected(void);

#ifdef __cplusplus
}
#endif

#endif // __CHASSIS_IMU_H
