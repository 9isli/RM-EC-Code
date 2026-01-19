//
// Created by pomelo on 2025/11/17.
//

#include "IMU.h"
#include "cmsis_os.h"
#include "task_user.h"

// 200Hz IMU姿态更新任务使用的信号量
extern osSemaphoreId_t imuSemHandle;

// 全局可见的三个欧拉角（单位：度）
float g_imu_yaw = 0.0f;   // 偏航角
float g_imu_pitch = 0.0f; // 俯仰角
float g_imu_roll = 0.0f;  // 滚转角
// 全局可见的三个角速度（单位：度/s）
float g_omega_yaw = 0.0f;
float g_omega_pitch = 0.0f;
float g_omega_roll = 0.0f;
// 全局可见的三个角速度原数据（单位：度/s）
float g_rawomega_yaw = 0.0f;
float g_rawomega_pitch = 0.0f;
float g_rawomega_roll = 0.0f;
// 全局可见的yaw累计角度
float g_totalyaw = 0.0f;

/**
 * @brief IMU 姿态更新任务，周期性读取并更新全局姿态角
 * @param argument 未使用
 */

void IMU_Task(void *argument) {
  (void)argument;

  // 等待系统初始化完成
  while (!g_system_init_done) {
    osDelay(1);
  }

  // 根据温控状态决定是否等待达温
  // g_imu_temp_check_enable = 1 时，需要等到温度接近目标值（由温控任务置位）
  while (g_imu_temp_check_enable && !g_imu_temp_ready) {
    osDelay(10);
  }

  float angles[3];
  float omega[6];

  for (;;) {
    // 由 TIM6 中断以 1000Hz 释放信号量
    osSemaphoreAcquire(imuSemHandle, osWaitForever);

    // 更新姿态并保存到全局变量
    IMU_getYawPitchRoll(angles);
    IMU_getGyroData(omega);
    g_totalyaw = IMU_getYawTotalAngle();
    g_imu_yaw = angles[0];
    g_imu_pitch = angles[1];
    g_imu_roll = angles[2];
    g_omega_roll = omega[0];
    g_omega_pitch = omega[1];
    g_omega_yaw = omega[2];
    g_rawomega_roll = omega[3];
    g_rawomega_pitch = omega[4];
    g_rawomega_yaw = omega[5];
  }
}
