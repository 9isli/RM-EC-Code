//
// Created by pomelo on 2025/11/18.
//

#include "IMU.h"
#include "bmi088_module.h"
#include "bsp_uart.h"
#include "chassis_imu.h"
#include "cmsis_os.h"
#include "dji_motor.h"
#include "main.h"
#include "task_user.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static float g_imu_temperature_c = 0.0f;
static float g_m1_speed = 0.0f;
static float g_m2_speed = 0.0f;
// static float g_m3_speed = 0.0f;
// static float g_m4_speed = 0.0f;
static float g_m2006_speed = 0.0f;
static float g_yaw_speed = 0.0f;
static float g_yaw_angle = 0.0f;
static float g_yaw_current = 0.0f;
extern float g_imu_yaw;
static float g_chassis_yaw = 0.0f;
static float g_total_yaw = 0.0f;
static float g_pitch_angle = 0.0f;
static float g_chassis_wz = 0.0f;
// static uint8_t tx_buffer[128];
extern int16_t ctrl_output;

extern UART_HandleTypeDef huart1;

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 监控任务，周期性读取 IMU 温度、电机转速等状态
 * @param argument 未使用
 */

void Monitor_Task(void *argument) {
  (void)argument;

  int32_t temp_raw = 0;
  int8_t rslt;

  // 等待系统初始化任务完成
  while (!g_system_init_done) {
    osDelay(1);
  }

  for (;;) {
    // 每10ms获取一次BMI088温度
    rslt = BMI088_Module_Read_Temp(&temp_raw);
    if (rslt == BMI08X_OK) {
      // temp_raw is in milli-Celsius (e.g., 23000 = 23°C)
      g_imu_temperature_c = temp_raw / 1000.0f;
    }

    // 获取电机转速
    const motor_measure_t *m1 = get_chassis_m3508_measure_point(0);
    const motor_measure_t *m2 = get_chassis_m3508_measure_point(1);
    const motor_measure_t *m2006 = get_chassis_m3508_measure_point(2);
    // const motor_measure_t *m3 = get_chassis_m3508_measure_point(2);
    // const motor_measure_t *m4 = get_chassis_m3508_measure_point(3);

    const motor_measure_t *yaw = get_yaw_gimbal_gm6020_measure_point();
    const motor_measure_t *pitch = get_pitch_gimbal_gm6020_measure_point();

    g_m1_speed = m1->speed_rpm;
    g_m2_speed = m2->speed_rpm;
    g_m2006_speed = m2006->speed_rpm;
    // g_m3_speed = m3->speed_rpm;
    // g_m4_speed = m4->speed_rpm;
    g_yaw_speed = yaw->speed_rpm;
    g_yaw_angle = yaw->ecd;
    g_yaw_current = yaw->given_current;
    g_pitch_angle = pitch->ecd;
    g_chassis_yaw = Chassis_IMU_Get_Yaw();
    g_total_yaw = IMU_getYawTotalAngle();
    g_chassis_wz = chassis_omega_z * (M_PI / 180.0f);

    // int length = sprintf((char *)tx_buffer, "%d\n", (int32_t)g_chassis_yaw);

    // // 等待上一次发送完成，再发送新数据
    // while (huart1.gState != HAL_UART_STATE_READY) {
    //   osDelay(1);
    // }
    // HAL_UART_Transmit_DMA(&huart1, tx_buffer, length);

    osDelay(10); // 100Hz 发送频率
  }
}
