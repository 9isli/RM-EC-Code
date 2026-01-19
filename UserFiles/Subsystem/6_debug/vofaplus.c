//
// Created by pomelo on 2025/10/14.
//

/**
 * @file vofaplus.c
 * @brief VOFA+数据采集模块实现
 * @details 用于读取传感器数据并通过UART发送给VOFA+上位机进行实时监测
 */

#include "vofaplus.h"
#include "bsp_uart.h"
#include "tim.h"

float m1speed = 0.0f;

/** @brief 发送缓冲区 */
static uint8_t tx_buffer[128];

/** @brief 操作结果状态 */
int8_t rslt;

/** @brief 原始温度数据 */
int32_t temp_raw = 0;

/** @brief 摄氏温度 */
float temperature_c = 0.0f;

/**
 * @brief 更新VOFA+数据采集
 * @details 读取BMI088温度传感器数据，格式化后通过UART发送给VOFA+上位机
 * @retval 无
 * @note 建议定期调用（建议10ms周期）
 */
void GetRpm_Update(void) {
  // const motor_measure_t *m1 = get_chassis_m3508_measure_point(0);
  // const motor_measure_t *m2 = get_chassis_m3508_measure_point(1);
  // const motor_measure_t *m3 = get_chassis_m3508_measure_point(2);
  // const motor_measure_t *m4 = get_chassis_m3508_measure_point(3);

  // m1speed = m1->speed_rpm;

  rslt = BMI088_Module_Read_Temp(&temp_raw);
  if (rslt == BMI08X_OK) {
    // temp_raw is in milli-Celsius (e.g., 23000 = 23°C)
    temperature_c = temp_raw / 1000.0f;

    // Split temperature into integer and decimal parts
    int temp_int = (int)temperature_c;
    int temp_dec = (int)((temperature_c - temp_int) * 100);
    int length = sprintf((char *)tx_buffer, "%d.%02d\n", temp_int, temp_dec);
    UART1_Send_DMA(tx_buffer, length);
  }

  // vofa+格式化输出，协议为firewater
  // int length = sprintf((char*)tx_buffer, "%d, %d, %d, %d\n", m1->speed_rpm,
  // m2->speed_rpm, m3->speed_rpm, m4->speed_rpm);
  // m2->speed_rpm, m3->speed_rpm, m4->speed_rpm); int length =
  // sprintf((char*)tx_buffer, "%d, %d\n", m1->ecd, m1->last_ecd);

  // UART1_Send_DMA(tx_buffer, length);
}
