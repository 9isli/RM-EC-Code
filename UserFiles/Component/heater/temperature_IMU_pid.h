/**
 * @file temperature_IMU_pid.h
 * @brief BMI088 IMU温度控制PID算法头文件
 * @details 使用PID算法对BMI088传感器进行温度控制
 */

#ifndef __TEMPERATURE_IMU_PID_H
#define __TEMPERATURE_IMU_PID_H

#ifdef __cplusplus
extern "C" {

#endif

#include "bmi088_module.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 温度PID控制器结构体
 */
typedef struct {
  float kp; // 比例增益
  float ki; // 积分增益
  float kd; // 微分增益
  float kf; // 前馈增益（用于稳态补偿）

  float target_temp;     // 目标温度（°C）
  float actual_temp;     // 实际温度（°C）
  float error;           // 当前误差
  float last_error;      // 上一次误差
  float integral_sum;    // 积分累加项
  float last_derivative; // 上一次微分项

  float output;     // PID输出（绝对PWM值）
  float max_output; // 输出上限（100% = 5000）
  float min_output; // 输出下限（0%）

  float integral_max;       // 积分累加最大值
  float integral_min;       // 积分累加最小值
  float integral_threshold; // 积分分离阈值

  float dead_zone;           // 死区（误差小于此值时不输出）
  float fast_heat_threshold; // 大温差快速加热阈值（°C）

  float filter_alpha; // 低通滤波系数（0-1）
  float last_output;  // 上一次输出（用于滤波）

  float temp_filter_alpha; // 温度读数滤波系数
  float last_temp;         // 上一次温度读数
} Temperature_PID_t;

/**
 * @brief 初始化温度PID控制器
 * @details 初始化温度控制相关参数和状态
 * @retval 无
 * @note 使用温度控制前必须调用此函数
 */
extern void Temperature_PID_Init(void);

/**
 * @brief 设置目标温度
 * @param[in] target_temp 目标温度（°C）
 * @retval 无
 */
extern void Temperature_PID_Set_Target(float target_temp);

/**
 * @brief 更新温度PID控制
 * @details 计算温度误差并执行PID算法
 * @return PWM占空比（0-5000）
 * @note 建议1ms周期调用（由1kHz温控任务调用）
 */
extern uint16_t Temperature_PID_Loop(void);

/**
 * @brief 设置PID参数
 * @param[in] kp 比例增益
 * @param[in] ki 积分增益
 * @param[in] kd 微分增益
 * @retval 无
 */
extern void Temperature_PID_Set_Params(float kp, float ki, float kd);

/**
 * @brief 设置前馈增益
 * @param[in] kff 前馈增益（每°C误差对应的PWM值）
 * @retval 无
 */
extern void Temperature_PID_Set_Feedforward(float kff);

/**
 * @brief 设置积分抗饱和限制
 * @param[in] integral_max 积分累加最大值
 * @param[in] integral_min 积分累加最小值
 * @retval 无
 */
extern void Temperature_PID_Set_Integral_Limits(float integral_max,
                                                float integral_min);

/**
 * @brief 获取当前温度读数
 * @return 当前温度（°C）
 */
extern float Temperature_PID_Get_Current_Temp(void);

/**
 * @brief 获取当前目标温度
 * @return 目标温度（°C）
 */
extern float Temperature_PID_Get_Target_Temp(void);

/**
 * @brief 获取当前PID输出
 * @return 当前PWM占空比（0-5000）
 */
extern uint16_t Temperature_PID_Get_Output(void);

/**
 * @brief 停止温度控制
 * @details 将输出设置为0
 * @retval 无
 */
extern void Temperature_PID_Stop(void);

#ifdef __cplusplus
}
#endif

#endif // __TEMPERATURE_IMU_PID_H
