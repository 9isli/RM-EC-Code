//
// Created by pomelo on 2025/10/9.
//

#include "temperature_IMU_pid.h"
#include "robot_config.h"
#include "tim.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

static Temperature_PID_t temp_pid;

/**
 * @brief 温度PID计算
 * @details 读取BMI088温度传感器，计算温度误差，执行PID算法
 * @param[in,out] pid 指向温度PID结构体的指针
 * @return 计算得到的PWM输出值
 * @note 此函数包含温度传感器读取和低通滤波
 */
static float Temperature_PID_Calc(Temperature_PID_t *pid) {
  float p_term, i_term, d_term, ff_term; // PID 各项
  float error_abs;                       // 误差绝对值
  float filtered_output;                 // 滤波后的输出
  float temp_celsius;                    // 摄氏温度
  int32_t temp_raw;                      // 传感器原始温度数据

  // 读取 BMI088 的温度
  if (BMI088_Module_Read_Temp(&temp_raw) == BMI08X_OK) {
    temp_celsius = temp_raw / 1000.0f;

    // 对温度进行一阶低通滤波
    pid->actual_temp = pid->temp_filter_alpha * temp_celsius +
                       (1.0f - pid->temp_filter_alpha) * pid->last_temp;
    pid->last_temp = pid->actual_temp;
  }

  // 计算误差
  pid->error = pid->target_temp - pid->actual_temp;
  error_abs = (pid->error > 0.0f) ? pid->error : -pid->error;

  // 大温差快速加热：当前温度远低于目标温度时直接全功率输出
  if (pid->error > pid->fast_heat_threshold) {
    pid->integral_sum = 0.0f;
    pid->last_error = pid->error;
    pid->output = pid->max_output;
    pid->last_output = pid->max_output;
    return pid->max_output;
  }

  // 死区控制：误差太小时保持原输出
  if (error_abs < pid->dead_zone) {
    return pid->last_output;
  }

  // 比例项
  p_term = pid->kp * pid->error;

  // 积分项（带限幅与分离）
  if (error_abs > pid->integral_threshold) {
    pid->integral_sum += pid->ki * pid->error;

    // 防积分饱和
    if (pid->integral_sum > pid->integral_max)
      pid->integral_sum = pid->integral_max;
    if (pid->integral_sum < pid->integral_min)
      pid->integral_sum = pid->integral_min;
  } else {
    pid->integral_sum *= 0.95f;
  }
  i_term = pid->integral_sum;

  // 微分项
  d_term = pid->kd * (pid->error - pid->last_error);
  pid->last_derivative = d_term;

  // 前馈项
  ff_term = pid->kf * pid->error;

  // 总输出
  pid->output = p_term + i_term + d_term + ff_term;

  // 输出滤波
  filtered_output = pid->filter_alpha * pid->output +
                    (1.0f - pid->filter_alpha) * pid->last_output;

  // 输出限幅
  if (filtered_output > pid->max_output)
    filtered_output = pid->max_output;
  if (filtered_output < pid->min_output)
    filtered_output = pid->min_output;

  pid->last_error = pid->error;
  pid->last_output = filtered_output;

  return filtered_output;
}

/**
 * @brief 初始化温度PID控制器
 * @details 设置PID参数并初始化状态变量，启动PWM输出
 * @retval 无
 * @note 在使用温控功能前必须调用一次
 */
void Temperature_PID_Init(void) {
  temp_pid.kp = TEMP_PID_KP;
  temp_pid.ki = TEMP_PID_KI;
  temp_pid.kd = TEMP_PID_KD;
  temp_pid.kf = TEMP_PID_KF;

  temp_pid.target_temp = TEMP_TARGET;
  temp_pid.actual_temp = 25.0f;
  temp_pid.last_temp = 25.0f;

  temp_pid.error = 0.0f;
  temp_pid.last_error = 0.0f;
  temp_pid.integral_sum = 0.0f;
  temp_pid.last_derivative = 0.0f;

  temp_pid.output = 0.0f;
  temp_pid.last_output = 0.0f;

  temp_pid.max_output = TEMP_MAX_OUTPUT;
  temp_pid.min_output = TEMP_MIN_OUTPUT;

  // 防积分饱和限幅
  temp_pid.integral_max = TEMP_INTEGRAL_MAX;
  temp_pid.integral_min = TEMP_INTEGRAL_MIN;

  temp_pid.dead_zone = TEMP_DEAD_ZONE;
  temp_pid.integral_threshold = TEMP_INTEGRAL_THRESHOLD;

  temp_pid.fast_heat_threshold = TEMP_FAST_HEAT_THRESHOLD;

  temp_pid.filter_alpha = TEMP_FILTER_ALPHA;

  temp_pid.temp_filter_alpha = TEMP_SENSOR_FILTER_ALPHA;

  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
}

/**
 * @brief 设置目标温度
 * @param[in] target_temp 目标温度（°C），范围20-80°C
 * @retval 无
 */
void Temperature_PID_Set_Target(float target_temp) {
  // 限定目标温度范围（20~80°C）
  if (target_temp < 20.0f)
    target_temp = 20.0f;
  if (target_temp > 80.0f)
    target_temp = 80.0f;

  temp_pid.target_temp = target_temp;
}

/**
 * @brief 更新温度PID控制
 * @details 读取温度、计算PID输出并更新PWM
 * @return 当前PWM占空比（0-5000）
 * @note 建议每1ms周期调用一次（由1kHz温控任务调用）
 */
uint16_t Temperature_PID_Loop(void) {
  uint16_t pwm_value;

  // 计算 PID 输出
  Temperature_PID_Calc(&temp_pid);

  // 转换为 PWM 输出值
  pwm_value = (uint16_t)(temp_pid.output);

  // 更新 TIM10 通道 1（PF6）的 PWM 输出
  TIM10->CCR1 = (pwm_value);

  return pwm_value;
}

/**
 * @brief 设置PID参数
 * @param[in] kp 比例增益
 * @param[in] ki 积分增益
 * @param[in] kd 微分增益
 * @retval 无
 */
void Temperature_PID_Set_Params(float kp, float ki, float kd) {
  temp_pid.kp = kp;
  temp_pid.ki = ki;
  temp_pid.kd = kd;
}

/**
 * @brief 设置积分抗饱和限制
 * @param[in] integral_max 积分累加最大值
 * @param[in] integral_min 积分累加最小值
 * @retval 无
 */
void Temperature_PID_Set_Integral_Limits(float integral_max,
                                         float integral_min) {
  temp_pid.integral_max = integral_max;
  temp_pid.integral_min = integral_min;
}

/**
 * @brief 获取当前温度读数
 * @return 当前温度（°C）
 */
float Temperature_PID_Get_Current_Temp(void) { return temp_pid.actual_temp; }

/**
 * @brief 获取当前目标温度
 * @return 目标温度（°C）
 */
float Temperature_PID_Get_Target_Temp(void) { return temp_pid.target_temp; }

/**
 * @brief 获取当前PID输出
 * @return 当前PWM占空比（0-5000）
 */
uint16_t Temperature_PID_Get_Output(void) {
  return (uint16_t)(temp_pid.output);
}

/**
 * @brief 停止温度控制
 * @details 将PWM输出设为0，停止加热
 * @retval 无
 */
void Temperature_PID_Stop(void) {
  temp_pid.output = 0.0f;
  temp_pid.last_output = 0.0f;
  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
}
