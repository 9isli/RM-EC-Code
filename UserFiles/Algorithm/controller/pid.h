//
// Created by pomelo on 2025/11/29.
//

#ifndef __PID_H
#define __PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PID 模式选择
 */
typedef enum {
  PID_POSITION = 0, // 位置式 PID: output = Kp*e + Ki*∫e + Kd*de/dt
  PID_DELTA = 1,    // 增量式 PID: Δoutput = Kp*(e-e') + Ki*e + Kd*(e-2e'+e'')
} pid_mode_e;

/**
 * @brief PID 优化环节使能标志位
 * @note  通过位或组合多个优化: PID_Integral_Limit | PID_DerivativeFilter
 */
typedef enum {
  PID_IMPROVE_NONE = 0x00,              // 无优化，纯净 PID
  PID_Integral_Limit = 0x01,            // 积分限幅（含抗饱和）
  PID_Derivative_On_Measurement = 0x02, // 微分先行（对反馈值微分而非误差）
  PID_Trapezoid_Integral = 0x04,        // 梯形积分
  PID_ChangingIntegrationRate = 0x08,   // 变速积分
  PID_DerivativeFilter = 0x10,          // 微分滤波
  PID_OutputFilter = 0x20,              // 输出滤波
  PID_DeadBand = 0x40,                  // 死区
} pid_improve_e;

/**
 * @brief PID 初始化配置结构体
 */
typedef struct {
  // 基本参数
  float kp;
  float ki;
  float kd;
  float maxout;        // 输出限幅
  float integralLimit; // 积分限幅

  // 模式选择
  pid_mode_e mode; // PID 模式 (位置式/增量式)

  // 优化环节
  uint8_t improve; // 优化标志位组合 (PID_Improve_e)

  // 优化参数（仅在对应 Improve 位启用时生效）
  float deadBand; // 死区阈值
  float coefA;    // 变速积分系数 A
  float coefB; // 变速积分系数 B: ITerm *= (A - |err| + B) / A, when B < |err| <
               // A+B
  float derivative_LPF_RC; // 微分滤波器时间常数 (RC = 1/omega_c)
  float output_LPF_RC;     // 输出滤波器时间常数
} pid_config_t;

/**
 * @brief PID 控制器实例
 */
typedef struct {
  // 参数
  float kp;
  float ki;
  float kd;
  float maxout;
  float integralLimit;

  pid_mode_e mode;
  uint8_t improve;

  float deadBand;
  float coefA;
  float coefB;
  float derivative_LPF_RC;
  float output_LPF_RC;

  // 状态变量
  float ref;         // 目标值
  float measure;     // 测量值
  float lastMeasure; // 上次测量值
  float err;         // 当前误差
  float lastErr;     // 上次误差
  float lastLastErr; // 上上次误差（增量式需要）

  float pout;  // P 项输出
  float iout;  // I 项累计输出
  float dout;  // D 项输出
  float iTerm; // 本次 I 项增量

  float output;     // 总输出
  float lastOutput; // 上次输出
  float lastDout;   // 上次 D 项输出（用于滤波）

  float dt; // 控制周期 (秒)

  // 真实时间积分相关（基于 nowtime）
  uint32_t last_update_time; // 上次更新的时间戳（nowtime 值）
  uint8_t use_real_dt;       // 是否使用真实 dt：1=启用，0=禁用（默认启用）
} PID_t;

/**
 * @brief 初始化 PID 控制器
 * @param pid    PID 实例指针
 * @param config 配置结构体指针
 * @param dt     控制周期 (秒)，如 0.001f 表示 1kHz
 */
void PID_Init(PID_t *pid, const pid_config_t *config, float dt);

/**
 * @brief 计算 PID 输出（自动根据 Mode 选择位置式或增量式）
 * @param pid     PID 实例指针
 * @param measure 测量值（反馈）
 * @param ref     目标值（设定）
 * @return PID 输出
 */
float PID_Calculate(PID_t *pid, float measure, float ref);

/**
 * @brief 重置 PID 状态
 * @param pid PID 实例指针
 */
void PID_Reset(PID_t *pid);

/**
 * @brief 设置 PID 参数
 * @param pid PID 实例指针
 * @param kp  比例系数
 * @param ki  积分系数
 * @param kd  微分系数
 */
void PID_SetParams(PID_t *pid, float kp, float ki, float kd);

/**
 * @brief 手动设置积分值（仅位置式有效）
 * @param pid      PID 实例指针
 * @param integral 积分值
 */
void PID_SetIntegral(PID_t *pid, float integral);

/**
 * @brief 切换 PID 模式
 * @param pid  PID 实例指针
 * @param mode 新模式
 * @note  切换模式时会自动重置状态
 */
void PID_SetMode(PID_t *pid, pid_mode_e mode);

/**
 * @brief 启用或禁用真实 dt 模式
 * @param pid    PID 实例指针
 * @param enable 1=启用真实 dt（基于 nowtime），0=禁用（使用固定 dt）
 * @note 在需要自适应控制周期时启用
 */
void PID_Set_Real_DT_Mode(PID_t *pid, uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif //__PID_H
