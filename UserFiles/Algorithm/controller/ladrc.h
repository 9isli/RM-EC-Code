//
// Created by pomelo on 2025/11/30.
//

#ifndef __LADRC_H
#define __LADRC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LADRC 模式选择
 * @note  一阶用于速度环，二阶用于位置环
 */
typedef enum {
  LADRC_FIRST_ORDER = 1,  // 一阶 LADRC（二阶 LESO）：适用于速度环
  LADRC_SECOND_ORDER = 2, // 二阶 LADRC（三阶 LESO）：适用于位置环
} ladrc_order_e;

/**
 * @brief LADRC 优化环节使能标志位
 * @note  通过位或组合多个优化: LADRC_LESO_Trapezoid | LADRC_DerivativeFilter
 */
typedef enum {
  LADRC_IMPROVE_NONE = 0x00,              // 无优化，纯净 LADRC
  LADRC_LESO_Trapezoid = 0x01,            // LESO 梯形积分（提高观测精度）
  LADRC_Feedback_LPF = 0x02,              // 反馈信号低通滤波（抑制测量噪声）
  LADRC_Derivative_On_Measurement = 0x04, // PD 微分先行（对反馈微分而非误差）
  LADRC_DerivativeFilter = 0x08,          // PD 微分滤波（滤除高频噪声）
  LADRC_Disturbance_Limit = 0x10,         // 扰动幅度衰减/限幅
  LADRC_DeadBand = 0x20,                  // 死区处理
} ladrc_improve_e;

/**
 * @brief LADRC 初始化配置结构体
 */
typedef struct {
  // 系统阶数
  ladrc_order_e order; // 一阶或二阶 LADRC

  // LESO 参数
  float w0; // LESO 带宽 (rad/s)，越大观测越快但噪声放大
  float b0; // 控制增益（系统模型参数）

  // PD 控制器参数
  float kp; // 比例系数
  float kd; // 微分系数（仅二阶 LADRC 使用）

  // 输出限幅
  float maxout; // 输出限幅

  // 优化环节
  uint8_t improve; // 优化标志位组合 (ladrc_improve_e)

  // 优化参数（仅在对应 Improve 位启用时生效）
  float deadBand;          // 死区阈值
  float feedback_LPF_RC;   // 反馈滤波器时间常数 (RC = 1/omega_c)
  float derivative_LPF_RC; // 微分滤波器时间常数
  float disturbance_limit; // 扰动限幅值（绝对值）
  float disturbance_decay; // 扰动衰减系数（0-1，1为不衰减）
} ladrc_config_t;

/**
 * @brief LADRC 控制器实例
 */
typedef struct {
  ladrc_order_e order; // 系统阶数

  // LESO 参数
  float w0; // LESO 带宽
  float b0; // 控制增益

  // PD 参数
  float kp; // 比例系数
  float kd; // 微分系数

  // 限幅
  float maxout; // 输出限幅

  // 优化配置
  uint8_t improve;         // 优化标志位
  float deadBand;          // 死区阈值
  float feedback_LPF_RC;   // 反馈滤波器时间常数
  float derivative_LPF_RC; // 微分滤波器时间常数
  float disturbance_limit; // 扰动限幅值
  float disturbance_decay; // 扰动衰减系数

  // 二阶 LESO（一阶 LADRC）: z1 ≈ y, z2 ≈ f
  // 三阶 LESO（二阶 LADRC）: z1 ≈ y, z2 ≈ ẏ, z3 ≈ f
  float z1; // 状态观测值 1（输出估计）
  float z2; // 状态观测值 2（一阶：扰动；二阶：速度估计）
  float z3; // 状态观测值 3（仅二阶：扰动估计）

  // LESO 增益（内部计算）
  float beta1; // LESO 增益 1
  float beta2; // LESO 增益 2
  float beta3; // LESO 增益 3（仅二阶）

  float ref;              // 目标值
  float measure;          // 测量值（原始）
  float filtered_measure; // 滤波后测量值
  float lastMeasure;      // 上次测量值
  float err;              // 当前误差
  float lastErr;          // 上次误差

  float u0;    // 控制律输出（补偿前）
  float u;     // 最终控制输出（补偿后）
  float lastU; // 上次控制输出

  float dout;     // 微分项输出
  float lastDout; // 上次微分输出（用于滤波）

  float output;     // 最终输出
  float lastOutput; // 上次输出

  float dt;                  // 控制周期 (秒)
  uint32_t last_update_time; // 上次更新时间戳
  uint8_t use_real_dt;       // 是否使用真实 dt

  float last_z1_dot; // 上次 z1 导数
  float last_z2_dot; // 上次 z2 导数
  float last_z3_dot; // 上次 z3 导数（仅二阶）
} LADRC_t;

/**
 * @brief 初始化 LADRC 控制器
 * @param ladrc   LADRC 实例指针
 * @param config  配置结构体指针
 * @param dt      控制周期 (秒)，如 0.001f 表示 1kHz
 */
void LADRC_Init(LADRC_t *ladrc, const ladrc_config_t *config, float dt);

/**
 * @brief 计算 LADRC 输出
 * @param ladrc   LADRC 实例指针
 * @param measure 测量值（反馈）
 * @param ref     目标值（设定）
 * @return LADRC 输出
 */
float LADRC_Calculate(LADRC_t *ladrc, float measure, float ref);

/**
 * @brief 重置 LADRC 状态
 * @param ladrc LADRC 实例指针
 */
void LADRC_Reset(LADRC_t *ladrc);

/**
 * @brief 重置 LADRC 状态（带初始值）
 * @param ladrc       LADRC 实例指针
 * @param init_state  初始状态值（通常为当前测量值）
 */
void LADRC_Reset_With_State(LADRC_t *ladrc, float init_state);

/**
 * @brief 设置 LESO 参数
 * @param ladrc LADRC 实例指针
 * @param w0    LESO 带宽
 * @param b0    控制增益
 */
void LADRC_SetLESO(LADRC_t *ladrc, float w0, float b0);

/**
 * @brief 设置 PD 参数
 * @param ladrc LADRC 实例指针
 * @param kp    比例系数
 * @param kd    微分系数
 */
void LADRC_SetPD(LADRC_t *ladrc, float kp, float kd);

/**
 * @brief 设置输出限幅
 * @param ladrc  LADRC 实例指针
 * @param maxout 最大输出
 */
void LADRC_SetMaxout(LADRC_t *ladrc, float maxout);

/**
 * @brief 切换 LADRC 阶数
 * @param ladrc LADRC 实例指针
 * @param order 新阶数
 * @note  切换阶数时会自动重置状态
 */
void LADRC_SetOrder(LADRC_t *ladrc, ladrc_order_e order);

/**
 * @brief 启用或禁用真实 dt 模式
 * @param ladrc  LADRC 实例指针
 * @param enable 1=启用真实 dt（基于 nowtime），0=禁用（使用固定 dt）
 */
void LADRC_Set_Real_DT_Mode(LADRC_t *ladrc, uint8_t enable);

/**
 * @brief 获取 LESO 扰动估计值
 * @param ladrc LADRC 实例指针
 * @return 扰动估计值（一阶返回 z2，二阶返回 z3）
 */
float LADRC_GetDisturbance(const LADRC_t *ladrc);

/**
 * @brief 获取 LESO 速度估计值（仅二阶有效）
 * @param ladrc LADRC 实例指针
 * @return 速度估计值 z2
 */
float LADRC_GetVelocity(const LADRC_t *ladrc);

#ifdef __cplusplus
}
#endif

#endif //__LADRC_H
