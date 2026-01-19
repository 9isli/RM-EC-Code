//
// Created by pomelo on 2025/11/30.
//

#ifndef __NLTD_H
#define __NLTD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 通用非线性跟踪微分器(NLTD)结构体
 * @details 基于二阶系统模型: G(s) = ω² / (s² + 2ζωs + ω²)
 *          包含Sat函数平滑限幅和阻尼项，可被PID和LADRC共用
 */
typedef struct {
  // 状态变量
  float x1; // 跟踪信号（位置）
  float x2; // 跟踪信号导数（速度）

  // 二阶系统参数
  float omega;   // 自然频率(rad/s)，越大响应越快
  float zeta;    // 阻尼比（1.0为临界阻尼，推荐0.7-1.2）
  float max_vel; // 最大速度限制（0表示不限制）

  // 控制周期
  float dt; // 控制周期（秒）
} NLTD_t;

/**
 * @brief 饱和函数(Sat)
 * @details 替代SGN函数，在|x|<delta时为线性，提供平滑过渡
 *          x > delta  → 返回 1.0
 *          x < -delta → 返回 -1.0
 *          |x| <= delta → 返回 x/delta（线性区）
 * @param[in] x 输入值
 * @param[in] delta 线性区宽度
 * @return Sat输出，范围[-1, 1]
 */
float NLTD_Sat(float x, float delta);

/**
 * @brief 限幅函数
 * @param[in] val 输入值
 * @param[in] limit 限幅绝对值
 * @return 限幅后的值，范围[-limit, limit]
 */
float NLTD_Limit(float val, float limit);

/**
 * @brief 初始化NLTD
 * @param[out] nltd NLTD结构体指针
 * @param[in] omega 自然频率(rad/s)（推荐5-50，越大响应越快）
 * @param[in] zeta 阻尼比（1.0为临界阻尼，推荐0.7-1.2）
 * @param[in] max_vel 最大速度限制（0表示不限制）
 * @param[in] dt 控制周期（秒，如0.001f表示1kHz）
 * @note 参数建议:
 *       - LADRC角度环: omega=5-15, zeta=1.0, max_vel=8000
 *       - LADRC速度环: omega=20-50, zeta=1.0, max_vel=0
 *       - PID角度环: omega=5-10, zeta=1.0, max_vel=8000
 */
void NLTD_Init(NLTD_t *nltd, float omega, float zeta, float max_vel, float dt);

/**
 * @brief 更新NLTD
 * @param[in,out] nltd NLTD结构体指针
 * @param[in] target 目标值
 * @note 核心公式: accel = ω² * (target - x1) - 2ζω * x2
 *       其中 -2ζω * x2 是阻尼项，用于消除振荡
 *       速度限幅使用Sat函数实现平滑过渡
 */
void NLTD_Update(NLTD_t *nltd, float target);

/**
 * @brief 复位NLTD状态
 * @param[in,out] nltd NLTD结构体指针
 * @param[in] init_pos 初始位置值
 */
void NLTD_Reset(NLTD_t *nltd, float init_pos);

/**
 * @brief 复位NLTD状态（带初始速度）
 * @param[in,out] nltd NLTD结构体指针
 * @param[in] init_pos 初始位置值
 * @param[in] init_vel 初始速度值
 */
void NLTD_Reset_Full(NLTD_t *nltd, float init_pos, float init_vel);

/**
 * @brief 设置NLTD控制周期
 * @param[in,out] nltd NLTD结构体指针
 * @param[in] dt 新的控制周期（秒）
 */
void NLTD_Set_DT(NLTD_t *nltd, float dt);

/**
 * @brief 设置NLTD参数
 * @param[in,out] nltd NLTD结构体指针
 * @param[in] omega 自然频率
 * @param[in] zeta 阻尼比
 * @param[in] max_vel 最大速度限制
 */
void NLTD_Set_Params(NLTD_t *nltd, float omega, float zeta, float max_vel);

/**
 * @brief 获取NLTD当前位置输出
 * @param[in] nltd NLTD结构体指针
 * @return 当前位置（x1）
 */
float NLTD_Get_Position(const NLTD_t *nltd);

/**
 * @brief 获取NLTD当前速度输出
 * @param[in] nltd NLTD结构体指针
 * @return 当前速度（x2）
 */
float NLTD_Get_Velocity(const NLTD_t *nltd);

#ifdef __cplusplus
}
#endif

#endif //__NLTD_H
