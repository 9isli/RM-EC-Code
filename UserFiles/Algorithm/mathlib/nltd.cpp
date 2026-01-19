//
// Created by pomelo on 2025/11/30.
// 通用非线性跟踪微分器(NLTD)模块实现
// 基于二阶系统模型，包含Sat函数和阻尼项
// 可被PID和LADRC算法共用
//

#include "nltd.h"
#include <math.h>

/**
 * @brief 饱和函数(Sat)
 * @details 替代SGN函数，在|x|<delta时为线性，提供平滑过渡
 *          避免在零点附近产生抖振
 */
float NLTD_Sat(float x, float delta) {
  if (delta <= 0.0f) {
    // delta无效时退化为符号函数
    if (x > 0.0f)
      return 1.0f;
    if (x < 0.0f)
      return -1.0f;
    return 0.0f;
  }

  if (x > delta) {
    return 1.0f;
  } else if (x < -delta) {
    return -1.0f;
  } else {
    return x / delta; // 线性区
  }
}

/**
 * @brief 限幅函数
 * @details 将输入值限制在[-limit, limit]范围内
 */
float NLTD_Limit(float val, float limit) {
  if (limit <= 0.0f) {
    return val; // 不限幅
  }
  if (val > limit) {
    return limit;
  } else if (val < -limit) {
    return -limit;
  }
  return val;
}

/**
 * @brief 初始化NLTD
 * @details 基于二阶系统模型: G(s) = ω² / (s² + 2ζωs + ω²)
 */
void NLTD_Init(NLTD_t *nltd, float omega, float zeta, float max_vel, float dt) {
  if (nltd == NULL)
    return;

  // 状态初始化
  nltd->x1 = 0.0f;
  nltd->x2 = 0.0f;

  // 参数设置
  nltd->omega = omega;
  nltd->zeta = zeta;
  nltd->max_vel = max_vel;
  nltd->dt = dt;
}

/**
 * @brief 更新NLTD
 * @details 二阶系统离散化更新
 *          核心公式: accel = ω² * (target - x1) - 2ζω * x2
 *          其中:
 *          - ω² * (target - x1) 是位置误差驱动项
 *          - -2ζω * x2 是阻尼项，用于消除振荡
 *          速度限幅使用Sat函数实现平滑过渡
 */
void NLTD_Update(NLTD_t *nltd, float target) {
  if (nltd == NULL)
    return;

  float omega = nltd->omega;
  float zeta = nltd->zeta;
  float max_vel = nltd->max_vel;
  float dt = nltd->dt;

  // 位置误差
  float pos_error = target - nltd->x1;

  // 计算加速度
  // accel = ω² * (target - x1) - 2ζω * x2
  // 其中 -2ζω * x2 是阻尼项
  float accel = omega * omega * pos_error - 2.0f * zeta * omega * nltd->x2;

  // 速度限幅（使用Sat函数实现平滑过渡）
  if (max_vel > 0.0f) {
    // 预测下一时刻速度
    float next_vel = nltd->x2 + accel * dt;

    // 使用Sat函数实现平滑限幅
    // 当速度接近限幅时，使用Sat函数平滑减速
    float vel_ratio = next_vel / max_vel;
    float soft_limit_zone = 0.1f; // 软限幅区宽度（相对于max_vel的10%）

    if (fabsf(vel_ratio) > 1.0f - soft_limit_zone) {
      // 进入软限幅区，使用Sat函数平滑
      float sat_input = (fabsf(next_vel) - max_vel * (1.0f - soft_limit_zone)) /
                        (max_vel * soft_limit_zone);
      float sat_factor = 1.0f - NLTD_Sat(sat_input, 1.0f) * 0.5f; // 减速因子
      accel *= sat_factor;
    }

    // 更新速度
    nltd->x2 += accel * dt;

    // 硬限幅（最终保护）
    nltd->x2 = NLTD_Limit(nltd->x2, max_vel);
  } else {
    // 无速度限制
    nltd->x2 += accel * dt;
  }

  // 更新位置
  nltd->x1 += nltd->x2 * dt;
}

/**
 * @brief 复位NLTD状态
 * @details 将位置设为初始值，速度清零
 */
void NLTD_Reset(NLTD_t *nltd, float init_pos) {
  if (nltd == NULL)
    return;

  nltd->x1 = init_pos;
  nltd->x2 = 0.0f;
}

/**
 * @brief 复位NLTD状态（带初始速度）
 * @details 将位置和速度设为指定值
 */
void NLTD_Reset_Full(NLTD_t *nltd, float init_pos, float init_vel) {
  if (nltd == NULL)
    return;

  nltd->x1 = init_pos;
  nltd->x2 = init_vel;
}

/**
 * @brief 设置NLTD控制周期
 */
void NLTD_Set_DT(NLTD_t *nltd, float dt) {
  if (nltd == NULL)
    return;

  if (dt > 0.0f) {
    nltd->dt = dt;
  }
}

/**
 * @brief 设置NLTD参数
 */
void NLTD_Set_Params(NLTD_t *nltd, float omega, float zeta, float max_vel) {
  if (nltd == NULL)
    return;

  nltd->omega = omega;
  nltd->zeta = zeta;
  nltd->max_vel = max_vel;
}

/**
 * @brief 获取NLTD当前位置输出
 */
float NLTD_Get_Position(const NLTD_t *nltd) {
  if (nltd == NULL)
    return 0.0f;
  return nltd->x1;
}

/**
 * @brief 获取NLTD当前速度输出
 */
float NLTD_Get_Velocity(const NLTD_t *nltd) {
  if (nltd == NULL)
    return 0.0f;
  return nltd->x2;
}
