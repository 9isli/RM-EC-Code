//
// Created by pomelo on 2025/11/30.
// 斜坡函数模块实现
// 移植自 USTC-RoboWalker 的 alg_slope
//

#include "slope.h"
#include <math.h>

/**
 * @brief 取绝对值（内部辅助函数）
 */
static inline float Slope_Abs(float x) { return (x >= 0.0f) ? x : -x; }

/**
 * @brief 初始化斜坡函数
 */
void Slope_Init(Slope_t *slope, float increase_value, float decrease_value,
                Slope_First_e first_mode) {
  if (slope == NULL)
    return;

  slope->out = 0.0f;
  slope->now_planning = 0.0f;
  slope->now_real = 0.0f;
  slope->target = 0.0f;
  slope->increase_value = increase_value;
  slope->decrease_value = decrease_value;
  slope->first_mode = first_mode;
}

/**
 * @brief 设置目标值
 */
void Slope_Set_Target(Slope_t *slope, float target) {
  if (slope == NULL)
    return;
  slope->target = target;
}

/**
 * @brief 设置当前真实值
 */
void Slope_Set_Now_Real(Slope_t *slope, float now_real) {
  if (slope == NULL)
    return;
  slope->now_real = now_real;
}

/**
 * @brief 设置增量参数
 */
void Slope_Set_Values(Slope_t *slope, float increase_value,
                      float decrease_value) {
  if (slope == NULL)
    return;
  slope->increase_value = increase_value;
  slope->decrease_value = decrease_value;
}

/**
 * @brief 斜坡函数调整值, 计算周期取决于调用者
 *
 */
void Slope_Update(Slope_t *slope) {
  if (slope == NULL)
    return;

  float target = slope->target;
  float now_planning = slope->now_planning;
  float now_real = slope->now_real;
  float increase = slope->increase_value;
  float decrease = slope->decrease_value;

  // 规划为当前真实值优先的额外逻辑
  if (slope->first_mode == SLOPE_FIRST_REAL) {
    if ((target >= now_real && now_real >= now_planning) ||
        (target <= now_real && now_real <= now_planning)) {
      slope->out = now_real;
    }
  }

  if (now_planning > 0.0f) {
    if (target > now_planning) {
      // 正值加速
      if (Slope_Abs(now_planning - target) > increase) {
        slope->out += increase;
      } else {
        slope->out = target;
      }
    } else if (target < now_planning) {
      // 正值减速
      if (Slope_Abs(now_planning - target) > decrease) {
        slope->out -= decrease;
      } else {
        slope->out = target;
      }
    }
  } else if (now_planning < 0.0f) {
    if (target < now_planning) {
      // 负值加速
      if (Slope_Abs(now_planning - target) > increase) {
        slope->out -= increase;
      } else {
        slope->out = target;
      }
    } else if (target > now_planning) {
      // 负值减速
      if (Slope_Abs(now_planning - target) > decrease) {
        slope->out += decrease;
      } else {
        slope->out = target;
      }
    }
  } else {
    if (target > now_planning) {
      // 0值正加速
      if (Slope_Abs(now_planning - target) > increase) {
        slope->out += increase;
      } else {
        slope->out = target;
      }
    } else if (target < now_planning) {
      // 0值负加速
      if (Slope_Abs(now_planning - target) > increase) {
        slope->out -= increase;
      } else {
        slope->out = target;
      }
    }
  }

  // 善后工作
  now_planning = slope->out;
}

/**
 * @brief 获取输出值
 */
float Slope_Get_Out(const Slope_t *slope) {
  if (slope == NULL)
    return 0.0f;
  return slope->out;
}

/**
 * @brief 获取当前规划值
 */
float Slope_Get_Planning(const Slope_t *slope) {
  if (slope == NULL)
    return 0.0f;
  return slope->now_planning;
}

/**
 * @brief 复位斜坡函数状态
 */
void Slope_Reset(Slope_t *slope, float init_value) {
  if (slope == NULL)
    return;

  slope->out = init_value;
  slope->now_planning = init_value;
  slope->now_real = init_value;
  slope->target = init_value;
}
