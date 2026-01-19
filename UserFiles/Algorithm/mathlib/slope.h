//
// Created by pomelo on 2025/11/30.
// 斜坡函数模块，用于速度规划等
// 移植自 USTC-RoboWalker 的 alg_slope
//

#ifndef __SLOPE_H
#define __SLOPE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 规划优先类型枚举
 * @details SLOPE_FIRST_REAL: 真实值优先，当真实值夹在规划值和目标值之间时，直接采用真实值
 *          SLOPE_FIRST_TARGET: 目标值优先，即硬规划，不考虑真实值
 */
typedef enum {
  SLOPE_FIRST_REAL = 0, // 真实值优先（推荐用于底盘跟随等需要反馈的场景）
  SLOPE_FIRST_TARGET,   // 目标值优先（硬规划）
} Slope_First_e;

/**
 * @brief 斜坡函数结构体
 * @details 用于限制变量的变化速率，实现平滑加减速
 */
typedef struct {
  // 输出值
  float out;

  // 规划优先类型
  Slope_First_e first_mode;

  // 当前规划值（内部状态）
  float now_planning;

  // 当前真实值（外部反馈）
  float now_real;

  // 绝对值增量，一次计算周期的最大增加值
  float increase_value;

  // 绝对值减量，一次计算周期的最大减少值
  float decrease_value;

  // 目标值
  float target;
} Slope_t;

/**
 * @brief 初始化斜坡函数
 * @param[out] slope 斜坡函数结构体指针
 * @param[in] increase_value 增长最大幅度（每周期）
 * @param[in] decrease_value 降低最大幅度（每周期）
 * @param[in] first_mode 规划优先类型
 * @note 对于1kHz控制频率，increase_value = 目标加速度 * 0.001
 *       例如：最大角加速度 12.57 rad/s²，则 increase_value = 0.01257
 */
void Slope_Init(Slope_t *slope, float increase_value, float decrease_value,
                Slope_First_e first_mode);

/**
 * @brief 设置目标值
 * @param[in,out] slope 斜坡函数结构体指针
 * @param[in] target 目标值
 */
void Slope_Set_Target(Slope_t *slope, float target);

/**
 * @brief 设置当前真实值（外部反馈）
 * @param[in,out] slope 斜坡函数结构体指针
 * @param[in] now_real 当前真实值
 * @note 仅在 SLOPE_FIRST_REAL 模式下有效
 */
void Slope_Set_Now_Real(Slope_t *slope, float now_real);

/**
 * @brief 设置增量参数
 * @param[in,out] slope 斜坡函数结构体指针
 * @param[in] increase_value 增长最大幅度
 * @param[in] decrease_value 降低最大幅度
 */
void Slope_Set_Values(Slope_t *slope, float increase_value,
                      float decrease_value);

/**
 * @brief 更新斜坡函数（每控制周期调用一次）
 * @param[in,out] slope 斜坡函数结构体指针
 * @note 调用前需先设置 target 和 now_real（如果使用 SLOPE_FIRST_REAL 模式）
 */
void Slope_Update(Slope_t *slope);

/**
 * @brief 获取斜坡函数输出值
 * @param[in] slope 斜坡函数结构体指针
 * @return 当前输出值
 */
float Slope_Get_Out(const Slope_t *slope);

/**
 * @brief 获取当前规划值（内部状态）
 * @param[in] slope 斜坡函数结构体指针
 * @return 当前规划值
 */
float Slope_Get_Planning(const Slope_t *slope);

/**
 * @brief 复位斜坡函数状态
 * @param[in,out] slope 斜坡函数结构体指针
 * @param[in] init_value 初始值
 */
void Slope_Reset(Slope_t *slope, float init_value);

#ifdef __cplusplus
}
#endif

#endif //__SLOPE_H

