//
// Created by pomelo on 2025/12/2.
//

#ifndef __MATHLIB_H
#define __MATHLIB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== 常量定义 ========================== */

#ifndef PI
#define PI 3.14159265359f
#endif

#ifndef TWO_PI
#define TWO_PI 6.28318530718f
#endif

#ifndef HALF_PI
#define HALF_PI 1.57079632679f
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.01745329252f // π/180
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.2957795131f // 180/π
#endif

#ifndef SQRT2
#define SQRT2 1.41421356237f
#endif

#ifndef SQRT2_INV
#define SQRT2_INV 0.70710678118f // 1/√2
#endif

/* ========================== 基本数学函数 ========================== */

/**
 * @brief 浮点数限幅
 */
float Math_Clamp(float val, float min_val, float max_val);

/**
 * @brief 浮点数对称限幅 [-limit, +limit]
 */
float Math_Limit(float val, float limit);

/**
 * @brief 死区处理
 * @return 如果 |val| < deadband 返回 0，否则返回 val
 */
float Math_Deadband(float val, float deadband);

/**
 * @brief 死区处理（带输出缩放）
 * @note  死区内返回0，死区外线性映射到 [0, max_out]
 */
float Math_Deadband_Scale(float val, float deadband, float max_in,
                          float max_out);

/**
 * @brief 符号函数
 * @return 1.0f (val > 0), -1.0f (val < 0), 0.0f (val == 0)
 */
float Math_Sign(float val);

/**
 * @brief 饱和函数 (Sat)
 * @note  |x| > delta 返回 sign(x)，|x| <= delta 返回 x/delta
 */
float Math_Sat(float x, float delta);

/**
 * @brief 浮点数绝对值
 */
float Math_Abs(float val);

/**
 * @brief 浮点数取最大值
 */
float Math_Max(float a, float b);

/**
 * @brief 浮点数取最小值
 */
float Math_Min(float a, float b);

/**
 * @brief 线性插值
 * @param t 插值系数 [0, 1]
 * @return a + t * (b - a)
 */
float Math_Lerp(float a, float b, float t);

/**
 * @brief 线性映射
 * @note  将 val 从 [in_min, in_max] 映射到 [out_min, out_max]
 */
float Math_Map(float val, float in_min, float in_max, float out_min,
               float out_max);

/* ========================== 角度处理 ========================== */

/**
 * @brief 角度归一化到 [-180, 180) 度
 */
float Math_NormalizeAngle_Deg(float angle);

/**
 * @brief 角度归一化到 [0, 360) 度
 */
float Math_NormalizeAngle_Deg_Positive(float angle);

/**
 * @brief 弧度归一化到 [-π, π)
 */
float Math_NormalizeAngle_Rad(float angle);

/**
 * @brief 弧度归一化到 [0, 2π)
 */
float Math_NormalizeAngle_Rad_Positive(float angle);

/**
 * @brief 计算两个角度之间的最短差值（度）
 * @return 范围 [-180, 180]
 */
float Math_AngleDiff_Deg(float target, float current);

/**
 * @brief 计算两个角度之间的最短差值（弧度）
 * @return 范围 [-π, π]
 */
float Math_AngleDiff_Rad(float target, float current);

/**
 * @brief 度转弧度
 */
float Math_Deg2Rad(float deg);

/**
 * @brief 弧度转度
 */
float Math_Rad2Deg(float rad);

/* ========================== 斜坡函数 ========================== */

/**
 * @brief 斜坡函数（限制变化率）
 * @param current 当前值
 * @param target 目标值
 * @param step 每次最大变化量
 * @return 新的当前值
 */
float Math_Ramp(float current, float target, float step);

/**
 * @brief 斜坡函数（带时间）
 * @param current 当前值
 * @param target 目标值
 * @param rate 变化率（单位/秒）
 * @param dt 时间间隔（秒）
 */
float Math_Ramp_Time(float current, float target, float rate, float dt);

/* ========================== 快速数学函数 ========================== */

/**
 * @brief 快速平方根倒数 (1/√x)
 * @note  Quake III 算法，精度约 1%
 */
float Math_FastInvSqrt(float x);

/**
 * @brief 快速平方根
 */
float Math_FastSqrt(float x);

/**
 * @brief 整数平方
 */
int32_t Math_Square_Int(int32_t x);

/**
 * @brief 浮点数平方
 */
float Math_Square(float x);

/* ========================== 环形缓冲区角度处理 ========================== */

/**
 * @brief 处理编码器过零点（如 0-8191）
 * @param current 当前编码器值
 * @param last 上次编码器值
 * @param max_val 编码器最大值（如 8191）
 * @return 变化量（可正可负）
 */
int32_t Math_EncoderDelta(int32_t current, int32_t last, int32_t max_val);

/**
 * @brief 累计编码器值（处理过零点）
 * @param accumulated 累计值指针
 * @param current 当前编码器值
 * @param last 上次编码器值
 * @param max_val 编码器最大值
 */
void Math_EncoderAccumulate(int64_t *accumulated, int32_t current, int32_t last,
                            int32_t max_val);

#ifdef __cplusplus
}
#endif

#endif //__MATHLIB_H
