//
// Created by pomelo on 2025/12/2.
// 常用数学方法库实现
//

#include "mathlib.h"
#include <math.h>

extern "C" {

/* ========================== 基本数学函数 ========================== */

/**
 * @brief 浮点数限幅
 * @param[in] val 输入值
 * @param[in] min_val 最小值
 * @param[in] max_val 最大值
 * @return 限幅后的值，范围 [min_val, max_val]
 * @note 如果 val < min_val 返回 min_val，如果 val > max_val 返回 max_val
 */
float Math_Clamp(float val, float min_val, float max_val) {
  if (val < min_val)
    return min_val;
  if (val > max_val)
    return max_val;
  return val;
}

/**
 * @brief 浮点数对称限幅
 * @param[in] val 输入值
 * @param[in] limit 限幅绝对值
 * @return 限幅后的值，范围 [-|limit|, +|limit|]
 * @note 等价于 Math_Clamp(val, -limit, limit)
 */
float Math_Limit(float val, float limit) {
  float abs_limit = fabsf(limit);
  if (val > abs_limit)
    return abs_limit;
  if (val < -abs_limit)
    return -abs_limit;
  return val;
}

/**
 * @brief 死区处理
 * @param[in] val 输入值
 * @param[in] deadband 死区阈值
 * @return 如果 |val| < |deadband| 返回 0，否则返回 val
 * @note 用于消除传感器或遥控器的零点漂移
 */
float Math_Deadband(float val, float deadband) {
  if (fabsf(val) < fabsf(deadband)) {
    return 0.0f;
  }
  return val;
}

/**
 * @brief 死区处理（带输出缩放）
 * @param[in] val 输入值
 * @param[in] deadband 死区阈值
 * @param[in] max_in 输入最大值
 * @param[in] max_out 输出最大值
 * @return 死区内返回 0，死区外线性映射到 [-max_out, max_out]
 * @note 将 [deadband, max_in] 映射到 [0, max_out]，保持输出连续性
 * @example Math_Deadband_Scale(joystick, 10, 100, 1000)
 *          输入 0~10 输出 0，输入 10~100 输出 0~1000
 */
float Math_Deadband_Scale(float val, float deadband, float max_in,
                          float max_out) {
  float abs_val = fabsf(val);
  float abs_deadband = fabsf(deadband);

  if (abs_val < abs_deadband) {
    return 0.0f;
  }

  float sign = (val > 0.0f) ? 1.0f : -1.0f;
  float scaled = (abs_val - abs_deadband) / (max_in - abs_deadband) * max_out;

  return sign * Math_Clamp(scaled, 0.0f, max_out);
}

/**
 * @brief 符号函数
 * @param[in] val 输入值
 * @return 1.0f (val > 0), -1.0f (val < 0), 0.0f (val == 0)
 * @note 数学定义的 sgn(x) 函数
 */
float Math_Sign(float val) {
  if (val > 0.0f)
    return 1.0f;
  if (val < 0.0f)
    return -1.0f;
  return 0.0f;
}

/**
 * @brief 饱和函数 (Sat)
 * @param[in] x 输入值
 * @param[in] delta 线性区宽度
 * @return |x| > delta 返回 sign(x)，|x| <= delta 返回 x/delta
 * @note 平滑版的符号函数，在零点附近是线性的，避免抖振
 *       常用于 LADRC 的非线性控制律
 */
float Math_Sat(float x, float delta) {
  if (delta <= 0.0f) {
    return Math_Sign(x);
  }
  if (x > delta)
    return 1.0f;
  if (x < -delta)
    return -1.0f;
  return x / delta;
}

/**
 * @brief 浮点数绝对值
 * @param[in] val 输入值
 * @return |val|
 * @note 简单封装，避免引入 math.h
 */
float Math_Abs(float val) { return (val >= 0.0f) ? val : -val; }

/**
 * @brief 浮点数取最大值
 * @param[in] a 第一个值
 * @param[in] b 第二个值
 * @return max(a, b)
 */
float Math_Max(float a, float b) { return (a > b) ? a : b; }

/**
 * @brief 浮点数取最小值
 * @param[in] a 第一个值
 * @param[in] b 第二个值
 * @return min(a, b)
 */
float Math_Min(float a, float b) { return (a < b) ? a : b; }

/**
 * @brief 线性插值
 * @param[in] a 起始值
 * @param[in] b 结束值
 * @param[in] t 插值系数 [0, 1]
 * @return a + t * (b - a)
 * @note t=0 返回 a，t=1 返回 b，t=0.5 返回 (a+b)/2
 */
float Math_Lerp(float a, float b, float t) { return a + t * (b - a); }

/**
 * @brief 线性映射
 * @param[in] val 输入值
 * @param[in] in_min 输入范围最小值
 * @param[in] in_max 输入范围最大值
 * @param[in] out_min 输出范围最小值
 * @param[in] out_max 输出范围最大值
 * @return 映射后的值
 * @note 将 val 从 [in_min, in_max] 线性映射到 [out_min, out_max]
 * @example Math_Map(50, 0, 100, 0, 1000) = 500
 */
float Math_Map(float val, float in_min, float in_max, float out_min,
               float out_max) {
  if (fabsf(in_max - in_min) < 1e-10f) {
    return out_min;
  }
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* ========================== 角度处理 ========================== */

/**
 * @brief 角度归一化到 [-180, 180) 度
 * @param[in] angle 输入角度（度）
 * @return 归一化后的角度，范围 [-180, 180)
 * @note 用于角度差值计算前的归一化
 */
float Math_NormalizeAngle_Deg(float angle) {
  while (angle >= 180.0f)
    angle -= 360.0f;
  while (angle < -180.0f)
    angle += 360.0f;
  return angle;
}

/**
 * @brief 角度归一化到 [0, 360) 度
 * @param[in] angle 输入角度（度）
 * @return 归一化后的角度，范围 [0, 360)
 * @note 用于需要正角度的场合
 */
float Math_NormalizeAngle_Deg_Positive(float angle) {
  while (angle >= 360.0f)
    angle -= 360.0f;
  while (angle < 0.0f)
    angle += 360.0f;
  return angle;
}

/**
 * @brief 弧度归一化到 [-π, π)
 * @param[in] angle 输入角度（弧度）
 * @return 归一化后的角度，范围 [-π, π)
 */
float Math_NormalizeAngle_Rad(float angle) {
  while (angle >= PI)
    angle -= TWO_PI;
  while (angle < -PI)
    angle += TWO_PI;
  return angle;
}

/**
 * @brief 弧度归一化到 [0, 2π)
 * @param[in] angle 输入角度（弧度）
 * @return 归一化后的角度，范围 [0, 2π)
 */
float Math_NormalizeAngle_Rad_Positive(float angle) {
  while (angle >= TWO_PI)
    angle -= TWO_PI;
  while (angle < 0.0f)
    angle += TWO_PI;
  return angle;
}

/**
 * @brief 计算两个角度之间的最短差值（度）
 * @param[in] target 目标角度（度）
 * @param[in] current 当前角度（度）
 * @return 最短角度差，范围 [-180, 180]
 * @note 自动处理 0°/360° 过零点问题
 * @example Math_AngleDiff_Deg(10, 350) = 20（而非 -340）
 */
float Math_AngleDiff_Deg(float target, float current) {
  float diff = target - current;
  return Math_NormalizeAngle_Deg(diff);
}

/**
 * @brief 计算两个角度之间的最短差值（弧度）
 * @param[in] target 目标角度（弧度）
 * @param[in] current 当前角度（弧度）
 * @return 最短角度差，范围 [-π, π]
 * @note 自动处理 0/2π 过零点问题
 */
float Math_AngleDiff_Rad(float target, float current) {
  float diff = target - current;
  return Math_NormalizeAngle_Rad(diff);
}

/**
 * @brief 度转弧度
 * @param[in] deg 角度（度）
 * @return 角度（弧度）
 * @note rad = deg * π / 180
 */
float Math_Deg2Rad(float deg) { return deg * DEG_TO_RAD; }

/**
 * @brief 弧度转度
 * @param[in] rad 角度（弧度）
 * @return 角度（度）
 * @note deg = rad * 180 / π
 */
float Math_Rad2Deg(float rad) { return rad * RAD_TO_DEG; }

/* ========================== 斜坡函数 ========================== */

/**
 * @brief 斜坡函数（限制变化率）
 * @param[in] current 当前值
 * @param[in] target 目标值
 * @param[in] step 每次最大变化量
 * @return 新的当前值
 * @note 用于平滑过渡，避免突变
 * @example 从 0 到 100，step=10，需要 10 次调用才能到达
 */
float Math_Ramp(float current, float target, float step) {
  float diff = target - current;
  float abs_step = fabsf(step);

  if (fabsf(diff) <= abs_step) {
    return target;
  }

  if (diff > 0.0f) {
    return current + abs_step;
  } else {
    return current - abs_step;
  }
}

/**
 * @brief 斜坡函数（带时间）
 * @param[in] current 当前值
 * @param[in] target 目标值
 * @param[in] rate 变化率（单位/秒）
 * @param[in] dt 时间间隔（秒）
 * @return 新的当前值
 * @note step = rate * dt
 * @example Math_Ramp_Time(0, 100, 50, 0.001) 每毫秒变化 0.05
 */
float Math_Ramp_Time(float current, float target, float rate, float dt) {
  float step = fabsf(rate) * dt;
  return Math_Ramp(current, target, step);
}

/* ========================== 快速数学函数 ========================== */

/**
 * @brief 快速平方根倒数 (1/√x)
 * @param[in] x 输入值（必须 > 0）
 * @return 1/√x 的近似值
 * @note Quake III 经典算法，精度约 1%，速度比 1/sqrtf(x) 快 4 倍
 * @warning 仅适用于对精度要求不高的场合
 */
float Math_FastInvSqrt(float x) {
  float xhalf = 0.5f * x;
  int32_t i = *(int32_t *)&x;
  i = 0x5f3759df - (i >> 1);
  x = *(float *)&i;
  x = x * (1.5f - xhalf * x * x);
  return x;
}

/**
 * @brief 快速平方根
 * @param[in] x 输入值（必须 >= 0）
 * @return √x 的近似值
 * @note 基于快速平方根倒数算法
 */
float Math_FastSqrt(float x) {
  if (x <= 0.0f)
    return 0.0f;
  return x * Math_FastInvSqrt(x);
}

/**
 * @brief 整数平方
 * @param[in] x 输入值
 * @return x²
 */
int32_t Math_Square_Int(int32_t x) { return x * x; }

/**
 * @brief 浮点数平方
 * @param[in] x 输入值
 * @return x²
 * @note 比 powf(x, 2) 快
 */
float Math_Square(float x) { return x * x; }

/* ========================== 编码器处理 ========================== */

/**
 * @brief 处理编码器过零点
 * @param[in] current 当前编码器值
 * @param[in] last 上次编码器值
 * @param[in] max_val 编码器最大值（如 8191 表示 0-8191）
 * @return 变化量（可正可负）
 * @note 自动处理从 max_val 跳到 0 或从 0 跳到 max_val 的情况
 * @example 编码器 0-8191，从 8100 变到 100：
 *          Math_EncoderDelta(100, 8100, 8191) = 192（正向转了 192）
 */
int32_t Math_EncoderDelta(int32_t current, int32_t last, int32_t max_val) {
  int32_t delta = current - last;
  int32_t half = (max_val + 1) / 2;

  if (delta > half) {
    delta -= (max_val + 1);
  } else if (delta < -half) {
    delta += (max_val + 1);
  }

  return delta;
}

/**
 * @brief 累计编码器值（处理过零点）
 * @param[in,out] accumulated 累计值指针
 * @param[in] current 当前编码器值
 * @param[in] last 上次编码器值
 * @param[in] max_val 编码器最大值
 * @note 将增量累加到 accumulated，可用于计算总转动量
 * @example
 *   int64_t total = 0;
 *   Math_EncoderAccumulate(&total, new_ecd, old_ecd, 8191);
 */
void Math_EncoderAccumulate(int64_t *accumulated, int32_t current, int32_t last,
                            int32_t max_val) {
  int32_t delta = Math_EncoderDelta(current, last, max_val);
  *accumulated += delta;
}

} // extern "C"
