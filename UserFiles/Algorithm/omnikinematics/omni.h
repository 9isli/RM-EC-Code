//
// X型全向轮运动学库
// Created by pomelo on 2025/11/18
// 重构为纯数学计算库，不依赖硬件
//

#ifndef __OMNI_H
#define __OMNI_H

#include <stdbool.h>
#include <stdint.h>

// 运动学常量配置
#define PI 3.14159265359f
#define SQRT2 1.41421356237f
#define SQRT2_INV 0.70710678118f // 1/√2，用于45°计算

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 里程计数据结构体（正运动学输出）
 */
typedef struct {
  float vx;    // X方向速度 (m/s)
  float vy;    // Y方向速度 (m/s)
  float omega; // 角速度 (rad/s)
  float x;     // X方向累计位移 (m)
  float y;     // Y方向累计位移 (m)
  float theta; // 累计旋转角度 (rad)
} OmniOdometry_t;

/**
 * @brief 初始化全向轮运动学模块
 * @param[in] wheel_radius 轮子半径 (mm)
 * @param[in] chassis_radius 底盘半径 (mm)
 * @param[in] max_rpm 电机最大转速 (RPM)
 */
extern void Omni_Init(float wheel_radius, float chassis_radius,
                      int16_t max_rpm);

/**
 * @brief 逆运动学解算：从速度和角速度计算电机转速
 * @param[in] vx X方向速度 (m/s)
 * @param[in] vy Y方向速度 (m/s)
 * @param[in] omega 角速度 (rad/s)
 * @param[out] out_rpm 输出的四个电机转速
 */
extern void Omni_Kinematics_Calculate(float vx, float vy, float omega,
                                      int16_t *out_rpm);

/**
 * @brief 正运动学解算：从电机转速计算底盘速度
 * @param[in] motor_rpm 四个电机的实际转速 (RPM)，顺序为[M1, M2, M3, M4]
 * @param[out] vx X方向速度 (m/s)
 * @param[out] vy Y方向速度 (m/s)
 * @param[out] omega 角速度 (rad/s)
 * @note 用于里程计、速度反馈等场景
 */
extern void Omni_Forward_Kinematics(const int16_t motor_rpm[4], float *vx,
                                    float *vy, float *omega);

/**
 * @brief 更新里程计（累计位移和角度）
 * @param[in] motor_rpm 四个电机的实际转速 (RPM)
 * @param[in] dt 时间间隔 (秒)
 * @param[out] odom 里程计数据结构体指针
 * @note 需要周期性调用以累计位移
 */
extern void Omni_Update_Odometry(const int16_t motor_rpm[4], float dt,
                                 OmniOdometry_t *odom);

/**
 * @brief 重置里程计
 * @param[out] odom 里程计数据结构体指针
 */
extern void Omni_Reset_Odometry(OmniOdometry_t *odom);

#ifdef __cplusplus
}
#endif

#endif //__OMNI_H
