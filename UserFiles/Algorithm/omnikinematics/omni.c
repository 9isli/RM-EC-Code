//
// X型全向轮运动学库
// Created by pomelo on 2025/11/18
// 重构为纯数学计算库，不依赖硬件
//

#include "omni.h"
#include <math.h>
#include <string.h>

// 减速箱参数：电机轴 / 输出轴
#define GEAR_MOTOR 3591.0f // 分子（电机侧）
#define GEAR_OUTPUT 187.0f // 分母（输出轴侧）
#define GEAR_RATIO (GEAR_MOTOR / GEAR_OUTPUT)

// 全向轮参数（统一使用米和 RPM）
static float g_wheel_radius = 0.076f;   // 轮子半径 (m)
static float g_chassis_radius = 0.230f; // 底盘半径 (m)
static int16_t g_max_rpm = 8000;        // 电机最大转速 (RPM)

// 预计算的转换系数（在 Init 时计算）
static float g_vel_to_rpm = 0.0f; // 线速度(m/s) -> 电机转速(RPM)
static float g_rpm_to_vel = 0.0f; // 电机转速(RPM) -> 线速度(m/s)

/**
 * @brief 初始化全向轮运动学模块
 * @param[in] wheel_radius_mm 轮子半径 (mm)
 * @param[in] chassis_radius_mm 底盘半径 (mm)
 * @param[in] max_rpm 电机最大转速 (RPM)
 */
void Omni_Init(float wheel_radius_mm, float chassis_radius_mm,
               int16_t max_rpm) {
  // 将 mm 转换为 m 存储
  g_wheel_radius = wheel_radius_mm / 1000.0f;
  g_chassis_radius = chassis_radius_mm / 1000.0f;
  g_max_rpm = max_rpm;

  // 预计算转换系数
  // 线速度(m/s) -> 电机转速(RPM): rpm = v * 60 * GEAR_RATIO / (2π * r_wheel)
  g_vel_to_rpm = 60.0f * GEAR_RATIO / (2.0f * PI * g_wheel_radius);
  // 电机转速(RPM) -> 线速度(m/s): v = rpm * 2π * r_wheel / (60 * GEAR_RATIO)
  g_rpm_to_vel = (2.0f * PI * g_wheel_radius) / (60.0f * GEAR_RATIO);
}

/**
 * @brief 逆运动学解算：从速度和角速度计算电机转速
 *
 * 底盘结构（俯视图，右手系，与IMU对应）：
 *
 *        +X (前进方向，M1和M2之间)
 *         ↑
 *    M1    \   /    M2
 *     135°  \ /  45°
 *    Y+ ←────O
 *     225°  / \  315°
 *    M4    /   \    M3
 *
 * 坐标系定义（俯视图）：
 *   - X轴：指向机器人前方（正值=前进，1,2轮夹角方向）
 *   - Y轴：指向机器人左方（正值=左移）
 *   - Wz：逆时针为正（正值=左转）
 *
 * 电机安装说明：
 *   - 所有电机正电流时顺时针旋转（从上方看）
 *   - 左侧轮子(1,4)顺时针=向后滚，需要取反
 *   - 右侧轮子(2,3)顺时针=向前滚，不取反
 *
 * 最终公式：
 *   v_M1 = +(vx + vy - ω*R)  // 1: LF
 *   v_M2 = -(vx - vy + ω*R)  // 2: RF
 *   v_M3 = -(vx + vy + ω*R)  // 3: RB
 *   v_M4 = +(vx - vy - ω*R)  // 4: LB
 *
 * @param vx X 方向速度 (m/s)
 * @param vy Y 方向速度 (m/s)
 * @param omega 角速度 (rad/s)
 * @param[out] out_rpm 输出的四个电机转速
 */
void Omni_Kinematics_Calculate(float vx, float vy, float omega,
                               int16_t *out_rpm) {
  // 计算角速度对轮子的贡献
  float wz_contrib = omega * g_chassis_radius;

  // 逆运动学解算（输出为轮子线速度 m/s）
  float v_M1 = +(+vx + vy - wz_contrib); // 1: 左前轮 (LF)
  float v_M2 = -(+vx - vy + wz_contrib); // 2: 右前轮 (RF)
  float v_M3 = -(+vx + vy + wz_contrib); // 3: 右后轮 (RB)
  float v_M4 = +(+vx - vy - wz_contrib); // 4: 左后轮 (LB)

  // 转换为电机转速 (RPM)
  float rpm_M1 = v_M1 * g_vel_to_rpm;
  float rpm_M2 = v_M2 * g_vel_to_rpm;
  float rpm_M3 = v_M3 * g_vel_to_rpm;
  float rpm_M4 = v_M4 * g_vel_to_rpm;

  // 若有轮的转速超出最大范围，则按比例整体缩放，保持运动方向不变
  float max_abs = fabsf(rpm_M1);
  if (fabsf(rpm_M2) > max_abs)
    max_abs = fabsf(rpm_M2);
  if (fabsf(rpm_M3) > max_abs)
    max_abs = fabsf(rpm_M3);
  if (fabsf(rpm_M4) > max_abs)
    max_abs = fabsf(rpm_M4);

  if (max_abs > g_max_rpm) {
    float scale = (float)g_max_rpm / max_abs;
    rpm_M1 *= scale;
    rpm_M2 *= scale;
    rpm_M3 *= scale;
    rpm_M4 *= scale;
  }

  // 输出
  out_rpm[0] = (int16_t)rpm_M1;
  out_rpm[1] = (int16_t)rpm_M2;
  out_rpm[2] = (int16_t)rpm_M3;
  out_rpm[3] = (int16_t)rpm_M4;
}

/**
 * @brief 正运动学解算：从电机转速计算底盘速度
 *
 * 正运动学是逆运动学的反向过程。
 * 给定四个轮的转速，求解底盘的 vx, vy, omega。
 *
 * @param[in] motor_rpm 四个电机的实际转速 (RPM)，顺序为[M1, M2, M3, M4]
 * @param[out] vx X方向速度 (m/s)
 * @param[out] vy Y方向速度 (m/s)
 * @param[out] omega 角速度 (rad/s)
 */
void Omni_Forward_Kinematics(const int16_t motor_rpm[4], float *vx, float *vy,
                             float *omega) {
  // 从物理电机转速还原逻辑轮速（考虑安装方向）
  float rpm_M1 = (float)motor_rpm[0];
  float rpm_M2 = -(float)motor_rpm[1]; // 电机2安装方向取反
  float rpm_M3 = (float)motor_rpm[2];
  float rpm_M4 = -(float)motor_rpm[3]; // 电机4安装方向取反

  // 转换为线速度 (m/s)
  float v_M1 = rpm_M1 * g_rpm_to_vel;
  float v_M2 = rpm_M2 * g_rpm_to_vel;
  float v_M3 = rpm_M3 * g_rpm_to_vel;
  float v_M4 = rpm_M4 * g_rpm_to_vel;

  // 正运动学解算
  float inv_2sqrt2 = SQRT2_INV * 0.5f; // 1/(2√2) = √2/4

  *vx = (v_M2 + v_M3 - v_M1 - v_M4) * inv_2sqrt2;
  *vy = (v_M1 + v_M2 - v_M3 - v_M4) * inv_2sqrt2;
  *omega = (v_M1 + v_M2 + v_M3 + v_M4) / (4.0f * g_chassis_radius);
}

/**
 * @brief 更新里程计（累计位移和角度）
 * @param[in] motor_rpm 四个电机的实际转速 (RPM)
 * @param[in] dt 时间间隔 (秒)
 * @param[out] odom 里程计数据结构体指针
 */
void Omni_Update_Odometry(const int16_t motor_rpm[4], float dt,
                          OmniOdometry_t *odom) {
  // 计算当前速度
  float vx, vy, omega;
  Omni_Forward_Kinematics(motor_rpm, &vx, &vy, &omega);

  // 更新速度
  odom->vx = vx;
  odom->vy = vy;
  odom->omega = omega;

  // 累计角度
  odom->theta += omega * dt;

  // 将速度从机体坐标系转换到世界坐标系，然后累计位移
  // 使用中点法：用当前角度的中点来近似积分
  float mid_theta = odom->theta - omega * dt * 0.5f;
  float cos_theta = cosf(mid_theta);
  float sin_theta = sinf(mid_theta);

  // 世界坐标系下的速度
  float vx_world = vx * cos_theta - vy * sin_theta;
  float vy_world = vx * sin_theta + vy * cos_theta;

  // 累计位移
  odom->x += vx_world * dt;
  odom->y += vy_world * dt;
}

/**
 * @brief 重置里程计
 * @param[out] odom 里程计数据结构体指针
 */
void Omni_Reset_Odometry(OmniOdometry_t *odom) {
  memset(odom, 0, sizeof(OmniOdometry_t));
}
