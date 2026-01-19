//
// Created by pomelo on 2025/12/4
//

#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 暂时没用到这部分，大部分由NORMAL即可实现
typedef enum {
  CHASSIS_MODE_STOP = 0,  // 停止模式
  CHASSIS_MODE_NORMAL,    // 普通模式（底盘跟随云台）
  CHASSIS_MODE_SPIN,      // 小陀螺模式（底盘自旋，云台保持方向）
  CHASSIS_MODE_NO_FOLLOW, // 不跟随模式（底盘独立控制）
  CHASSIS_MODE_SOFT_STOP, // 软停止模式（平滑减速中）
} Chassis_Mode_e;

typedef struct {
  // 目标速度（底盘坐标系）
  float vx_set; // 前后速度 (m/s)，正为前
  float vy_set; // 左右速度 (m/s)，正为左
  float wz_set; // 自转角速度 (rad/s)，正为逆时针
  float offset_rad;

  // 四个轮子的目标转速 (RPM)
  int16_t wheel_rpm[4];

  // 当前模式
  Chassis_Mode_e mode;

  // 云台角度偏移（用于坐标变换）
  float gimbal_yaw_offset; // 云台相对底盘的角度 (rad)
} Chassis_t;

/**
 * @brief 底盘控制模块初始化
 */
void Chassis_Init(void);

/**
 * @brief 设置底盘运动模式
 * @param mode 运动模式
 */
void Chassis_Set_Mode(Chassis_Mode_e mode);

/**
 * @brief 设置底盘目标速度（底盘坐标系）
 * @param vx 前后速度 (m/s)
 * @param vy 左右速度 (m/s)
 * @param wz 自转角速度 (rad/s)
 */
void Chassis_Set_Velocity(float vx, float vy, float wz);

/**
 * @brief 底盘控制循环
 * @note 在 1kHz 控制任务中调用
 *       运动学解算（Vx/Vy/Wz -> 4轮转速）
 *       调用 M3508_Set_RPM 设置电机目标
 */
void Chassis_Control_Loop(void);

/**
 * @brief 获取底盘状态指针
 */
const Chassis_t *Chassis_Get_State(void);

/**
 * @brief 底盘急停
 */
void Chassis_Stop(void);

#ifdef __cplusplus
}
#endif

#endif // __CHASSIS_CONTROL_H
