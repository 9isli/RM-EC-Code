//
// 底盘运动控制模块实现
// Created by pomelo on 2025/12/4
//

#include "chassis_control.h"
#include "dual_board_comm.h"
#include "m3508_rpm.h"
#include "omni.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>

/*====================内部变量====================*/
static Chassis_t chassis;
static uint8_t is_initialized = 0;

/*====================运动学参数====================*/

// 速度到电机转速的转换系数
#define CHASSIS_SPEED_TO_RPM                                                   \
  (60.0f / (2.0f * 3.14159265f * CHASSIS_WHEEL_RADIUS) * M3508_GEAR_RATIO)

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
/*====================内部函数====================*/

/**
 * @brief 限幅函数
 */
static float Limit_Float(float value, float min_val, float max_val) {
  if (value > max_val)
    return max_val;
  if (value < min_val)
    return min_val;
  return value;
}

/*====================外部接口实现====================*/

void Chassis_Init(void) {
  // 初始化底盘状态
  chassis.vx_set = 0.0f;
  chassis.vy_set = 0.0f;
  chassis.wz_set = 0.0f;
  chassis.mode = CHASSIS_MODE_STOP;
  chassis.gimbal_yaw_offset = 0.0f;

  for (int i = 0; i < 4; i++) {
    chassis.wheel_rpm[i] = 0;
  }

  // 初始化 omni 运动学库
  Omni_Init(CHASSIS_WHEEL_RADIUS * 1000.0f, CHASSIS_RADIUS * 1000.0f,
            CHASSIS_MAX_WHEEL_RPM);

  is_initialized = 1;
}

void Chassis_Set_Mode(Chassis_Mode_e mode) { chassis.mode = mode; }

void Chassis_Set_Velocity(float vx, float vy, float wz) {
  // 限幅
  chassis.vx_set = Limit_Float(vx, -CHASSIS_MAX_VX, CHASSIS_MAX_VX);
  chassis.vy_set = Limit_Float(vy, -CHASSIS_MAX_VY, CHASSIS_MAX_VY);
  chassis.wz_set = Limit_Float(wz, -CHASSIS_MAX_WZ, CHASSIS_MAX_WZ);
}

void Chassis_Control_Loop(void) {
  if (!is_initialized)
    return;

  // 停止模式下直接输出零
  if (chassis.mode == CHASSIS_MODE_STOP) {
#if CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL
    // 云台板：发送零速命令给底盘板
    int16_t zero_rpm[4] = {0, 0, 0, 0};
    DualBoard_Send_Chassis_Cmd(zero_rpm, CHASSIS_MODE_STOP);
#else
    // 底盘板：直接控制电机
    M3508_Set_RPM(0, 0, 0, 0);
#endif
    return;
  }

  // X型全向轮逆运动学解算 (使用 Omni 库)
  Omni_Kinematics_Calculate(chassis.vx_set, chassis.vy_set, chassis.wz_set,
                            chassis.wheel_rpm);

#if CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL
  // 云台板：通过 CAN2 发送目标转速给底盘板
  DualBoard_Send_Chassis_Cmd(chassis.wheel_rpm, chassis.mode);
#else
  // 底盘板：直接设置电机目标转速
  M3508_Set_RPM(chassis.wheel_rpm[0], chassis.wheel_rpm[1],
                chassis.wheel_rpm[2], chassis.wheel_rpm[3]);
#endif
}

const Chassis_t *Chassis_Get_State(void) { return &chassis; }

void Chassis_Stop(void) {
  chassis.mode = CHASSIS_MODE_STOP;
  chassis.vx_set = 0.0f;
  chassis.vy_set = 0.0f;
  chassis.wz_set = 0.0f;

  for (int i = 0; i < 4; i++) {
    chassis.wheel_rpm[i] = 0;
  }

#if CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL
  // 云台板：发送停止命令
  int16_t zero_rpm[4] = {0, 0, 0, 0};
  DualBoard_Send_Chassis_Cmd(zero_rpm, CHASSIS_MODE_STOP);
#else
  // 底盘板：直接停止电机
  M3508_Set_RPM(0, 0, 0, 0);
#endif
}
