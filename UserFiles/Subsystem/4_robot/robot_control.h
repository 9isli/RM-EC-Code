//
// Created by pomelo on 2025/12/4
//

#ifndef __ROBOT_CONTROL_H
#define __ROBOT_CONTROL_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  ROBOT_MODE_STOP = 0,    // 停止模式：所有电机停止
  ROBOT_MODE_NORMAL,      // 普通模式：底盘跟随云台方向移动
  ROBOT_MODE_NO_FOLLOW,   // 解耦模式：底盘不跟随云台旋转，但可以平移
  ROBOT_MODE_SPIN,        // 小陀螺模式：底盘自旋，云台保持世界坐标系方向
  ROBOT_MODE_SPIN_MOVE,   // 小陀螺+平移：底盘自旋的同时可以平移
  ROBOT_MODE_GIMBAL_ONLY, // 仅云台模式：底盘不动，只控制云台
} Robot_Mode_e;

typedef struct {
  // 当前模式
  Robot_Mode_e mode;

  // 遥控器输入（归一化 [-1, 1]）
  float rc_vx;    // 前后输入
  float rc_vy;    // 左右输入
  float rc_yaw;   // 云台 Yaw 输入
  float rc_pitch; // 云台 Pitch 输入

  // 云台角度（用于坐标变换）
  float gimbal_yaw_rad;   // 云台 Yaw 角度 (rad)
  float gimbal_pitch_rad; // 云台 Pitch 角度 (rad)

  // 小陀螺模式参数
  float spin_speed; // 小陀螺角速度 (rad/s)

  // 连接状态
  uint8_t rc_connected; // 遥控器是否连接
} Robot_t;

/**
 * @brief 整车控制模块初始化
 */
void Robot_Init(void);

/**
 * @brief 设置机器人运动模式
 * @param mode 运动模式
 */
void Robot_Set_Mode(Robot_Mode_e mode);

/**
 * @brief 获取当前运动模式
 */
Robot_Mode_e Robot_Get_Mode(void);

/**
 * @brief 设置小陀螺角速度
 * @param speed 角速度 (rad/s)，正值为逆时针
 */
void Robot_Set_Spin_Speed(float speed);

/**
 * @brief 整车控制主循环
 * @note 在 1kHz 控制任务中调用
 *       读取遥控器输入
 *       根据模式计算底盘和云台目标
 *       调用底盘和云台控制接口
 */
void Robot_Control_Loop(void);

/**
 * @brief 获取机器人状态指针
 */
const Robot_t *Robot_Get_State(void);

/**
 * @brief 紧急停止
 */
void Robot_Emergency_Stop(void);

/**
 * @brief 获取云台与底盘的偏差角度
 * @return 偏差角度 (度)，-180 ~ 180
 * @note 云台指向底盘正前方时为 0
 *       云台向右偏时为正，向左偏时为负
 */
float Robot_Get_Gimbal_Offset_Angle(void);

#ifdef __cplusplus
}
#endif

#endif // __ROBOT_CONTROL_H
