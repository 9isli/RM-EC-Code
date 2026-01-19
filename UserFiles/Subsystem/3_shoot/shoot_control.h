//
// Created by pomelo on 2025/12/9
//

#ifndef __SHOOT_CONTROL_H
#define __SHOOT_CONTROL_H

#include "main.h"
#include "robot_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 仅在云台板模式下编译
#if CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL

typedef enum {
  SHOOT_MODE_OFF = 0, // 紧急停止，所有电机输出为零
  SHOOT_MODE_IDLE,    // 摩擦轮保持转速，拨弹盘停止（待命）
  SHOOT_MODE_SINGLE,  // 单发模式，拨弹盘角度环
  SHOOT_MODE_BURST,   // 连发模式，拨弹盘速度环
} Shoot_Mode_e;

typedef enum {
  JAM_STATE_NORMAL = 0, // 正常状态
  JAM_STATE_SUSPECT,    // 疑似卡弹（电流超阈值）
  JAM_STATE_CONFIRM,    // 确认卡弹（持续大电流）
  JAM_STATE_PROCESSING, // 卡弹处理中（反转）
} Jam_State_e;

typedef struct {
  // 当前模式
  Shoot_Mode_e mode;

  // 摩擦轮状态
  int16_t friction_speed_left;  // 左轮目标转速 (RPM)
  int16_t friction_speed_right; // 右轮目标转速 (RPM)
  uint8_t friction_ready;       // 摩擦轮是否就绪

  // 拨弹盘状态
  float trigger_angle;          // 拨弹盘目标角度 (度)
  int16_t trigger_speed;        // 拨弹盘目标转速 (RPM)
  float trigger_fire_rate;      // 射频 (发/秒)
  uint8_t trigger_single_fired; // 单发已触发标志

  // 卡弹检测
  Jam_State_e jam_state;
  uint32_t jam_detect_start_tick;  // 卡弹检测开始时间
  uint32_t jam_process_start_tick; // 卡弹处理开始时间
  float jam_reverse_start_angle;   // 卡弹反转起始角度
} Shoot_State_t;

/**
 * @brief 发射机构初始化
 * @note 初始化摩擦轮和拨弹盘电机控制器
 */
void Shoot_Init(void);

/**
 * @brief 发射机构控制循环
 * @note 在 1kHz 控制任务中调用
 *       更新卡弹检测 FSM
 *       根据模式控制摩擦轮和拨弹盘
 *       调用底层电机控制
 */
void Shoot_Control_Loop(void);

/**
 * @brief 设置发射模式
 * @param mode 发射模式
 */
void Shoot_Set_Mode(Shoot_Mode_e mode);

/**
 * @brief 获取当前发射模式
 * @return 当前发射模式
 */
Shoot_Mode_e Shoot_Get_Mode(void);

/**
 * @brief 设置摩擦轮转速
 * @param speed 目标转速 (RPM)，正值
 * @note 左轮正转，右轮反转（自动处理）
 */
void Shoot_Set_Friction_Speed(int16_t speed);

/**
 * @brief 设置连发射频
 * @param rate 射频 (发/秒)
 */
void Shoot_Set_Fire_Rate(float rate);

/**
 * @brief 触发单发
 * @note 仅在 SHOOT_MODE_SINGLE 模式下有效
 *       带防抖处理，短时间内多次调用只触发一次
 */
void Shoot_Trigger_Single(void);

/**
 * @brief 获取发射机构状态
 * @return 状态结构体指针
 */
const Shoot_State_t *Shoot_Get_State(void);

/**
 * @brief 紧急停止
 * @note 立即停止所有发射电机
 */
void Shoot_Emergency_Stop(void);

/**
 * @brief 检查摩擦轮是否就绪
 * @return 1: 就绪, 0: 未就绪
 */
uint8_t Shoot_Is_Friction_Ready(void);

/**
 * @brief 手动卡弹反转控制
 * @param enable 1=开始反转, 0=停止反转
 * @note 按住时持续反转，松开时停止
 */
void Shoot_Manual_Reverse(uint8_t enable);

/**
 * @brief 处理发射机构遥控器输入
 * @param rc_connected 遥控器是否连接
 * @param master_enabled 总开关是否启用
 * @note 通道5: 摩擦轮速度档位 (321=关闭, 992=15m/s, 1663=18m/s)
 *       通道6: 扳机 (321=不发射, 1663=发射)
 *       通道7: 射频/反转 (321=持续反转, 992=低射频, 1663=高射频)
 *       仅当摩擦轮开启且就绪时扳机才有效
 */
void Shoot_Process_RC_Input(uint8_t rc_connected, uint8_t master_enabled);

#endif // CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL

#ifdef __cplusplus
}
#endif

#endif // __SHOOT_CONTROL_H
