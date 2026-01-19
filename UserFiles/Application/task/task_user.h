//
// Created by pomelo on 2025/11/17.
//

#ifndef __TASK_USER_H
#define __TASK_USER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 系统初始化任务，初始化外设和控制算法
 * @param argument 未使用
 * @note 初始化完成后自删除
 */
void System_InitTask(void *argument);

/**
 * @brief 1kHz 统一控制任务
 * @param argument 未使用
 * @note 根据 CURRENT_BOARD_ROLE 执行不同逻辑：
 *       - 云台板：遥控器读取、运动决策、云台控制、底盘命令发送
 *       - 底盘板：接收云台命令、M3508 速度环、IMU 数据上报
 */
void Control_1kHz_Task(void *argument);

/**
 * @brief IMU 姿态更新任务，周期性更新全局姿态角
 * @param argument 未使用
 */
void IMU_Task(void *argument);

/**
 * @brief IMU 温度控制任务，维持 IMU 在目标温度附近
 * @param argument 未使用
 */
void Temperature_ControlTask(void *argument);

/**
 * @brief 监控任务，读取 IMU 温度和电机状态等信息
 * @param argument 未使用
 * @note 仅用于调试，可选
 */
void Monitor_Task(void *argument);

// 系统初始化完成标志，由 System_InitTask 置位
extern volatile uint8_t g_system_init_done;

// 双板通信就绪标志，由 System_InitTask 置位
// 1 = 对端板在线，0 = 未检测到或超时
extern volatile uint8_t g_dual_board_comm_ready;

// IMU 温度就绪与是否需要达温标志
extern volatile uint8_t g_imu_temp_ready; // 1 = 已达到目标温度附近
extern volatile uint8_t
    g_imu_temp_check_enable; // 1 = 需要达温后再启动 IMU 任务

// 全局可见的 IMU 三个姿态角（单位：度）
extern float g_imu_yaw;   // 偏航角
extern float g_imu_pitch; // 俯仰角
extern float g_imu_roll;  // 滚转角
// 全局可见的三个角速度与原数据（单位：度/s）
extern float g_omega_yaw;
extern float g_omega_pitch;
extern float g_omega_roll;
extern float g_rawomega_yaw;
extern float g_rawomega_pitch;
extern float g_rawomega_roll;
// 全局可见的yaw累计角度
extern float g_totalyaw;

#ifdef __cplusplus
}
#endif

#endif //__TASK_USER_H