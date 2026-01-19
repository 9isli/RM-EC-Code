//
// GM6020 云台电机控制，基于 IMU 反馈
// Created by pomelo on 2025/12/23.
//
//

#ifndef __GM6020_IMU_H
#define __GM6020_IMU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 GM6020 IMU 反馈控制模块
 * @note  初始化 PID、LADRC、滤波器等组件
 */
void GM6020_IMU_Init(void);

/**
 * @brief GM6020 IMU 控制主循环
 * @note  必须以固定周期调用 (推荐 1kHz)
 *        内部完成: 读取 IMU 数据 -> 双环控制 -> 输出电压
 */
void GM6020_IMU_Control_Loop(void);

/**
 * @brief 设置 Yaw 轴目标角度
 * @param target_deg 目标角度 (度), 使用 IMU 累计角度坐标系
 */
void IMU_Yaw_Target(float target_deg);

/**
 * @brief 设置 Pitch 轴目标角度
 * @param target_deg 目标角度 (度), 使用 IMU 绝对角度坐标系
 */
void IMU_Pitch_Target(float target_deg);

/**
 * @brief 摇杆控制yaw轴的接口
 * @param[in] throttle 油门输入 [-1.0, 1.0]，正值向上，负值向下
 */
void GM6020_IMU_Yaw_Speed_Control_Mode(float throttle);

/**
 * @brief 摇杆控制pitch轴的接口
 * @param[in] throttle 油门输入 [-1.0, 1.0]，正值向上，负值向下
 */
void GM6020_IMU_Pitch_Speed_Control_Mode(float throttle);

/**
 * @brief 云台紧急停止
 * @note  立即停止电机输出，设置停止标志
 */
void GM6020_IMU_Emergency_Stop(void);

/**
 * @brief 云台恢复控制
 * @note  清除停止标志，自动对齐目标到当前位置，防止恢复后电机发疯
 */
void GM6020_IMU_Resume_Control(void);

/**
 * @brief 检查云台是否处于停止状态
 * @return 1: 已停止, 0: 正常运行
 */
uint8_t GM6020_IMU_Is_Stopped(void);

#ifdef __cplusplus
}
#endif

#endif //__GM6020_IMU_H
