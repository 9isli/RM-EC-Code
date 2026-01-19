//
// 机器人参数配置文件
// 用于统一管理机械参数、控制参数和遥控器配置
// 有些单位可能暂时还是错误的，还没有校对
// 如果你不清楚你的修改的会带来什么后果，就不要修改
// Created by pomelo on 2025/12/6
//

#ifndef __ROBOT_CONFIG_H
#define __ROBOT_CONFIG_H

#include "chassis_config.h"
#include "control_config.h"
#include "gimbal_config.h"
#include "shoot_config.h"

/*====================双板角色配置====================*/

// 控制板角色定义
#define BOARD_ROLE_GIMBAL 0  // 云台板（接收遥控器，运行主控逻辑）
#define BOARD_ROLE_CHASSIS 1 // 底盘板（接收云台命令，控制 M3508，上报 IMU）

// 控制板角色选择
#define CURRENT_BOARD_ROLE BOARD_ROLE_GIMBAL

/*====================初始化配置====================*/

// 是否在初始化阶段执行电机温度自检：
// 正常使用保持为 1
// 调试阶段跳过电机在线检测，改为 0
#define INIT_ENABLE_MOTOR_CHECK 0

// 是否在初始化阶段等待双板通信建立：
// 双板部署时设为 1
// 单板调试时设为 0
#define INIT_WAIT_DUAL_BOARD_COMM 1

// 电机温度自检参数
#define MOTOR_TEMP_MIN 5U            // 电机最低有效温度 (°C)
#define MOTOR_TEMP_MAX 120U          // 电机最高有效温度 (°C)
#define MOTOR_CHECK_TIMEOUT_MS 1000U // 电机检测超时时间 (ms)

// 双板通信初始化参数
#define DUAL_BOARD_INIT_TIMEOUT_MS 3000U // 双板通信建立超时时间 (ms)

// IMU 温度等待参数
#define IMU_TEMP_TIMEOUT_MS 30000U // IMU 达温等待超时 (ms)
#define IMU_TEMP_TARGET 45.0f      // IMU 目标温度 (°C)
#define IMU_TEMP_TOLERANCE 5.0f    // IMU 温度容差 (°C)

/*====================双板通信配置====================*/

// 心跳超时时间 (ms) - 设置为发送间隔的 10 倍以确保稳定
#define HEARTBEAT_TIMEOUT_MS 500

// 心跳发送间隔 (ms)
#define HEARTBEAT_INTERVAL_MS 50

/*====================底盘 IMU 配置====================*/

// 是否启用底盘 IMU（如果没有第二个 IMU，设为 0）
#define CHASSIS_IMU_ENABLED 1

// 底盘 IMU 类型选择
// 0: 本地硬件 IMU（需要第二个 SPI 接口的 BMI088）
// 1: CAN 接收模式（从底盘主控板通过 CAN 接收 IMU 数据）
#define CHASSIS_IMU_TYPE 1

// CAN 通信超时时间 (ms)
#define CHASSIS_IMU_CAN_TIMEOUT 100

/*====================云台 IMU 姿态解算配置====================*/

// 姿态解算 PI 控制器参数
#define IMU_AHRS_KP 0.5f   // 比例系数
#define IMU_AHRS_KI 0.001f // 积分系数

// 姿态滤波器参数
#define IMU_FILTER_CUTOFF_FREQ 30.0f  // 低通滤波截止频率 (Hz)
#define IMU_FILTER_SAMPLE_FREQ 200.0f // 采样频率 (Hz)

/*====================IMU 温度控制 PID====================*/

// 温度 PID 参数
#define TEMP_PID_KP 25.0f
#define TEMP_PID_KI 112.0f
#define TEMP_PID_KD 8.0f
#define TEMP_PID_KF 53.0f // 前馈系数

// 温度控制目标和限制
#define TEMP_TARGET 40.0f       // 目标温度 (°C)
#define TEMP_MAX_OUTPUT 5000.0f // PWM 最大输出
#define TEMP_MIN_OUTPUT 0.0f    // PWM 最小输出

// 积分抗饱和限制
#define TEMP_INTEGRAL_MAX 2000.0f
#define TEMP_INTEGRAL_MIN -500.0f

// 温度控制死区和阈值
#define TEMP_DEAD_ZONE 0.1f           // 误差死区 (°C)
#define TEMP_INTEGRAL_THRESHOLD 0.5f  // 积分分离阈值 (°C)
#define TEMP_FAST_HEAT_THRESHOLD 5.0f // 快速加热阈值 (°C)

// 滤波系数
#define TEMP_FILTER_ALPHA 0.6f        // 输出滤波系数
#define TEMP_SENSOR_FILTER_ALPHA 0.6f // 温度传感器滤波系数

#endif // __ROBOT_CONFIG_H
