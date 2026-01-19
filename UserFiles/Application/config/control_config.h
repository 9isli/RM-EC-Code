#ifndef __CONTROL_CONFIG_H
#define __CONTROL_CONFIG_H

#if CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL

/*====================整车控制与遥控器====================*/

#define ROBOT_CONTROL_DT 0.001f // 控制周期

// 遥控器配置
#define RC_DEADZONE 0.05f            // 死区阈值
#define RC_DISCONNECT_TIMEOUT_MS 500 // 断连超时 (ms)
#define SOFT_STOP_DURATION_MS 500    // 软停止时间 (ms)

// 遥控器通道映射 (WBUS 接收机实测)
// J1=通道0, J2=通道1, J3=通道2, J4=通道3
#define RC_CH_VX 2        // 前后平移 (左摇杆纵向 = J3)
#define RC_CH_VY 3        // 左右平移 (左摇杆横向 = J4)
#define RC_CH_YAW 0       // 云台 Yaw (右摇杆横向 = J1)
#define RC_CH_PITCH 1     // 云台 Pitch (右摇杆纵向 = J2)
#define RC_CH_MASTER_SW 4 // 总开关: 321=关闭锁定, 其他=启用

// 底盘/云台模式通道映射
// 底盘模式: 321=逆时针小陀螺, 992=跟随模式, 1663=顺时针小陀螺
#define RC_CH_CHASSIS_MODE 8

// 摩擦轮速度档位: 321=关闭, 992=15m/s, 1663=18m/s
#define RC_CH_FRICTION_SPEED 5
// 射击触发: 321=不发射, 1663=发射
#define RC_CH_SHOOT_TRIGGER 6
// 射频/反转: 321=持续反转, 992=低射频, 1663=高射频
#define RC_CH_FIRE_RATE_REVERSE 7

// 灵敏度设置
#define RC_VX_SENSITIVITY 1.0f // m/s
#define RC_VY_SENSITIVITY 1.0f // m/s

// 云台手动旋转速度限制 (°/s)
#define YAW_MAX_MANUAL_SPEED 300.0f   // Yaw 轴最大手动旋转速度
#define PITCH_MAX_MANUAL_SPEED 200.0f // Pitch 轴最大手动旋转速度

// 底盘跟随云台控制参数
#define CHASSIS_FOLLOW_KP 2.0f       // P 参数
#define CHASSIS_FOLLOW_KF 0.42f      // 前馈系数
#define CHASSIS_FOLLOW_DEADZONE 1.0f // 死区（度）
#define CHASSIS_FOLLOW_MAX_WZ 12.0f  // 最大角速度（rad/s）

// 底盘跟随斜坡函数参数
// 单位：rad/s 每毫秒，即 rad/s²/1000
// 对应加速度 = value * 1000 rad/s²
#define CHASSIS_FOLLOW_SLOPE_INCREASE 0.050f // 加速斜率（每周期增量）
#define CHASSIS_FOLLOW_SLOPE_DECREASE 0.050f // 减速斜率（每周期减量）

// 用于预测底盘旋转后的角度，避免转弯时路径画圈
#define CHASSIS_COORD_FEEDFORWARD 0.6f

#define WZ_KP 1.0f
#define WZ_KI 0.0f
#define WZ_KD 0.0f
#define WZ_MAX_OUT 0.0f
#define WZ_MAX_I 0.0f

#endif // CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL

#endif // __CONTROL_CONFIG_H
