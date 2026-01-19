#ifndef __GIMBAL_CONFIG_H
#define __GIMBAL_CONFIG_H

#if CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL

/*====================GM6020 云台电机控制====================*/

#define GM6020_CONTROL_DT 0.001f
#define GM6020_ECD_RANGE 8192.0f
#define GM6020_ECD_HALF 4096.0f

// 是否启用云台上电复位功能
// 启用后，云台会在上电时自动回到指定的编码器刻度
#define GIMBAL_ENABLE_STARTUP_RESET 1

// 云台上电复位目标刻度 (0 ~ 8191)
// 需要根据实际机械安装位置校准：
#define GIMBAL_YAW_RESET_ECD 760    // Yaw 轴复位刻度
#define GIMBAL_PITCH_RESET_ECD 7681 // Pitch 轴复位刻度

// 复位速度限制 (ECD/s)，越小复位越平稳
#define GIMBAL_RESET_SPEED 2000.0f

// 复位完成判定阈值 (ECD)，误差小于此值视为复位完成
#define GIMBAL_RESET_THRESHOLD 50.0f

// ----------------Yaw 轴参数----------------
// NLTD 跟踪微分器 (二阶系统参数)
#define YAW_NLTD_OMEGA 21.0f      // 响应频率 (Hz)，越大响应越快
#define YAW_NLTD_ZETA 1.0f        // 阻尼比，1.0=临界阻尼无超调，<1有超调但更快
#define YAW_NLTD_MAX_VEL 40000.0f // 最大速度限制 (ECD/s)，0=不限制

// 外环 PID (位置环)
#define YAW_PID_KP 1.2f
#define YAW_PID_KI 0.0f
#define YAW_PID_KD 0.0f
#define YAW_PID_MAX_OUT 350.0f // 最大输出 (RPM)
#define YAW_PID_MAX_I 100.0f   // 积分限幅

// 内环 LADRC (速度环)
#define YAW_LADRC_WO 64.0f     // 观测带宽
#define YAW_LADRC_WC 16.0f     // 控制带宽
#define YAW_LADRC_B0 0.35f     // 系统增益
#define YAW_LADRC_MAX 30000.0f // 最大输出电压

// ----------------Pitch 轴参数----------------
// 机械限位 (度)
#define PITCH_LIMIT_MIN -29.0f
#define PITCH_LIMIT_MAX 16.0f

// 软限位缓冲区 (度)，接近边界时自动减速
#define PITCH_SOFT_LIMIT_ZONE 5.0f

// NLTD 跟踪微分器 (二阶系统参数)
#define PITCH_NLTD_OMEGA 21.0f      // 响应频率
#define PITCH_NLTD_ZETA 1.0f        // 阻尼比
#define PITCH_NLTD_MAX_VEL 40000.0f // 最大速度限制

// 外环 PID (位置环)
#define PITCH_PID_KP 1.5f
#define PITCH_PID_KI 0.0f
#define PITCH_PID_KD 0.0f
#define PITCH_PID_MAX_OUT 350.0f
#define PITCH_PID_MAX_I 100.0f

// 内环 LADRC (速度环)
#define PITCH_LADRC_WO 64.0f
#define PITCH_LADRC_WC 16.0f
#define PITCH_LADRC_B0 9.0f
#define PITCH_LADRC_MAX 30000.0f

/*====================GM6020 云台电机控制 (IMU 反馈模式)====================*/
// 角速度滤波器截止频率 (Hz)
// 建议范围: 30-60Hz, 越低越平滑但延迟越大
#define IMU_GYRO_LPF_CUTOFF_HZ 40.0f

// ----------------Yaw 轴 (IMU 反馈)----------------

// NLTD 目标平滑器参数
// 作用: 平滑目标角度变化，避免阶跃输入导致的抖动
// omega: 响应速度 (rad/s)，越大越快但可能抖动
// zeta: 阻尼比，1.0 为临界阻尼
// max_vel: 最大速度限制 (deg/s)
#define IMU_YAW_NLTD_OMEGA 30.0f    // 响应速度
#define IMU_YAW_NLTD_ZETA 1.56f     // 阻尼比
#define IMU_YAW_NLTD_MAX_VEL 300.0f // 最大速度 (deg/s)

// 外环 PID (角度环)
// 输入: 目标角度(deg), 当前角度(deg)
// 输出: 目标角速度(deg/s)
#define IMU_YAW_ANGLE_KP 37.0f       // 比例系数
#define IMU_YAW_ANGLE_KI 100.0f      // 积分系数
#define IMU_YAW_ANGLE_KD 1.0f        // 微分系数
#define IMU_YAW_ANGLE_MAX_OUT 300.0f // 最大输出角速度 (deg/s)
#define IMU_YAW_ANGLE_MAX_I 100.0f   // 积分限幅

// 内环 LADRC (速度环)
// 输入: 目标角速度(deg/s), 当前角速度(deg/s)
// 输出: 电机电压
#define IMU_YAW_LADRC_WO 48.0
#define IMU_YAW_LADRC_B0 0.25
#define IMU_YAW_LADRC_WC 12.0
#define IMU_YAW_SPEED_MAX_OUT 20000.0f // 最大输出电压

// 内环速度环pid
#define IMU_YAW_OMEGA_KP 500.0
#define IMU_YAW_OMEGA_KI 0.0
#define IMU_YAW_OMEGA_KD 0.0
#define IMU_YAW_OMEGA_MAX_OUT 20000
#define IMU_YAW_OMEGA_MAX_I 200

// 速度前馈增益
// 将 NLTD 输出的速度乘以此系数后加到内环目标上
#define IMU_YAW_FEEDFORWARD_GAIN 1.0f

// Yaw 轴最大手动旋转速度 (deg/s), 用于遥控器速度模式
#define IMU_YAW_MAX_SPEED 450.0f

// ----------------Pitch 轴 (IMU 反馈)----------------

// NLTD 目标平滑器参数
#define IMU_PITCH_NLTD_OMEGA 30.0f    // 响应速度
#define IMU_PITCH_NLTD_ZETA 1.65f     // 阻尼比
#define IMU_PITCH_NLTD_MAX_VEL 200.0f // 最大速度 (deg/s)

// 外环 PID (角度环)
#define IMU_PITCH_ANGLE_KP 40.0f       // 比例系数
#define IMU_PITCH_ANGLE_KI 100.0f      // 积分系数
#define IMU_PITCH_ANGLE_KD 1.0f        // 微分系数
#define IMU_PITCH_ANGLE_MAX_OUT 200.0f // 最大输出角速度 (deg/s)
#define IMU_PITCH_ANGLE_MAX_I 100.0f   // 积分限幅

// 内环 LADRC (速度环)
#define IMU_PITCH_LADRC_WO 48.00
#define IMU_PITCH_LADRC_B0 0.5
#define IMU_PITCH_LADRC_WC 12.00
#define IMU_PITCH_SPEED_MAX_OUT 20000.0f // 最大输出电压

// 内环 PID (速度环)
#define IMU_PITCH_OMEGA_KP 1.0
#define IMU_PITCH_OMEGA_KI 0.0
#define IMU_PITCH_OMEGA_KD 0.0
#define IMU_PITCH_OMEGA_MAX_OUT 20000
#define IMU_PITCH_OMEGA_MAX_I 200

// 速度前馈增益
#define IMU_PITCH_FEEDFORWARD_GAIN 1.0f

// Pitch 轴最大手动旋转速度 (deg/s)
#define IMU_PITCH_MAX_SPEED 180.0f

// Pitch 轴角度限位 (IMU 坐标系下的角度)
// 右手坐标系下低头为正抬头为负
#define IMU_PITCH_MIN_ANGLE -29.0f // 向下最大角度
#define IMU_PITCH_MAX_ANGLE 16.0f  // 向上最大角度

#endif // CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL

#endif // __GIMBAL_CONFIG_H