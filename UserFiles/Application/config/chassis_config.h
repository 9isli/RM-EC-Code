#ifndef __CHASSIS_CONFIG_H
#define __CHASSIS_CONFIG_H

#if CURRENT_BOARD_ROLE == BOARD_ROLE_CHASSIS

/*====================底盘机械参数====================*/

// 轮子到底盘中心的距离 (m) - X型全向轮布局
#define CHASSIS_RADIUS 0.230f
// 轮子半径 (m)
#define CHASSIS_WHEEL_RADIUS 0.076f
// M3508 减速比 (电机转速 / 输出轴转速)
#define M3508_GEAR_RATIO 19.203208f

/*====================底盘运动限制====================*/

#define CHASSIS_MAX_VX 1.0f        // 最大前后速度 (m/s)
#define CHASSIS_MAX_VY 1.0f        // 最大左右速度 (m/s)
#define CHASSIS_MAX_WZ 6.28f       // 最大自转角速度 (rad/s)
#define CHASSIS_MAX_WHEEL_RPM 8000 // 单轮最大转速 (RPM)
#define CHASSIS_SPIN_SPEED 3.14f   // 小陀螺默认角速度 (rad/s)

// 底盘加速度限制 (用于 NLTD 平滑)
#define CHASSIS_ACCEL_LIMIT 3.0f // 最大线加速度 (m/s^2)，调小会让加减速更柔和
#define CHASSIS_WZ_ACCEL_LIMIT 12.0f // 最大角加速度 (rad/s^2)
#define CHASSIS_NLTD_OMEGA 15.0f     // 跟踪响应速度 (越大约快，推荐 10-20)

/*====================M3508 底盘电机控制====================*/

#define M3508_MOTOR_NUM 4       // M3508电机数量
#define M3508_CONTROL_DT 0.001f // 控制周期: 1ms = 1kHz

// 一阶 LADRC 参数 (速度环)
#define M3508_LESO_W0 15.0f       // LESO 带宽 (rad/s)
#define M3508_LESO_B0 2.238910f   // 控制增益
#define M3508_CONTROL_KP 9.0f     // 比例系数
#define M3508_MAX_OUTPUT 10000.0f // 最大输出

// 优化参数
#define M3508_FEEDBACK_LPF_RC 0.01f     // 反馈滤波时间常数 (RC = 10ms)
#define M3508_DISTURBANCE_LIMIT 5000.0f // 扰动估计限幅
#define M3508_DISTURBANCE_DECAY 0.95f   // 扰动衰减系数

#endif // CURRENT_BOARD_ROLE == BOARD_ROLE_CHASSIS

#endif // __CHASSIS_CONFIG_H
