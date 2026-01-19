#ifndef __SHOOT_CONFIG_H
#define __SHOOT_CONFIG_H

#if CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL

/*====================M2006 拨弹电机控制====================*/

#define M2006_MOTOR_NUM 1       // M2006电机数量
#define M2006_CONTROL_DT 0.001f // 控制周期: 1ms = 1kHz

// 一阶 LADRC 参数 (速度环)
#define M2006_LESO_W0 15.5f       // LESO 带宽 (rad/s)
#define M2006_LESO_B0 11.0f       // 控制增益
#define M2006_CONTROL_KP 8.0f     // 比例系数
#define M2006_MAX_OUTPUT 10000.0f // 最大输出

// 优化参数
#define M2006_FEEDBACK_LPF_RC 0.01f     // 反馈滤波时间常数 (RC = 10ms)
#define M2006_DISTURBANCE_LIMIT 5000.0f // 扰动估计限幅
#define M2006_DISTURBANCE_DECAY 0.95f   // 扰动衰减系数

/*====================发射机构控制====================*/

// 摩擦轮参数 (M3508)
#define FRICTION_WHEEL_NUM 2     // 摩擦轮数量
#define FRICTION_SPEED_15MS 1000 // 15m/s 弹速对应转速 (RPM)（4000）
#define FRICTION_SPEED_18MS 4500 // 18m/s 弹速对应转速 (RPM)
#define FRICTION_DEFAULT_SPEED FRICTION_SPEED_15MS // 默认转速
#define FRICTION_READY_THRESHOLD 200 // 转速误差小于此值视为就绪 (RPM)
#define FRICTION_READY_DELAY_MS 500  // 摩擦轮启动延迟 (ms)

// 拨弹盘参数 (M2006)
#define TRIGGER_GEAR_RATIO 36.0f // M2006 减速比
#define TRIGGER_AMMO_CAPACITY 8  // 弹仓容量 (发/圈)
#define TRIGGER_SINGLE_ANGLE                                                   \
  (2.0f * 3.14159265f / TRIGGER_AMMO_CAPACITY) // 单发角度 (rad)
#define TRIGGER_SINGLE_ANGLE_DEG                                               \
  (360.0f / TRIGGER_AMMO_CAPACITY)         // 单发角度 (度)
#define TRIGGER_SINGLE_DEADZONE_MS 150     // 单发死区时间 (ms)
#define TRIGGER_BURST_FIRE_RATE_LOW 10.0f  // 低射频 (发/秒)
#define TRIGGER_BURST_FIRE_RATE_HIGH 35.0f // 高射频 (发/秒)

// 拨弹盘 PID 参数 (角度环)
#define TRIGGER_PID_KP 4.0f
#define TRIGGER_PID_KI 0.5f // 较大积分项保证低速力矩
#define TRIGGER_PID_KD 0.0f
#define TRIGGER_PID_MAX_OUT 800.0f
#define TRIGGER_PID_MAX_I 2000.0f

// 卡弹检测参数
#define JAM_CURRENT_THRESHOLD 9500            // 卡弹电流阈值 (mA)
#define JAM_DETECT_TIME_MS 500                // 卡弹判定时间 (ms)
#define JAM_REVERSE_ANGLE 15.0f               // 卡弹回退角度 (度)
#define JAM_PROCESS_TIME_MS 300               // 卡弹处理时间 (ms)
#define TRIGGER_MANUAL_REVERSE_SPEED -1000.0f // 手动反转速度 (RPM)

#endif // CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL

#endif // __SHOOT_CONFIG_H