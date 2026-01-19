//
// 整车协调控制模块实现
// Created by pomelo on 2025/12/4
//

#include "robot_control.h"
#include "IMU.h"
#include "bsp_dwt.h"
#include "bsp_sbus.h"
#include "chassis_control.h"
#include "chassis_imu.h"
#include "dji_motor.h"
#include "filter.h"
#include "gm6020_imu.h"
#include "nltd.h"
#include "pid.h"
#include "robot_config.h"
#include "slope.h"
#include "task_user.h"
#include <math.h>
#include <string.h>

#if CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL
#include "shoot_control.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 角度转换宏
#define DEG2RAD (M_PI / 180.0f)
#define RAD2DEG (180.0f / M_PI)
#define ECD_ANGLE_COEF (360.0f / 8192.0f) // 编码器值转角度系数

/*====================内部变量====================*/
static PID_t wz_pid;

static Robot_t robot;
static uint8_t is_initialized = 0;

// 云台电机单圈角度（用于坐标变换）
static float yaw_motor_single_round_angle = 0.0f; // 0~360°

// 底盘与云台的偏差角度
static float chassis_gimbal_offset_angle = 0.0f;

static float chassis_gimbal_offset_ecd = 0.0f;

// DWT 计时器
static uint32_t dwt_cnt_gimbal = 0;

// 小陀螺模式陀螺仪角速度滤波器
static LPF1_t spin_omega_filter;
static uint8_t spin_omega_filter_initialized = 0;

// Follow 模式陀螺仪角速度滤波器
static LPF1_t follow_omega_filter;
static uint8_t follow_omega_filter_initialized = 0;

// 底盘跟随角速度斜坡函数
static Slope_t chassis_follow_slope;
static uint8_t chassis_follow_slope_initialized = 0;

static uint8_t chassis_nofollow_initialized = 0;

static NLTD_t nltd_vx;
static NLTD_t nltd_vy;
static NLTD_t nltd_wz;

// NLTD 状态管理（防止模式切换跳变）
static uint8_t nltd_initialized = 0;     // NLTD 是否已初始化（上电后首次）
static uint8_t chassis_was_stopped = 1;  // 底盘是否处于停止状态（上电默认停止）

// 软停止/软启动相关
typedef struct {
  uint8_t is_active;
  float initial_vx;
  float initial_vy;
  float initial_wz;
  uint32_t start_tick;
} SoftStop_t;

typedef struct {
  uint8_t is_active;
  Robot_Mode_e target_mode;
  uint32_t start_tick;
} SoftStart_t;

static SoftStop_t soft_stop = {0};
static SoftStart_t soft_start = {0};

// 遥控器断连检测
static uint32_t last_rc_connected_tick = 0;
static uint8_t was_disconnected = 0;

// 总开关状态
static uint8_t master_enabled = 0;

static void Read_RC_Input(void);
static void Update_Gimbal_Data(void);
static void Calc_Offset_Angle(void);
static float Apply_Deadzone(float value, float deadzone);

static void Mode_Stop(void);
static void Mode_No_Follow(void);
static void Mode_Gimbal_Only(void);

static void Start_Soft_Stop(void);
static uint8_t Process_Soft_Stop(void);
static void Reset_Soft_Stop(void);
static void Start_Soft_Start(Robot_Mode_e target_mode);
static uint8_t Process_Soft_Start(void);
static void Reset_Soft_Start(void);
static void Ensure_NLTD_Ready(void);

/*====================核心函数实现====================*/

/**
 * @brief 计算云台与底盘的偏差角度
 *       使用云台电机单圈角度与对齐位置的偏差
 *       结果范围 -180° ~ 180°
 */
static void Calc_Offset_Angle(void) {
  // 获取云台电机单圈角度 (0~360°)
  float angle = yaw_motor_single_round_angle;

  chassis_gimbal_offset_ecd = angle;

  // 对齐位置角度 (编码器值转角度)
  float align_angle = (float)GIMBAL_YAW_RESET_ECD * ECD_ANGLE_COEF;

  // 直接用 angle - align_angle，然后归一化到 -180~180
  float diff = angle - align_angle;

  // 归一化到 -180 ~ 180
  while (diff > 180.0f)
    diff -= 360.0f;
  while (diff < -180.0f)
    diff += 360.0f;

  chassis_gimbal_offset_angle = diff;
}

/**
 * @brief 更新云台数据
 */
static void Update_Gimbal_Data(void) {
  // 获取 IMU 姿态角度（用于坐标变换参考）
  float angles[3];
  angles[0] = g_imu_yaw;
  angles[1] = g_imu_pitch;
  robot.gimbal_yaw_rad = angles[0] * DEG2RAD;
  robot.gimbal_pitch_rad = angles[1] * DEG2RAD;

  // 更新云台电机单圈角度（用于底盘跟随）
  const motor_measure_t *yaw_fb = get_yaw_gimbal_gm6020_measure_point();
  float single_ecd = yaw_fb->ecd;
  yaw_motor_single_round_angle = single_ecd * ECD_ANGLE_COEF;

  Chassis_IMU_Update();
}

/**
 * @brief 应用死区
 */
static float Apply_Deadzone(float value, float deadzone) {
  if (value > -deadzone && value < deadzone) {
    return 0.0f;
  }
  if (value > 0) {
    return (value - deadzone) / (1.0f - deadzone);
  } else {
    return (value + deadzone) / (1.0f - deadzone);
  }
}

/**
 * @brief 读取遥控器输入
 */
static void Read_RC_Input(void) {
  const SBUS_Data_t *sbus = SBUS_Get_Data();
  robot.rc_connected = sbus->connect_status;

  // 遥控器断连时清零所有输入
  if (!robot.rc_connected) {
    robot.rc_vx = 0.0f;
    robot.rc_vy = 0.0f;
    robot.rc_yaw = 0.0f;
    robot.rc_pitch = 0.0f;
    robot.spin_speed = 0.0f;
    master_enabled = 0;
    GM6020_IMU_Emergency_Stop(); // 云台紧急停止
    return;
  }

  // 总开关判定 (最高优先级)
  // channels[4]: 321=关闭锁定, 其他=启用
  uint16_t master_sw_ch = sbus->channels[RC_CH_MASTER_SW];
  if (master_sw_ch < 650) {
    // 总开关关闭，全车锁定，清零所有输入
    master_enabled = 0;
    robot.rc_vx = 0.0f;
    robot.rc_vy = 0.0f;
    robot.rc_yaw = 0.0f;
    robot.rc_pitch = 0.0f;
    robot.spin_speed = 0.0f;
    robot.mode = ROBOT_MODE_STOP;
    GM6020_IMU_Emergency_Stop(); // 云台紧急停止
    return;
  }

  // 总开关开启
  master_enabled = 1;

  // 从停止状态恢复时，重新对齐云台目标到当前位置
  if (GM6020_IMU_Is_Stopped()) {
    GM6020_IMU_Resume_Control();
  }

  // 读取各通道归一化值并应用死区
  robot.rc_vx =
      Apply_Deadzone(SBUS_Get_Normalized_Channel(RC_CH_VX), RC_DEADZONE);
  robot.rc_vy =
      Apply_Deadzone(SBUS_Get_Normalized_Channel(RC_CH_VY), RC_DEADZONE);
  robot.rc_yaw =
      -Apply_Deadzone(SBUS_Get_Normalized_Channel(RC_CH_YAW), RC_DEADZONE);
  robot.rc_pitch =
      Apply_Deadzone(SBUS_Get_Normalized_Channel(RC_CH_PITCH), RC_DEADZONE);

  // 模式判定
  // channels[8]: 321=逆时针小陀螺, 992=跟随模式, 1663=顺时针小陀螺
  uint16_t chassis_mode_ch = sbus->channels[RC_CH_CHASSIS_MODE];

  if (chassis_mode_ch < 650) {
    // 逆时针小陀螺
    robot.mode = ROBOT_MODE_SPIN;
    robot.spin_speed = 4.0f;
  } else if (chassis_mode_ch < 1300) {
    // 跟随模式
    robot.mode = ROBOT_MODE_NORMAL;
    robot.spin_speed = 0.0f;
  } else {
    // 顺时针小陀螺
    robot.mode = ROBOT_MODE_SPIN;
    robot.spin_speed = -4.0f;
  }
}

/*====================软停止/软启动====================*/

static void Start_Soft_Stop(void) {
  if (soft_stop.is_active)
    return;

  const Chassis_t *chassis = Chassis_Get_State();
  soft_stop.is_active = 1;
  soft_stop.initial_vx = chassis->vx_set;
  soft_stop.initial_vy = chassis->vy_set;
  soft_stop.initial_wz = chassis->wz_set;
  soft_stop.start_tick = HAL_GetTick();
}

static uint8_t Process_Soft_Stop(void) {
  if (!soft_stop.is_active)
    return 1;

  uint32_t elapsed = HAL_GetTick() - soft_stop.start_tick;
  if (elapsed >= SOFT_STOP_DURATION_MS) {
    soft_stop.is_active = 0;
    Chassis_Set_Mode(CHASSIS_MODE_STOP);
    Chassis_Stop();
    return 1;
  }

  float smooth_ratio =
      (1.0f + cosf(M_PI * (float)elapsed / (float)SOFT_STOP_DURATION_MS)) /
      2.0f;
  Chassis_Set_Mode(CHASSIS_MODE_SOFT_STOP);
  Chassis_Set_Velocity(soft_stop.initial_vx * smooth_ratio,
                       soft_stop.initial_vy * smooth_ratio,
                       soft_stop.initial_wz * smooth_ratio);
  return 0;
}

static void Reset_Soft_Stop(void) { soft_stop.is_active = 0; }

static void Start_Soft_Start(Robot_Mode_e target_mode) {
  soft_start.is_active = 1;
  soft_start.target_mode = target_mode;
  soft_start.start_tick = HAL_GetTick();
}

static uint8_t Process_Soft_Start(void) {
  if (!soft_start.is_active)
    return 1;

  uint32_t elapsed = HAL_GetTick() - soft_start.start_tick;
  if (elapsed >= SOFT_STOP_DURATION_MS) {
    soft_start.is_active = 0;
    return 1;
  }

  float speed_limit_ratio =
      (1.0f - cosf(M_PI * (float)elapsed / (float)SOFT_STOP_DURATION_MS)) /
      2.0f;
  robot.rc_vx *= speed_limit_ratio;
  robot.rc_vy *= speed_limit_ratio;
  return 0;
}

static void Reset_Soft_Start(void) { soft_start.is_active = 0; }

/**
 * @brief 确保 NLTD 状态正确（防止模式切换跳变）
 *        - 刚上电：重置为 0
 *        - 从停止状态恢复：重置为 0（底盘物理上已静止）
 *        - 运动模式之间切换：不重置，自然过渡
 */
static void Ensure_NLTD_Ready(void) {
  if (!nltd_initialized) {
    // 首次上电，重置为 0
    NLTD_Reset(&nltd_vx, 0.0f);
    NLTD_Reset(&nltd_vy, 0.0f);
    nltd_initialized = 1;
    chassis_was_stopped = 0;
    return;
  }

  if (chassis_was_stopped) {
    // 从停止状态恢复，重置为 0（底盘物理上是静止的）
    NLTD_Reset(&nltd_vx, 0.0f);
    NLTD_Reset(&nltd_vy, 0.0f);
    chassis_was_stopped = 0;
    return;
  }

  // 正常模式切换，不重置，NLTD 自然过渡到新目标
}

/*====================模式处理函数====================*/

/**
 * @brief 停止模式
 */
static void Mode_Stop(void) {
  Chassis_Set_Mode(CHASSIS_MODE_STOP);
  Chassis_Stop();
  Reset_Soft_Stop();

  // 标记底盘已停止，下次进入运动模式时 NLTD 会重置为 0
  chassis_was_stopped = 1;

  // 立即重置 NLTD 为 0（底盘物理上已停止，保持状态一致）
  NLTD_Reset(&nltd_vx, 0.0f);
  NLTD_Reset(&nltd_vy, 0.0f);

  // 重置底盘跟随斜坡函数，下次进入跟随模式时重新初始化
  chassis_follow_slope_initialized = 0;
  chassis_nofollow_initialized = 0;

  // 重置滤波器
  spin_omega_filter_initialized = 0;
  follow_omega_filter_initialized = 0;
}

/**
 * @brief 解耦模式（底盘不跟随云台）
 */
static void Mode_No_Follow(void) {
  Chassis_Set_Mode(CHASSIS_MODE_NO_FOLLOW);

  GM6020_IMU_Yaw_Speed_Control_Mode(robot.rc_yaw);
  GM6020_IMU_Pitch_Speed_Control_Mode(robot.rc_pitch);

  // 统一的 NLTD 状态管理
  Ensure_NLTD_Ready();

  // 底盘不跟随，wz = 0
  float wz = 0.0f;

  float vx_gimbal = robot.rc_vx * RC_VX_SENSITIVITY;
  float vy_gimbal = robot.rc_vy * RC_VY_SENSITIVITY;

  float offset_rad = chassis_gimbal_offset_angle * DEG2RAD;

  // NLTD 平滑（在云台坐标系下进行）
  NLTD_Update(&nltd_vx, vx_gimbal);
  NLTD_Update(&nltd_vy, vy_gimbal);
  float smooth_vx = NLTD_Get_Position(&nltd_vx);
  float smooth_vy = NLTD_Get_Position(&nltd_vy);

  float cos_theta = cosf(offset_rad);
  float sin_theta = sinf(offset_rad);
  float vx_chassis = smooth_vx * cos_theta - smooth_vy * sin_theta;
  float vy_chassis = smooth_vx * sin_theta + smooth_vy * cos_theta;

  // 传入的是底盘系
  Chassis_Set_Velocity(vx_chassis, vy_chassis, wz);
}

void Motor_Follow(void) {
  Chassis_Set_Mode(CHASSIS_MODE_NORMAL);

  GM6020_IMU_Yaw_Speed_Control_Mode(robot.rc_yaw);
  GM6020_IMU_Pitch_Speed_Control_Mode(robot.rc_pitch);

  // 统一的 NLTD 状态管理
  Ensure_NLTD_Ready();

  // 斜坡函数初始化（仅首次进入 Follow 模式）
  if (!chassis_follow_slope_initialized) {
    Slope_Init(&chassis_follow_slope, CHASSIS_FOLLOW_SLOPE_INCREASE,
               CHASSIS_FOLLOW_SLOPE_DECREASE, SLOPE_FIRST_REAL);
    Slope_Reset(&chassis_follow_slope, chassis_omega_z * DEG2RAD);
    chassis_follow_slope_initialized = 1;
  }

  if (!follow_omega_filter_initialized) {
    LPF1_Init_Cutoff(&follow_omega_filter, 40.0f, 0.001f);
    follow_omega_filter_initialized = 1;
  }

  float filtered_omega_z_degs =
      LPF1_Calculate(&follow_omega_filter, chassis_omega_z);
  // 转换为 rad/s 供后续使用
  float filtered_omega_z_rads = filtered_omega_z_degs * DEG2RAD;

  float offset_deg = chassis_gimbal_offset_angle;

  float gimbal_target_speed_rads =
      robot.rc_yaw * YAW_MAX_MANUAL_SPEED * DEG2RAD;
  float wz_ff = CHASSIS_FOLLOW_KF * gimbal_target_speed_rads;

  float target_wz = 0.0f;
  if (fabsf(offset_deg) > CHASSIS_FOLLOW_DEADZONE) {
    target_wz = CHASSIS_FOLLOW_KP * offset_deg * DEG2RAD;
  }

  target_wz += wz_ff;

  if (target_wz > CHASSIS_FOLLOW_MAX_WZ)
    target_wz = CHASSIS_FOLLOW_MAX_WZ;
  if (target_wz < -CHASSIS_FOLLOW_MAX_WZ)
    target_wz = -CHASSIS_FOLLOW_MAX_WZ;

  float corrected_wz = PID_Calculate(&wz_pid, filtered_omega_z_rads, target_wz);

  float output_wz = target_wz + corrected_wz;

  float vx_gimbal = robot.rc_vx * RC_VX_SENSITIVITY;
  float vy_gimbal = robot.rc_vy * RC_VY_SENSITIVITY;

  // NLTD 平滑（在云台坐标系下进行）
  NLTD_Update(&nltd_vx, vx_gimbal);
  NLTD_Update(&nltd_vy, vy_gimbal);
  float smooth_vx = NLTD_Get_Position(&nltd_vx);
  float smooth_vy = NLTD_Get_Position(&nltd_vy);

  float offset_rad = offset_deg * DEG2RAD;

  float cos_theta = cosf(offset_rad);
  float sin_theta = sinf(offset_rad);
  float vx_chassis = smooth_vx * cos_theta - smooth_vy * sin_theta;
  float vy_chassis = smooth_vx * sin_theta + smooth_vy * cos_theta;

  Chassis_Set_Velocity(vx_chassis, vy_chassis, output_wz);
}

void Mode_Spin(void) {
  Chassis_Set_Mode(CHASSIS_MODE_NORMAL);

  GM6020_IMU_Yaw_Speed_Control_Mode(robot.rc_yaw);
  GM6020_IMU_Pitch_Speed_Control_Mode(robot.rc_pitch);

  // 统一的 NLTD 状态管理
  Ensure_NLTD_Ready();

  // 滤波器初始化（仅首次进入 Spin 模式）
  if (!spin_omega_filter_initialized) {
    LPF1_Init_Cutoff(&spin_omega_filter, 40.0f, 0.001f);
    spin_omega_filter_initialized = 1;
  }

  float filtered_omega_z_degs =
      LPF1_Calculate(&spin_omega_filter, chassis_omega_z);
  float filtered_omega_z_rads = filtered_omega_z_degs * DEG2RAD;

  float target_wz = robot.spin_speed; // 正值逆时针，负值顺时针

  float corrected_wz = PID_Calculate(&wz_pid, filtered_omega_z_rads, target_wz);
  float output_wz = target_wz + corrected_wz;

  float vx_gimbal = robot.rc_vx * RC_VX_SENSITIVITY;
  float vy_gimbal = robot.rc_vy * RC_VY_SENSITIVITY;

  NLTD_Update(&nltd_vx, vx_gimbal);
  NLTD_Update(&nltd_vy, vy_gimbal);
  float smooth_vx = NLTD_Get_Position(&nltd_vx);
  float smooth_vy = NLTD_Get_Position(&nltd_vy);

  float offset_rad = -chassis_gimbal_offset_angle * DEG2RAD;

  float cos_theta = cosf(offset_rad);
  float sin_theta = sinf(offset_rad);
  float vx_chassis = smooth_vx * cos_theta - smooth_vy * sin_theta;
  float vy_chassis = smooth_vx * sin_theta + smooth_vy * cos_theta;

  Chassis_Set_Velocity(vx_chassis, vy_chassis, output_wz);
}

/**
 * @brief 仅云台模式
 */
static void Mode_Gimbal_Only(void) {
  Chassis_Set_Mode(CHASSIS_MODE_STOP);
  Chassis_Stop();

  float dt = DWT_Get_Delta_T(&dwt_cnt_gimbal);
  if (dt > 0.1f || dt < 0.0001f) {
    dt = ROBOT_CONTROL_DT;
  }

  GM6020_IMU_Yaw_Speed_Control_Mode(robot.rc_yaw);
  GM6020_IMU_Pitch_Speed_Control_Mode(robot.rc_pitch);
}

/*====================外部接口实现====================*/

void Robot_Init(void) {
  pid_config_t pid_cfg;
  memset(&pid_cfg, 0, sizeof(pid_cfg));

  // WZ PID
  pid_cfg.kp = WZ_KP;
  pid_cfg.ki = WZ_KI;
  pid_cfg.kd = WZ_KD;
  pid_cfg.maxout = WZ_MAX_OUT;
  pid_cfg.integralLimit = WZ_MAX_I;
  pid_cfg.mode = PID_POSITION;
  pid_cfg.improve = PID_Integral_Limit | PID_DeadBand;
  pid_cfg.deadBand = 0.5;

  PID_Init(&wz_pid, &pid_cfg, 0.001);

  robot.mode = ROBOT_MODE_STOP;
  robot.rc_vx = 0.0f;
  robot.rc_vy = 0.0f;
  robot.rc_yaw = 0.0f;
  robot.rc_pitch = 0.0f;
  robot.gimbal_yaw_rad = 0.0f;
  robot.gimbal_pitch_rad = 0.0f;
  robot.spin_speed = 0.0f;
  robot.rc_connected = 0;

  // 初始化 NLTD 平滑滤波器
  NLTD_Init(&nltd_vx, CHASSIS_NLTD_OMEGA, 1.0f, CHASSIS_ACCEL_LIMIT, 0.001f);
  NLTD_Init(&nltd_vy, CHASSIS_NLTD_OMEGA, 1.0f, CHASSIS_ACCEL_LIMIT, 0.001f);
  NLTD_Init(&nltd_wz, CHASSIS_NLTD_OMEGA, 1.0f, CHASSIS_WZ_ACCEL_LIMIT, 0.001f);

  Chassis_IMU_Init();

  // 初始化软停止/软启动状态
  soft_stop.is_active = 0;
  soft_start.is_active = 0;
  last_rc_connected_tick = HAL_GetTick();
  was_disconnected = 0;

  is_initialized = 1;
}

void Robot_Set_Mode(Robot_Mode_e mode) { robot.mode = mode; }

Robot_Mode_e Robot_Get_Mode(void) { return robot.mode; }

void Robot_Set_Spin_Speed(float speed) { robot.spin_speed = speed; }

void Robot_Control_Loop(void) {
  if (!is_initialized)
    return;

  Read_RC_Input();

  Update_Gimbal_Data();

  Calc_Offset_Angle();

#if CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL
  // 发射机构 RC 输入处理（传递总开关状态）
  Shoot_Process_RC_Input(robot.rc_connected, master_enabled);
#endif

  // 遥控器连接状态处理
  if (robot.rc_connected) {
    last_rc_connected_tick = HAL_GetTick();

    if (soft_stop.is_active) {
      Reset_Soft_Stop();
      Start_Soft_Start(robot.mode);
      was_disconnected = 0;
    }

    if (was_disconnected) {
      was_disconnected = 0;
      Start_Soft_Start(robot.mode);
    }

    Process_Soft_Start();
  } else {
    uint32_t disconnect_time = HAL_GetTick() - last_rc_connected_tick;
    if (disconnect_time > RC_DISCONNECT_TIMEOUT_MS) {
      was_disconnected = 1;
      Reset_Soft_Start();

      if (!soft_stop.is_active) {
        Start_Soft_Stop();
      }

      if (Process_Soft_Stop()) {
        Mode_Stop();
      }
      return;
    }
  }

  // 根据模式执行相应控制
  switch (robot.mode) {
  case ROBOT_MODE_STOP:
    Mode_Stop();
    break;

  case ROBOT_MODE_NO_FOLLOW:
    Mode_No_Follow();
    break;

  case ROBOT_MODE_GIMBAL_ONLY:
    Mode_Gimbal_Only();
    break;

  case ROBOT_MODE_NORMAL:
    Motor_Follow();
    break;
  case ROBOT_MODE_SPIN:
    Mode_Spin();
    break;
  case ROBOT_MODE_SPIN_MOVE:
    Mode_No_Follow();
    break;

  default:
    Mode_Stop();
    break;
  }
}

const Robot_t *Robot_Get_State(void) { return &robot; }

void Robot_Emergency_Stop(void) {
  robot.mode = ROBOT_MODE_STOP;
  Chassis_Stop();

  // 紧急停止：立即重置 NLTD 状态
  chassis_was_stopped = 1;
  NLTD_Reset(&nltd_vx, 0.0f);
  NLTD_Reset(&nltd_vy, 0.0f);
}

/**
 * @brief 获取云台与底盘的偏差角度
 * @return 偏差角度 (度)，-180 ~ 180
 */
float Robot_Get_Gimbal_Offset_Angle(void) {
  return chassis_gimbal_offset_angle;
}
