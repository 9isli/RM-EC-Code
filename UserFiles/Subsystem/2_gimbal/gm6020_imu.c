//
// GM6020 云台电机控制
// Created by pomelo on 2025/12/3
//

#include "gm6020_imu.h"
#include "IMU.h"
#include "bsp_can.h"
#include "dji_motor.h"
#include "filter.h"
#include "ladrc.h"
#include "nltd.h"
#include "pid.h"
#include "robot_config.h"
#include "task_user.h"
#include <math.h>
#include <string.h>

/*这里选择速度环是pid还是ladrc*/
#define PID 0
#define LADRC 1
#define OMEGACONTROLLER LADRC

/*====================控制器实例====================*/

// 外环 PID
static PID_t yaw_pid;
static PID_t pitch_pid;

#if OMEGACONTROLLER == PID
// 内环 PID
static PID_t yaw_omega_pid;
static PID_t pitch_omega_pid;
#else
// 内环 LADRC
static LADRC_t yaw_ladrc;
static LADRC_t pitch_ladrc;
#endif

// NLTD 目标平滑器
static NLTD_t yaw_nltd;
static NLTD_t pitch_nltd;

// 角速度滤波器
static LPF1_t yaw_gyro_filter;
static LPF1_t pitch_gyro_filter;

/*====================状态变量====================*/

// 原始目标角度 (度) - 来自外部设定
static float raw_target_yaw_deg = 0.0f;
static float raw_target_pitch_deg = 0.0f;

// 平滑后的目标角度 (度) - NLTD 输出
static float smooth_target_yaw_deg = 0.0f;
static float smooth_target_pitch_deg = 0.0f;

// 当前 IMU 角度缓存
static float current_yaw_deg = 0.0f;
static float current_pitch_deg = 0.0f;

// 获取task_imu.c中维护的全局变量
extern float g_imu_pitch;
extern float g_totalyaw;
extern float g_omega_roll;
extern float g_omega_pitch;
extern float g_omega_yaw;
extern float g_rawomega_roll;
extern float g_rawomega_pitch;
extern float g_rawomega_yaw;

// yaw和pitch的角速度
static float gyro[3] = {0};

// pitch限位
static float pitch_min_angle = IMU_PITCH_MIN_ANGLE;
static float pitch_max_angle = IMU_PITCH_MAX_ANGLE;

// 滤波后的角速度缓存
static float filtered_gyro_yaw = 0.0f;
static float filtered_gyro_pitch = 0.0f;

// 初始化标志
static uint8_t is_initialized = 0;

// 首次运行标志 (用于对齐目标到当前角度)
static uint8_t first_run = 1;

// IMU 稳定检测相关
static uint8_t imu_stable = 0;      // IMU 是否已稳定
static float last_pitch_deg = 0.0f; // 上一帧 Pitch 角度
static float last_yaw_deg = 0.0f;   // 上一帧 Yaw 角度
static uint16_t stable_counter = 0; // 稳定计数器

// IMU 稳定判定参数
#define IMU_STABLE_THRESHOLD 0.5f     // 角度变化阈值 (度/帧)，小于此值认为稳定
#define IMU_STABLE_COUNT_REQUIRED 100 // 需要连续稳定的帧数 (100帧 = 100ms)

// 紧急停止标志
static uint8_t gimbal_stopped = 0;

/*====================内部函数====================*/

/**
 * @brief 限幅函数
 */
static inline float Clamp(float val, float min_val, float max_val) {
  if (val < min_val)
    return min_val;
  if (val > max_val)
    return max_val;
  return val;
}

/**
 * @brief 就近转位 - 防止目标角度跳变导致电机疯转
 * @param target 目标角度
 * @param current 当前角度
 * @return 调整后的目标角度
 */
static float Nearest_Angle(float target, float current) {
  float delta = target - current;
  delta = fmodf(delta, 360.0f);

  if (delta > 180.0f) {
    delta -= 360.0f;
  } else if (delta < -180.0f) {
    delta += 360.0f;
  }

  return current + delta;
}

/**
 * @brief 读取并更新 IMU 数据
 */
static void Update_IMU_Data(void) {
  // 获取角速度，原数据
  gyro[0] = g_rawomega_roll;
  gyro[1] = g_rawomega_pitch;
  gyro[2] = g_rawomega_yaw;

  // 更新角度缓存
  current_yaw_deg = g_totalyaw;
  current_pitch_deg = g_imu_pitch;

  // 角速度滤波
  filtered_gyro_yaw = LPF1_Calculate(&yaw_gyro_filter, gyro[2]);
  filtered_gyro_pitch = LPF1_Calculate(&pitch_gyro_filter, gyro[1]);
}

/*====================对外接口====================*/

void GM6020_IMU_Init(void) {
  // 角速度滤波器初始化
  LPF1_Init_Cutoff(&yaw_gyro_filter, IMU_GYRO_LPF_CUTOFF_HZ, GM6020_CONTROL_DT);
  LPF1_Init_Cutoff(&pitch_gyro_filter, IMU_GYRO_LPF_CUTOFF_HZ,
                   GM6020_CONTROL_DT);

  NLTD_Init(&yaw_nltd, IMU_YAW_NLTD_OMEGA, IMU_YAW_NLTD_ZETA,
            IMU_YAW_NLTD_MAX_VEL, GM6020_CONTROL_DT);
  NLTD_Init(&pitch_nltd, IMU_PITCH_NLTD_OMEGA, IMU_PITCH_NLTD_ZETA,
            IMU_PITCH_NLTD_MAX_VEL, GM6020_CONTROL_DT);

  pid_config_t pid_cfg;
  memset(&pid_cfg, 0, sizeof(pid_cfg));

  // Yaw PID
  pid_cfg.kp = IMU_YAW_ANGLE_KP;
  pid_cfg.ki = IMU_YAW_ANGLE_KI;
  pid_cfg.kd = IMU_YAW_ANGLE_KD;
  pid_cfg.maxout = IMU_YAW_ANGLE_MAX_OUT;
  pid_cfg.integralLimit = IMU_YAW_ANGLE_MAX_I;
  pid_cfg.mode = PID_POSITION;
  pid_cfg.improve = PID_Integral_Limit | PID_DeadBand;
  pid_cfg.deadBand = 0.5;

  PID_Init(&yaw_pid, &pid_cfg, GM6020_CONTROL_DT);

  // Pitch PID
  pid_cfg.kp = IMU_PITCH_ANGLE_KP;
  pid_cfg.ki = IMU_PITCH_ANGLE_KI;
  pid_cfg.kd = IMU_PITCH_ANGLE_KD;
  pid_cfg.maxout = IMU_PITCH_ANGLE_MAX_OUT;
  pid_cfg.integralLimit = IMU_PITCH_ANGLE_MAX_I;
  pid_cfg.mode = PID_POSITION;
  pid_cfg.improve = PID_Integral_Limit | PID_DeadBand;
  pid_cfg.deadBand = 0.5;

  PID_Init(&pitch_pid, &pid_cfg, GM6020_CONTROL_DT);

#if OMEGACONTROLLER == PID
  // Yaw PID
  pid_cfg.kp = IMU_YAW_OMEGA_KP;
  pid_cfg.ki = IMU_YAW_OMEGA_KI;
  pid_cfg.kd = IMU_YAW_OMEGA_KD;
  pid_cfg.maxout = IMU_YAW_OMEGA_MAX_OUT;
  pid_cfg.integralLimit = IMU_YAW_OMEGA_MAX_I;
  pid_cfg.mode = PID_DELTA;
  pid_cfg.improve = PID_Integral_Limit | PID_DeadBand;
  pid_cfg.deadBand = 0.5;

  PID_Init(&yaw_omega_pid, &pid_cfg, GM6020_CONTROL_DT);

  // Pitch PID
  pid_cfg.kp = IMU_PITCH_OMEGA_KP;
  pid_cfg.ki = IMU_PITCH_OMEGA_KI;
  pid_cfg.kd = IMU_PITCH_OMEGA_KD;
  pid_cfg.maxout = IMU_PITCH_OMEGA_MAX_OUT;
  pid_cfg.integralLimit = IMU_PITCH_OMEGA_MAX_I;
  pid_cfg.mode = PID_DELTA;
  pid_cfg.improve = PID_Integral_Limit | PID_DeadBand;
  pid_cfg.deadBand = 0.5;

  PID_Init(&pitch_omega_pid, &pid_cfg, GM6020_CONTROL_DT);

#else
  // 初始化内环 LADRC
  ladrc_config_t ladrc_cfg;
  memset(&ladrc_cfg, 0, sizeof(ladrc_cfg));

  ladrc_cfg.disturbance_limit = 20000.0f;
  ladrc_cfg.disturbance_decay = 1.0f; // 不衰减
  // 反馈滤波：RC = 1/(2*pi*fc), 设 0.002f 约为 80Hz 截止频率
  ladrc_cfg.feedback_LPF_RC = 0.002f;

  // Yaw LADRC
  ladrc_cfg.order = LADRC_FIRST_ORDER;
  ladrc_cfg.w0 = IMU_YAW_LADRC_WO;
  ladrc_cfg.b0 = IMU_YAW_LADRC_B0;
  ladrc_cfg.kp = IMU_YAW_LADRC_WC;
  ladrc_cfg.kd = 0.0f;
  ladrc_cfg.maxout = YAW_LADRC_MAX;
  // 梯形积分 + 反馈滤波 + 扰动限幅
  ladrc_cfg.improve =
      LADRC_LESO_Trapezoid | LADRC_Feedback_LPF | LADRC_Disturbance_Limit;

  LADRC_Init(&yaw_ladrc, &ladrc_cfg, GM6020_CONTROL_DT);

  // Pitch LADRC
  ladrc_cfg.w0 = IMU_PITCH_LADRC_WO;
  ladrc_cfg.b0 = IMU_PITCH_LADRC_B0;
  ladrc_cfg.kp = IMU_PITCH_LADRC_WC;
  ladrc_cfg.maxout = PITCH_LADRC_MAX;
  ladrc_cfg.improve =
      LADRC_LESO_Trapezoid | LADRC_Feedback_LPF | LADRC_Disturbance_Limit;

  LADRC_Init(&pitch_ladrc, &ladrc_cfg, GM6020_CONTROL_DT);
#endif

  // 标记初始化完成
  is_initialized = 1;
  first_run = 1;
  imu_stable = 0;
  stable_counter = 0;
}

void GM6020_IMU_Control_Loop(void) {
  if (!is_initialized)
    return;

  // 紧急停止状态：只发送零电压，不执行任何控制计算
  if (gimbal_stopped) {
    motor_control_t motors[2];
    motors[0].msg_id = CAN_MSG_GM6020_ID;
    motors[0].slot = MOTOR_SLOT_0;
    motors[0].value = 0;
    motors[1].msg_id = CAN_MSG_GM6020_ID;
    motors[1].slot = MOTOR_SLOT_1;
    motors[1].value = 0;
    CAN_Manager_SetMotors(motors, 2);
    return;
  }

  // 更新 IMU 数据
  Update_IMU_Data();

  // 首次运行时, 将目标对齐到当前实际角度
  if (first_run) {
    raw_target_yaw_deg = current_yaw_deg;
    raw_target_pitch_deg = current_pitch_deg;

    // 初始化 NLTD 状态到当前位置
    NLTD_Reset(&yaw_nltd, current_yaw_deg);
    NLTD_Reset(&pitch_nltd, current_pitch_deg);
    smooth_target_yaw_deg = current_yaw_deg;
    smooth_target_pitch_deg = current_pitch_deg;

    // 重置 PID 控制器
    PID_Reset(&yaw_pid);
    PID_Reset(&pitch_pid);
#if OMEGACONTROLLER == PID
    // 重置 PID 控制器
    PID_Reset(&yaw_omega_pid);
    PID_Reset(&pitch_omega_pid);
#else
    LADRC_Reset(&yaw_ladrc);
    LADRC_Reset(&pitch_ladrc);
#endif

    // 初始化稳定检测
    last_pitch_deg = current_pitch_deg;
    last_yaw_deg = current_yaw_deg;
    imu_stable = 0;
    stable_counter = 0;

    first_run = 0;
    return; // 首帧不输出, 防止跳变
  }

  // IMU 稳定性检测：在 IMU 未稳定前，持续将目标对齐到当前角度
  if (!imu_stable) {
    // 计算角度变化率
    float pitch_delta = fabsf(current_pitch_deg - last_pitch_deg);
    float yaw_delta = fabsf(current_yaw_deg - last_yaw_deg);

    // 更新上一帧角度
    last_pitch_deg = current_pitch_deg;
    last_yaw_deg = current_yaw_deg;

    // 判断是否稳定
    if (pitch_delta < IMU_STABLE_THRESHOLD &&
        yaw_delta < IMU_STABLE_THRESHOLD) {
      stable_counter++;
      if (stable_counter >= IMU_STABLE_COUNT_REQUIRED) {
        // IMU 已稳定，可以开始正常控制
        imu_stable = 1;
      }
    } else {
      // 不稳定，重置计数器
      stable_counter = 0;
    }

    // 未稳定时，持续将目标对齐到当前角度，防止累积误差
    raw_target_yaw_deg = current_yaw_deg;
    raw_target_pitch_deg = current_pitch_deg;
    NLTD_Reset(&yaw_nltd, current_yaw_deg);
    NLTD_Reset(&pitch_nltd, current_pitch_deg);
    smooth_target_yaw_deg = current_yaw_deg;
    smooth_target_pitch_deg = current_pitch_deg;

    // 重置控制器状态，防止积分累积
    PID_Reset(&yaw_pid);
    PID_Reset(&pitch_pid);
#if OMEGACONTROLLER == PID
    PID_Reset(&yaw_omega_pid);
    PID_Reset(&pitch_omega_pid);
#else
    LADRC_Reset(&yaw_ladrc);
    LADRC_Reset(&pitch_ladrc);
#endif

    // 输出零电压，等待 IMU 稳定
    motor_control_t motors[2];
    motors[0].msg_id = CAN_MSG_GM6020_ID;
    motors[0].slot = MOTOR_SLOT_0;
    motors[0].value = 0;
    motors[1].msg_id = CAN_MSG_GM6020_ID;
    motors[1].slot = MOTOR_SLOT_1;
    motors[1].value = 0;
    CAN_Manager_SetMotors(motors, 2);
    return;
  }

  float nearest_yaw_target = Nearest_Angle(raw_target_yaw_deg, current_yaw_deg);
  NLTD_Update(&yaw_nltd, nearest_yaw_target);
  smooth_target_yaw_deg = NLTD_Get_Position(&yaw_nltd);
  float feedforward_yaw_vel = NLTD_Get_Velocity(&yaw_nltd);

  NLTD_Update(&pitch_nltd, raw_target_pitch_deg);
  smooth_target_pitch_deg = NLTD_Get_Position(&pitch_nltd);
  float feedforward_pitch_vel = NLTD_Get_Velocity(&pitch_nltd);

  // yaw
  float target_gyro_yaw =
      PID_Calculate(&yaw_pid, current_yaw_deg, smooth_target_yaw_deg);
  target_gyro_yaw += feedforward_yaw_vel * IMU_YAW_FEEDFORWARD_GAIN;

  // pitch
  float target_gyro_pitch =
      PID_Calculate(&pitch_pid, current_pitch_deg, smooth_target_pitch_deg);
  target_gyro_pitch += feedforward_pitch_vel * IMU_PITCH_FEEDFORWARD_GAIN;

#if OMEGACONTROLLER == PID
  float yaw_volt;
  yaw_volt = PID_Calculate(&yaw_omega_pid, filtered_gyro_yaw, target_gyro_yaw);

  float pitch_volt;
  pitch_volt =
      PID_Calculate(&pitch_omega_pid, filtered_gyro_pitch, target_gyro_pitch);
#else
  float yaw_volt;
  yaw_volt = LADRC_Calculate(&yaw_ladrc, filtered_gyro_yaw, target_gyro_yaw);

  float pitch_volt;
  pitch_volt =
      LADRC_Calculate(&pitch_ladrc, filtered_gyro_pitch, target_gyro_pitch);
#endif
  motor_control_t motors[2];

  motors[0].msg_id = CAN_MSG_GM6020_ID;
  motors[0].slot = MOTOR_SLOT_0;
  motors[0].value = (int16_t)Clamp(yaw_volt, -30000.0f, 30000.0f);

  motors[1].msg_id = CAN_MSG_GM6020_ID;
  motors[1].slot = MOTOR_SLOT_1;
  motors[1].value = (int16_t)Clamp(-pitch_volt, -30000.0f, 30000.0f);

  CAN_Manager_SetMotors(motors, 2);
}

void IMU_Yaw_Target(float target_deg) { raw_target_yaw_deg = target_deg; }

void IMU_Pitch_Target(float target_deg) {
  raw_target_pitch_deg =
      Clamp(target_deg, IMU_PITCH_MIN_ANGLE, IMU_PITCH_MAX_ANGLE);
}

/**
 * @brief 摇杆控制yaw轴的接口
 * @param[in] throttle 油门输入 [-1.0, 1.0]，正值向上，负值向下
 */
void GM6020_IMU_Yaw_Speed_Control_Mode(float throttle) {
  if (throttle > -0.05f && throttle < 0.05f) {
    return;
  }

  // 限制输入范围
  if (throttle > 1.0f)
    throttle = 1.0f;
  if (throttle < -1.0f)
    throttle = -1.0f;

  // 限制目标与当前实际角度的差值不超过 90 度，防止高速时目标跑太远
  float max_lead = 90.0f;
  float diff = raw_target_yaw_deg - g_totalyaw;
  if (diff > max_lead) {
    raw_target_yaw_deg = g_totalyaw + max_lead;
  } else if (diff < -max_lead) {
    raw_target_yaw_deg = g_totalyaw - max_lead;
  }

  // 角度增量模式：摇杆值直接乘以增益系数，累加到目标角度
  // 增益系数决定了摇杆满打时每个控制周期增加多少度
  // 0.3f 在 1kHz 下约等于 300°/s 满打速度
  const float ANGLE_INCREMENT_GAIN = 0.3f;

  raw_target_yaw_deg += ANGLE_INCREMENT_GAIN * throttle;
}

/**
 * @brief 摇杆控制pitch轴的接口
 * @param[in] throttle 油门输入 [-1.0, 1.0]，正值向上，负值向下
 */
void GM6020_IMU_Pitch_Speed_Control_Mode(float throttle) {
  if (throttle > -0.05f && throttle < 0.05f) {
    return;
  }

  // 限制输入范围
  if (throttle > 1.0f)
    throttle = 1.0f;
  if (throttle < -1.0f)
    throttle = -1.0f;

  // 角度增量模式
  const float ANGLE_INCREMENT_GAIN = 0.15f; // Pitch 轴增益可以小一些

  float new_target_angle =
      raw_target_pitch_deg + ANGLE_INCREMENT_GAIN * throttle;

  // 限位检查
  if (new_target_angle < pitch_min_angle) {
    new_target_angle = pitch_min_angle;
  } else if (new_target_angle > pitch_max_angle) {
    new_target_angle = pitch_max_angle;
  }

  // 软限位：接近边界时减速
  const float SOFT_LIMIT_ZONE = PITCH_SOFT_LIMIT_ZONE;

  if (new_target_angle < pitch_min_angle + SOFT_LIMIT_ZONE && throttle < 0) {
    float ratio = (new_target_angle - pitch_min_angle) / SOFT_LIMIT_ZONE;
    if (ratio < 0)
      ratio = 0;
    new_target_angle =
        raw_target_pitch_deg + ANGLE_INCREMENT_GAIN * throttle * ratio;
  } else if (new_target_angle > pitch_max_angle - SOFT_LIMIT_ZONE &&
             throttle > 0) {
    float ratio = (pitch_max_angle - new_target_angle) / SOFT_LIMIT_ZONE;
    if (ratio < 0)
      ratio = 0;
    new_target_angle =
        raw_target_pitch_deg + ANGLE_INCREMENT_GAIN * throttle * ratio;
  }

  raw_target_pitch_deg = new_target_angle;
}

/*====================紧急停止接口====================*/

/**
 * @brief 云台紧急停止
 * @note  设置停止标志，控制循环将只输出零电压
 */
void GM6020_IMU_Emergency_Stop(void) {
  gimbal_stopped = 1;

  // 立即发送零电压，不等待下一个控制周期
  motor_control_t motors[2];
  motors[0].msg_id = CAN_MSG_GM6020_ID;
  motors[0].slot = MOTOR_SLOT_0;
  motors[0].value = 0;
  motors[1].msg_id = CAN_MSG_GM6020_ID;
  motors[1].slot = MOTOR_SLOT_1;
  motors[1].value = 0;
  CAN_Manager_SetMotors(motors, 2);
}

/**
 * @brief 云台恢复控制
 * @note  清除停止标志，并触发 first_run 重新对齐目标到当前位置
 *        这样恢复后不会因为目标与实际位置偏差而"发疯"
 */
void GM6020_IMU_Resume_Control(void) {
  if (!gimbal_stopped)
    return;

  // 关键：设置 first_run = 1
  // 下次控制循环会自动将目标对齐到当前 IMU 角度，并重置所有控制器状态
  first_run = 1;
  imu_stable = 1; // 恢复时 IMU 已经稳定，不需要再等待
  gimbal_stopped = 0;
}

/**
 * @brief 检查云台是否处于停止状态
 * @return 1: 已停止, 0: 正常运行
 */
uint8_t GM6020_IMU_Is_Stopped(void) { return gimbal_stopped; }
