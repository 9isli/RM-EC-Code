//
// 发射机构控制模块实现
// Created by pomelo on 2025/12/9
// 单发角度环是有问题的，暂时还不清楚在哪！！！
//

#include "shoot_control.h"

#if CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL

#include "bsp_can.h"
#include "bsp_sbus.h"
#include "dji_motor.h"
#include "ladrc.h"
#include "pid.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/*====================内部变量====================*/

static Shoot_State_t shoot_state;
static uint8_t is_initialized = 0;

// 摩擦轮 LADRC 控制器 (M3508)
static LADRC_t friction_ladrc[FRICTION_WHEEL_NUM];

// 拨弹盘 PID 控制器 (角度环)
static PID_t trigger_angle_pid;

// 拨弹盘 LADRC 控制器 (速度环)
static LADRC_t trigger_speed_ladrc;

// 拨弹盘多圈角度维护
typedef struct {
  int32_t round_cnt;
  uint16_t last_raw_ecd;
  float total_angle_deg; // 累计角度 (度)
} TriggerPosition_t;

static TriggerPosition_t trigger_pos = {0};

// 摩擦轮就绪检测
static uint32_t friction_start_tick = 0;

// 单发防抖
static uint32_t last_single_fire_tick = 0;

// 手动反转标志
static uint8_t manual_reverse_active = 0;

/*====================内部函数====================*/

/**
 * @brief 更新拨弹盘多圈角度
 */
static void Update_Trigger_Position(uint16_t raw_ecd) {
  const float ECD_TO_DEG = 360.0f / 8192.0f;

  int32_t diff = (int32_t)raw_ecd - (int32_t)trigger_pos.last_raw_ecd;

  if (diff > 4096) {
    trigger_pos.round_cnt--;
  } else if (diff < -4096) {
    trigger_pos.round_cnt++;
  }

  trigger_pos.total_angle_deg =
      (float)(trigger_pos.round_cnt * 8192 + raw_ecd) * ECD_TO_DEG;
  trigger_pos.last_raw_ecd = raw_ecd;
}

/**
 * @brief 卡弹检测 FSM
 */
static void Jam_Detection_FSM(void) {
  const motor_measure_t *trigger_fb = get_trigger_m2006_measure_point();
  if (!trigger_fb)
    return;

  uint32_t current_tick = HAL_GetTick();
  int16_t current_ma = trigger_fb->given_current;

  switch (shoot_state.jam_state) {
  case JAM_STATE_NORMAL:
    // 监测电流是否超阈值
    if (abs(current_ma) > JAM_CURRENT_THRESHOLD) {
      shoot_state.jam_state = JAM_STATE_SUSPECT;
      shoot_state.jam_detect_start_tick = current_tick;
    }
    break;

  case JAM_STATE_SUSPECT:
    // 检查是否持续大电流
    if (abs(current_ma) > JAM_CURRENT_THRESHOLD) {
      if (current_tick - shoot_state.jam_detect_start_tick >=
          JAM_DETECT_TIME_MS) {
        // 确认卡弹
        shoot_state.jam_state = JAM_STATE_CONFIRM;
        shoot_state.jam_reverse_start_angle = trigger_pos.total_angle_deg;
        shoot_state.jam_process_start_tick = current_tick;

        // 设置反转目标
        shoot_state.trigger_angle =
            trigger_pos.total_angle_deg - JAM_REVERSE_ANGLE;
      }
    } else {
      // 电流恢复正常，回到 NORMAL
      shoot_state.jam_state = JAM_STATE_NORMAL;
    }
    break;

  case JAM_STATE_CONFIRM:
    // 开始反转处理
    shoot_state.jam_state = JAM_STATE_PROCESSING;
    break;

  case JAM_STATE_PROCESSING:
    // 检查处理是否完成
    if (current_tick - shoot_state.jam_process_start_tick >=
        JAM_PROCESS_TIME_MS) {
      // 处理完成，恢复正常
      shoot_state.jam_state = JAM_STATE_NORMAL;

      // 重置拨弹盘目标到当前位置
      shoot_state.trigger_angle = trigger_pos.total_angle_deg;
      PID_Reset(&trigger_angle_pid);
    }
    break;
  }
}

/**
 * @brief 控制摩擦轮
 */
static void Control_Friction_Wheels(void) {
  const motor_measure_t *left_fb = get_chassis_m3508_measure_point(0);
  const motor_measure_t *right_fb = get_chassis_m3508_measure_point(1);

  if (!left_fb || !right_fb) {
    return;
  }

  int16_t left_output = 0;
  int16_t right_output = 0;

  if (shoot_state.mode != SHOOT_MODE_OFF) {
    // 左轮反转（负目标）
    float left_ladrc_out =
        LADRC_Calculate(&friction_ladrc[0], (float)left_fb->speed_rpm,
                        (float)(-shoot_state.friction_speed_left));
    left_output = (int16_t)(left_ladrc_out + 0.5f);

    // 右轮正转
    float right_ladrc_out =
        LADRC_Calculate(&friction_ladrc[1], (float)right_fb->speed_rpm,
                        (float)shoot_state.friction_speed_right);
    right_output = (int16_t)(right_ladrc_out + 0.5f);

    // 检查摩擦轮是否就绪
    if (!shoot_state.friction_ready) {
      uint32_t elapsed = HAL_GetTick() - friction_start_tick;
      int16_t left_err =
          abs(left_fb->speed_rpm + shoot_state.friction_speed_left);
      int16_t right_err =
          abs(right_fb->speed_rpm - shoot_state.friction_speed_right);

      if (elapsed >= FRICTION_READY_DELAY_MS &&
          left_err < FRICTION_READY_THRESHOLD &&
          right_err < FRICTION_READY_THRESHOLD) {
        shoot_state.friction_ready = 1;
      }
    }
  } else {
    shoot_state.friction_ready = 0;
  }

  // 发送摩擦轮控制命令 (CAN1, 0x200, ID1=left, ID2=right)
  motor_control_t motors[2];
  motors[0].msg_id = CAN_MSG_M3508_ID;
  motors[0].slot = MOTOR_SLOT_0;
  motors[0].value = left_output;

  motors[1].msg_id = CAN_MSG_M3508_ID;
  motors[1].slot = MOTOR_SLOT_1;
  motors[1].value = right_output;

  CAN_Manager_SetMotors(motors, 2);
}

/**
 * @brief 控制拨弹盘
 */
static void Control_Trigger(void) {
  const motor_measure_t *trigger_fb = get_trigger_m2006_measure_point();
  if (!trigger_fb) {
    return;
  }

  // 更新多圈角度
  Update_Trigger_Position(trigger_fb->ecd);

  int16_t trigger_output = 0;

  // 手动反转优先级最高
  if (manual_reverse_active) {
    // 手动反转：使用固定反转速度
    float ladrc_out =
        LADRC_Calculate(&trigger_speed_ladrc, (float)trigger_fb->speed_rpm,
                        TRIGGER_MANUAL_REVERSE_SPEED);
    trigger_output = (int16_t)(ladrc_out + 0.5f);
  } else {
    switch (shoot_state.mode) {
    case SHOOT_MODE_OFF:
      trigger_output = 0;
      break;

    case SHOOT_MODE_IDLE:
      if (shoot_state.jam_state == JAM_STATE_NORMAL) {
        shoot_state.trigger_angle = trigger_pos.total_angle_deg;
      }
      {
        float angle_err =
            shoot_state.trigger_angle - trigger_pos.total_angle_deg;
        float speed_target = PID_Calculate(&trigger_angle_pid, 0.0f, angle_err);
        float ladrc_out = LADRC_Calculate(
            &trigger_speed_ladrc, (float)trigger_fb->speed_rpm, speed_target);
        trigger_output = (int16_t)(ladrc_out + 0.5f);
      }
      break;

    case SHOOT_MODE_SINGLE: {
      float angle_err = shoot_state.trigger_angle - trigger_pos.total_angle_deg;
      float speed_target = PID_Calculate(&trigger_angle_pid, 0.0f, angle_err);
      float ladrc_out = LADRC_Calculate(
          &trigger_speed_ladrc, (float)trigger_fb->speed_rpm, speed_target);
      trigger_output = (int16_t)(ladrc_out + 0.5f);
    } break;

    case SHOOT_MODE_BURST:
      if (shoot_state.jam_state == JAM_STATE_NORMAL ||
          shoot_state.jam_state == JAM_STATE_SUSPECT) {
        float ladrc_out =
            LADRC_Calculate(&trigger_speed_ladrc, (float)trigger_fb->speed_rpm,
                            (float)shoot_state.trigger_speed);
        trigger_output = (int16_t)(ladrc_out + 0.5f);
      } else {
        float angle_err =
            shoot_state.trigger_angle - trigger_pos.total_angle_deg;
        float speed_target = PID_Calculate(&trigger_angle_pid, 0.0f, angle_err);
        float ladrc_out = LADRC_Calculate(
            &trigger_speed_ladrc, (float)trigger_fb->speed_rpm, speed_target);
        trigger_output = (int16_t)(ladrc_out + 0.5f);
      }
      break;
    }
  }

  motor_control_t motor;
  motor.msg_id = CAN_MSG_M2006_ID;
  motor.slot = MOTOR_SLOT_2;
  motor.value = trigger_output;

  CAN_Manager_SetMotors(&motor, 1);
}

/*====================外部接口实现====================*/

void Shoot_Init(void) {
  // 初始化状态
  memset(&shoot_state, 0, sizeof(shoot_state));
  shoot_state.mode = SHOOT_MODE_OFF;
  shoot_state.friction_speed_left = FRICTION_DEFAULT_SPEED;
  shoot_state.friction_speed_right = FRICTION_DEFAULT_SPEED;
  shoot_state.trigger_fire_rate = TRIGGER_BURST_FIRE_RATE_HIGH;
  shoot_state.jam_state = JAM_STATE_NORMAL;

  ladrc_config_t friction_cfg;
  memset(&friction_cfg, 0, sizeof(friction_cfg));
  friction_cfg.order = LADRC_FIRST_ORDER;
  friction_cfg.w0 = M3508_LESO_W0;
  friction_cfg.b0 = M3508_LESO_B0;
  friction_cfg.kp = M3508_CONTROL_KP;
  friction_cfg.kd = 0.0f;
  friction_cfg.maxout = M3508_MAX_OUTPUT;
  friction_cfg.improve =
      LADRC_LESO_Trapezoid | LADRC_Feedback_LPF | LADRC_Disturbance_Limit;
  friction_cfg.feedback_LPF_RC = M3508_FEEDBACK_LPF_RC;
  friction_cfg.disturbance_limit = M3508_DISTURBANCE_LIMIT;
  friction_cfg.disturbance_decay = M3508_DISTURBANCE_DECAY;

  for (uint8_t i = 0; i < FRICTION_WHEEL_NUM; i++) {
    LADRC_Init(&friction_ladrc[i], &friction_cfg, M3508_CONTROL_DT);
  }

  pid_config_t trigger_pid_cfg;
  memset(&trigger_pid_cfg, 0, sizeof(trigger_pid_cfg));
  trigger_pid_cfg.kp = TRIGGER_PID_KP;
  trigger_pid_cfg.ki = TRIGGER_PID_KI;
  trigger_pid_cfg.kd = TRIGGER_PID_KD;
  trigger_pid_cfg.maxout = TRIGGER_PID_MAX_OUT;
  trigger_pid_cfg.integralLimit = TRIGGER_PID_MAX_I;
  trigger_pid_cfg.mode = PID_POSITION;
  trigger_pid_cfg.improve = PID_Integral_Limit;
  PID_Init(&trigger_angle_pid, &trigger_pid_cfg, M2006_CONTROL_DT);

  ladrc_config_t trigger_ladrc_cfg;
  memset(&trigger_ladrc_cfg, 0, sizeof(trigger_ladrc_cfg));
  trigger_ladrc_cfg.order = LADRC_FIRST_ORDER;
  trigger_ladrc_cfg.w0 = M2006_LESO_W0;
  trigger_ladrc_cfg.b0 = M2006_LESO_B0;
  trigger_ladrc_cfg.kp = M2006_CONTROL_KP;
  trigger_ladrc_cfg.kd = 0.0f;
  trigger_ladrc_cfg.maxout = M2006_MAX_OUTPUT;
  trigger_ladrc_cfg.improve =
      LADRC_LESO_Trapezoid | LADRC_Feedback_LPF | LADRC_Disturbance_Limit;
  trigger_ladrc_cfg.feedback_LPF_RC = M2006_FEEDBACK_LPF_RC;
  trigger_ladrc_cfg.disturbance_limit = M2006_DISTURBANCE_LIMIT;
  trigger_ladrc_cfg.disturbance_decay = M2006_DISTURBANCE_DECAY;
  LADRC_Init(&trigger_speed_ladrc, &trigger_ladrc_cfg, M2006_CONTROL_DT);

  const motor_measure_t *trigger_fb = get_trigger_m2006_measure_point();
  if (trigger_fb) {
    trigger_pos.last_raw_ecd = trigger_fb->ecd;
    trigger_pos.total_angle_deg = (float)trigger_fb->ecd * (360.0f / 8192.0f);
    trigger_pos.round_cnt = 0;
    shoot_state.trigger_angle = trigger_pos.total_angle_deg;
  }

  is_initialized = 1;
}

void Shoot_Control_Loop(void) {
  if (!is_initialized)
    return;

  Jam_Detection_FSM();

  Control_Friction_Wheels();

  Control_Trigger();
}

void Shoot_Set_Mode(Shoot_Mode_e mode) {
  if (shoot_state.mode == mode)
    return;

  Shoot_Mode_e old_mode = shoot_state.mode;
  shoot_state.mode = mode;

  // 模式切换时的初始化
  switch (mode) {
  case SHOOT_MODE_OFF:
    shoot_state.friction_ready = 0;
    // 重置拨弹盘控制器，防止残留状态
    LADRC_Reset(&trigger_speed_ladrc);
    PID_Reset(&trigger_angle_pid);
    break;

  case SHOOT_MODE_IDLE:
    // 从连发模式切换过来时，重置控制器防止继续转动
    if (old_mode == SHOOT_MODE_BURST) {
      LADRC_Reset(&trigger_speed_ladrc);
      PID_Reset(&trigger_angle_pid);
    }
    // 注意：不重置 friction_ready，保持摩擦轮就绪状态
    shoot_state.trigger_angle = trigger_pos.total_angle_deg;
    break;

  case SHOOT_MODE_SINGLE:
    shoot_state.trigger_single_fired = 0;
    break;

  case SHOOT_MODE_BURST:
    // 计算连发转速
    shoot_state.trigger_speed =
        (int16_t)(shoot_state.trigger_fire_rate * TRIGGER_SINGLE_ANGLE_DEG *
                  TRIGGER_GEAR_RATIO /
                  6.0f); // RPM = (发/秒) * (度/发) * 减速比 / 6
    break;
  }
}

Shoot_Mode_e Shoot_Get_Mode(void) { return shoot_state.mode; }

void Shoot_Set_Friction_Speed(int16_t speed) {
  // 只有速度变化时才重置就绪状态
  if (shoot_state.friction_speed_left != speed ||
      shoot_state.friction_speed_right != speed) {
    shoot_state.friction_speed_left = speed;
    shoot_state.friction_speed_right = speed;
    shoot_state.friction_ready = 0;
    friction_start_tick = HAL_GetTick();
  }
}

void Shoot_Set_Fire_Rate(float rate) {
  shoot_state.trigger_fire_rate = rate;
  // 更新连发转速
  shoot_state.trigger_speed =
      (int16_t)(rate * TRIGGER_SINGLE_ANGLE_DEG * TRIGGER_GEAR_RATIO / 6.0f);
}

void Shoot_Trigger_Single(void) {
  if (shoot_state.mode != SHOOT_MODE_SINGLE)
    return;

  if (!shoot_state.friction_ready)
    return;

  uint32_t current_tick = HAL_GetTick();

  // 防抖处理
  if (current_tick - last_single_fire_tick < TRIGGER_SINGLE_DEADZONE_MS)
    return;

  // 触发单发
  shoot_state.trigger_angle += TRIGGER_SINGLE_ANGLE_DEG;
  last_single_fire_tick = current_tick;
}

const Shoot_State_t *Shoot_Get_State(void) { return &shoot_state; }

void Shoot_Emergency_Stop(void) {
  shoot_state.mode = SHOOT_MODE_OFF;
  shoot_state.friction_ready = 0;

  // 停止所有电机
  motor_control_t motors[3];
  for (int i = 0; i < 3; i++) {
    motors[i].msg_id = (i < 2) ? CAN_MSG_M3508_ID : CAN_MSG_M2006_ID;
    motors[i].slot = i;
    motors[i].value = 0;
  }
  CAN_Manager_SetMotors(motors, 3);
}

uint8_t Shoot_Is_Friction_Ready(void) { return shoot_state.friction_ready; }

void Shoot_Manual_Reverse(uint8_t enable) {
  manual_reverse_active = enable;
  if (enable) {
    // 开始手动反转时，更新当前角度
    shoot_state.trigger_angle = trigger_pos.total_angle_deg;
  }
}

/**
 * @brief 处理发射机构遥控器输入
 * @param rc_connected 遥控器是否连接
 * @param master_enabled 总开关是否启用
 */
void Shoot_Process_RC_Input(uint8_t rc_connected, uint8_t master_enabled) {
  // 总开关关闭或遥控器断连时，关闭发射机构
  if (!rc_connected || !master_enabled) {
    Shoot_Set_Mode(SHOOT_MODE_OFF);
    Shoot_Manual_Reverse(0);
    return;
  }

  // 读取原始通道值
  const SBUS_Data_t *sbus = SBUS_Get_Data();
  uint16_t speed_ch = sbus->channels[RC_CH_FRICTION_SPEED];
  uint16_t trigger_ch = sbus->channels[RC_CH_SHOOT_TRIGGER];
  uint16_t fire_rate_ch = sbus->channels[RC_CH_FIRE_RATE_REVERSE];

  // 根据通道5的值选择摩擦轮速度
  // 321 = 关闭, 992 = 15m/s, 1663 = 18m/s
  uint8_t friction_enabled = 1;
  uint16_t friction_speed;
  if (speed_ch < 650) {
    // 关闭摩擦轮
    friction_enabled = 0;
    friction_speed = 0;
  } else if (speed_ch < 1300) {
    // 15m/s
    friction_speed = FRICTION_SPEED_15MS;
  } else {
    // 18m/s
    friction_speed = FRICTION_SPEED_18MS;
  }

  // 设置摩擦轮速度
  if (friction_enabled) {
    Shoot_Set_Friction_Speed(friction_speed);
    // 摩擦轮开启时切换到 IDLE 模式（如果当前是 OFF）
    if (shoot_state.mode == SHOOT_MODE_OFF) {
      Shoot_Set_Mode(SHOOT_MODE_IDLE);
    }
  } else {
    // 关闭摩擦轮
    Shoot_Set_Mode(SHOOT_MODE_OFF);
    Shoot_Manual_Reverse(0);
    return; // 摩擦轮关闭时，扳机和反转都无效
  }

  // 根据通道7的值判断射频或反转
  // 321 = 持续反转, 992 = 低射频, 1663 = 高射频
  uint8_t manual_reverse = 0;
  float fire_rate = TRIGGER_BURST_FIRE_RATE_HIGH; // 默认高射频

  if (fire_rate_ch < 650) {
    // 持续卡弹反转模式
    manual_reverse = 1;
  } else if (fire_rate_ch < 1300) {
    // 低射频模式
    fire_rate = TRIGGER_BURST_FIRE_RATE_LOW;
  } else {
    // 高射频模式
    fire_rate = TRIGGER_BURST_FIRE_RATE_HIGH;
  }

  // 设置射频
  Shoot_Set_Fire_Rate(fire_rate);

  // 设置手动反转
  Shoot_Manual_Reverse(manual_reverse);

  // 根据通道6的值判断是否发射 (仅在非反转模式且摩擦轮就绪时有效)
  // 321 = 不发射, 1663 = 发射
  if (!manual_reverse) {
    uint8_t trigger_pressed = (trigger_ch > 1000);
    // TODO: 临时绕过friction_ready检查，测试完成后恢复
    if (trigger_pressed /* && shoot_state.friction_ready */) {
      Shoot_Set_Mode(SHOOT_MODE_BURST);
    } else {
      Shoot_Set_Mode(SHOOT_MODE_IDLE);
    }
  } else {
    // 反转模式下不发射
    Shoot_Set_Mode(SHOOT_MODE_IDLE);
  }
}

#endif // CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL
