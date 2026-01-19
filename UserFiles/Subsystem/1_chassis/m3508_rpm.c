//
// M3508 电机 LADRC 速度控制子系统实现
// M3508 就不用写角度环了
// Created by pomelo on 2025/12/2
//

#include "m3508_rpm.h"
#include "dji_motor.h"
#include "ladrc.h"
#include "robot_config.h"
#include <string.h>

/*====================配置参数====================*/
// 参数已移至 robot_config.h

// LADRC 控制器实例数组
static LADRC_t motor_ladrc[M3508_MOTOR_NUM];

// 目标转速缓存
static int16_t target_rpm[M3508_MOTOR_NUM] = {0};

// 控制输出缓存
static int16_t control_output[M3508_MOTOR_NUM] = {0};

// 初始化标志
static uint8_t is_initialized = 0;

/*====================内部函数====================*/

/**
 * @brief 创建 LADRC 配置结构体（一阶 LADRC，速度环）
 */
static void Create_LADRC_Config(ladrc_config_t *config) {
  memset(config, 0, sizeof(ladrc_config_t));

  // 系统阶数：一阶 LADRC（用于速度环）
  config->order = LADRC_FIRST_ORDER;

  // LESO 参数
  config->w0 = M3508_LESO_W0; // 观测器带宽
  config->b0 = M3508_LESO_B0; // 控制增益

  // PD 参数（一阶只用 kp）
  config->kp = M3508_CONTROL_KP; // 比例系数
  config->kd = 0.0f;             // 一阶不使用微分

  // 输出限幅
  config->maxout = M3508_MAX_OUTPUT;

  // 优化环节：梯形积分 + 反馈滤波 + 扰动限幅
  config->improve =
      LADRC_LESO_Trapezoid | LADRC_Feedback_LPF | LADRC_Disturbance_Limit;

  // 优化参数
  config->feedback_LPF_RC = M3508_FEEDBACK_LPF_RC;
  config->disturbance_limit = M3508_DISTURBANCE_LIMIT;
  config->disturbance_decay = M3508_DISTURBANCE_DECAY;
}

/*====================外部接口====================*/

void M3508_RPM_Init(void) {
  ladrc_config_t config;

  // 创建 LADRC 配置
  Create_LADRC_Config(&config);

  // 初始化每个电机的 LADRC 控制器
  for (uint8_t i = 0; i < M3508_MOTOR_NUM; i++) {
    LADRC_Init(&motor_ladrc[i], &config, M3508_CONTROL_DT);

    // 启用真实 dt 模式（基于 nowtime）
    LADRC_Set_Real_DT_Mode(&motor_ladrc[i], 1);

    // 初始化目标和输出
    target_rpm[i] = 0;
    control_output[i] = 0;
  }

  is_initialized = 1;
}

void M3508_Set_RPM(int16_t rpm1, int16_t rpm2, int16_t rpm3, int16_t rpm4) {
  target_rpm[0] = rpm1;
  target_rpm[1] = rpm2;
  target_rpm[2] = rpm3;
  target_rpm[3] = rpm4;
}

void M3508_RPM_Control_Loop(void) {
  if (!is_initialized) {
    return;
  }

  const motor_measure_t *measure;
  float ladrc_output;

  // 遍历每个电机
  for (uint8_t i = 0; i < M3508_MOTOR_NUM; i++) {
    // 获取电机反馈数据
    measure = get_chassis_m3508_measure_point(i);
    if (measure == NULL) {
      continue;
    }

    // LADRC 计算
    // measure: 实际转速 (RPM)
    // ref: 目标转速 (RPM)
    ladrc_output = LADRC_Calculate(&motor_ladrc[i], (float)measure->speed_rpm,
                                   (float)target_rpm[i]);

    // 转换为整型输出（四舍五入）
    control_output[i] = (int16_t)(ladrc_output + 0.5f);
  }

  // 发送 CAN 命令
  motor_control_t motors[4];
  motors[0].slot = MOTOR_SLOT_0;
  motors[1].slot = MOTOR_SLOT_1;
  motors[2].slot = MOTOR_SLOT_2;
  motors[3].slot = MOTOR_SLOT_3;
  for (int i = 0; i < 4; i++) {
    motors[i].msg_id = CAN_MSG_M3508_ID;
    motors[i].value = (int16_t)control_output[i];
  }

  CAN_Manager_SetMotors(motors, 4);
}

int16_t M3508_Get_Actual_RPM(uint8_t motor_index) {
  if (motor_index >= M3508_MOTOR_NUM) {
    return 0;
  }

  const motor_measure_t *measure = get_chassis_m3508_measure_point(motor_index);
  return (measure != NULL) ? measure->speed_rpm : 0;
}

int16_t M3508_Get_Output(uint8_t motor_index) {
  if (motor_index >= M3508_MOTOR_NUM) {
    return 0;
  }

  return control_output[motor_index];
}

float M3508_Get_Disturbance(uint8_t motor_index) {
  if (motor_index >= M3508_MOTOR_NUM) {
    return 0.0f;
  }

  return LADRC_GetDisturbance(&motor_ladrc[motor_index]);
}

void M3508_Reset_All(void) {
  for (uint8_t i = 0; i < M3508_MOTOR_NUM; i++) {
    LADRC_Reset(&motor_ladrc[i]);
    target_rpm[i] = 0;
    control_output[i] = 0;
  }
}
