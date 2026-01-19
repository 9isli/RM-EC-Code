//
// Created by pomelo on 2025/11/30.
//
// 通用 LADRC (线性自抗扰控制) 模块实现
// 支持一阶 LADRC（速度环）和二阶 LADRC（位置环）
// 优化功能通过位掩码按需启用
//

#include "ladrc.h"
#include <cmath>
#include <cstring>

extern "C" {

// 从 main.c 导入 nowtime（基于定时器的系统计时，单位 100μs）
extern uint32_t nowtime;

/* ========================== 内部工具函数 ========================== */

/**
 * @brief 计算真实的时间间隔（秒），自动处理 nowtime 溢出
 */
static inline float Calculate_Real_DT(uint32_t last_time,
                                      uint32_t current_time) {
  uint32_t dt_ticks = current_time - last_time;

  if (dt_ticks == 0) {
    return 0.0f;
  }

  if (dt_ticks > 100000000u) {
    return 0.001f; // 回退到 1ms
  }

  return (float)dt_ticks * 100e-6f;
}

/**
 * @brief 限幅函数
 */
static inline float Limit(float val, float limit) {
  if (limit <= 0.0f)
    return val;
  if (val > limit)
    return limit;
  if (val < -limit)
    return -limit;
  return val;
}

/* ========================== 优化环节实现 ========================== */

/**
 * @brief 反馈信号低通滤波
 * @note  一阶低通滤波，滤除测量噪声
 */
static void Feedback_LPF(LADRC_t *ladrc) {
  float alpha = ladrc->dt / (ladrc->feedback_LPF_RC + ladrc->dt);
  ladrc->filtered_measure =
      alpha * ladrc->measure + (1.0f - alpha) * ladrc->filtered_measure;
}

/**
 * @brief 微分先行
 * @note  对测量值微分而非误差，避免目标值突变时产生微分尖峰
 */
static void Derivative_On_Measurement(LADRC_t *ladrc) {
  ladrc->dout =
      ladrc->kd * (ladrc->lastMeasure - ladrc->filtered_measure) / ladrc->dt;
}

/**
 * @brief 微分滤波
 * @note  一阶低通滤波，滤除高频噪声
 */
static void Derivative_Filter(LADRC_t *ladrc) {
  float alpha = ladrc->dt / (ladrc->derivative_LPF_RC + ladrc->dt);
  ladrc->dout = alpha * ladrc->dout + (1.0f - alpha) * ladrc->lastDout;
}

/**
 * @brief 扰动限幅和衰减
 * @note  限制扰动估计的幅度，防止过补偿
 */
static float Disturbance_Process(LADRC_t *ladrc, float disturbance) {
  // 衰减
  float processed = disturbance * ladrc->disturbance_decay;

  // 限幅
  if (ladrc->disturbance_limit > 0.0f) {
    processed = Limit(processed, ladrc->disturbance_limit);
  }

  return processed;
}

/**
 * @brief 输出限幅
 */
static void Output_Limit(LADRC_t *ladrc) {
  ladrc->output = Limit(ladrc->output, ladrc->maxout);
}

/* ========================== LESO 更新 ========================== */

/**
 * @brief 二阶 LESO 更新（用于一阶 LADRC）
 * @note  观测: z1 ≈ y (输出), z2 ≈ f (总扰动)
 *        模型: ẏ = f + b0*u
 */
static void LESO_2nd_Update(LADRC_t *ladrc, float y, float u) {
  float dt = ladrc->dt;
  float e = ladrc->z1 - y;

  // 计算导数
  float z1_dot = ladrc->z2 - ladrc->beta1 * e + ladrc->b0 * u;
  float z2_dot = -ladrc->beta2 * e;

  // 积分更新
  if (ladrc->improve & LADRC_LESO_Trapezoid) {
    // 梯形积分: x += (x_dot + last_x_dot) / 2 * dt
    ladrc->z1 += (z1_dot + ladrc->last_z1_dot) * 0.5f * dt;
    ladrc->z2 += (z2_dot + ladrc->last_z2_dot) * 0.5f * dt;

    // 保存导数历史
    ladrc->last_z1_dot = z1_dot;
    ladrc->last_z2_dot = z2_dot;
  } else {
    // 欧拉积分
    ladrc->z1 += z1_dot * dt;
    ladrc->z2 += z2_dot * dt;
  }
}

/**
 * @brief 三阶 LESO 更新（用于二阶 LADRC）
 * @note  观测: z1 ≈ y (位置), z2 ≈ ẏ (速度), z3 ≈ f (总扰动)
 *        模型: ÿ = f + b0*u
 */
static void LESO_3rd_Update(LADRC_t *ladrc, float y, float u) {
  float dt = ladrc->dt;
  float e = ladrc->z1 - y;

  // 计算导数
  float z1_dot = ladrc->z2 - ladrc->beta1 * e;
  float z2_dot = ladrc->z3 - ladrc->beta2 * e + ladrc->b0 * u;
  float z3_dot = -ladrc->beta3 * e;

  // 积分更新
  if (ladrc->improve & LADRC_LESO_Trapezoid) {
    // 梯形积分
    ladrc->z1 += (z1_dot + ladrc->last_z1_dot) * 0.5f * dt;
    ladrc->z2 += (z2_dot + ladrc->last_z2_dot) * 0.5f * dt;
    ladrc->z3 += (z3_dot + ladrc->last_z3_dot) * 0.5f * dt;

    // 保存导数历史
    ladrc->last_z1_dot = z1_dot;
    ladrc->last_z2_dot = z2_dot;
    ladrc->last_z3_dot = z3_dot;
  } else {
    // 欧拉积分
    ladrc->z1 += z1_dot * dt;
    ladrc->z2 += z2_dot * dt;
    ladrc->z3 += z3_dot * dt;
  }
}

/* ========================== 控制律计算 ========================== */

/**
 * @brief 一阶 LADRC 控制律（P + 扰动补偿）
 * @note  u0 = kp * (ref - z1)
 *        u = (u0 - z2) / b0
 */
static float LADRC_1st_Control(LADRC_t *ladrc) {
  // P 控制
  ladrc->u0 = ladrc->kp * (ladrc->ref - ladrc->z1);

  // 获取扰动估计
  float disturbance = ladrc->z2;

  // 扰动处理
  if (ladrc->improve & LADRC_Disturbance_Limit) {
    disturbance = Disturbance_Process(ladrc, disturbance);
  }

  // 扰动补偿
  if (ladrc->b0 != 0.0f) {
    ladrc->u = (ladrc->u0 - disturbance) / ladrc->b0;
  } else {
    ladrc->u = ladrc->u0;
  }

  return ladrc->u;
}

/**
 * @brief 二阶 LADRC 控制律（PD + 扰动补偿）
 * @note  u0 = kp * (ref - z1) - kd * z2
 *        u = (u0 - z3) / b0
 */
static float LADRC_2nd_Control(LADRC_t *ladrc) {
  // 误差
  ladrc->err = ladrc->ref - ladrc->z1;

  // P 项
  float pout = ladrc->kp * ladrc->err;

  // D 项
  if (ladrc->improve & LADRC_Derivative_On_Measurement) {
    // 微分先行：对测量值微分
    Derivative_On_Measurement(ladrc);
  } else {
    // 标准微分：使用 LESO 观测的速度
    ladrc->dout = ladrc->kd * ladrc->z2;
  }

  // 微分滤波
  if (ladrc->improve & LADRC_DerivativeFilter) {
    Derivative_Filter(ladrc);
  }

  // 控制律: u0 = kp * e - kd * z2
  ladrc->u0 = pout - ladrc->dout;

  // 获取扰动估计
  float disturbance = ladrc->z3;

  // 扰动处理
  if (ladrc->improve & LADRC_Disturbance_Limit) {
    disturbance = Disturbance_Process(ladrc, disturbance);
  }

  // 扰动补偿
  if (ladrc->b0 != 0.0f) {
    ladrc->u = (ladrc->u0 - disturbance) / ladrc->b0;
  } else {
    ladrc->u = ladrc->u0;
  }

  return ladrc->u;
}

/* ========================== LESO 增益计算 ========================== */

/**
 * @brief 计算 LESO 增益（基于带宽配置法）
 * @note  一阶 LADRC: beta1 = 2*w0, beta2 = w0^2
 *        二阶 LADRC: beta1 = 3*w0, beta2 = 3*w0^2, beta3 = w0^3
 */
static void Calculate_LESO_Gains(LADRC_t *ladrc) {
  float w0 = ladrc->w0;
  float w0_2 = w0 * w0;

  if (ladrc->order == LADRC_FIRST_ORDER) {
    // 二阶 LESO 增益
    ladrc->beta1 = 2.0f * w0;
    ladrc->beta2 = w0_2;
    ladrc->beta3 = 0.0f;
  } else {
    // 三阶 LESO 增益
    ladrc->beta1 = 3.0f * w0;
    ladrc->beta2 = 3.0f * w0_2;
    ladrc->beta3 = w0_2 * w0;
  }
}

/* ========================== 外部接口 ========================== */

void LADRC_Init(LADRC_t *ladrc, const ladrc_config_t *config, float dt) {
  std::memset(ladrc, 0, sizeof(LADRC_t));

  // 复制配置参数
  ladrc->order = config->order;
  ladrc->w0 = config->w0;
  ladrc->b0 = config->b0;
  ladrc->kp = config->kp;
  ladrc->kd = config->kd;
  ladrc->maxout = config->maxout;

  ladrc->improve = config->improve;
  ladrc->deadBand = config->deadBand;
  ladrc->feedback_LPF_RC = config->feedback_LPF_RC;
  ladrc->derivative_LPF_RC = config->derivative_LPF_RC;
  ladrc->disturbance_limit = config->disturbance_limit;
  ladrc->disturbance_decay =
      (config->disturbance_decay > 0.0f) ? config->disturbance_decay : 1.0f;

  ladrc->dt = dt;

  // 计算 LESO 增益
  Calculate_LESO_Gains(ladrc);

  // 初始化时间相关
  ladrc->last_update_time = nowtime;
  ladrc->use_real_dt = 1; // 默认启用真实 dt
}

float LADRC_Calculate(LADRC_t *ladrc, float measure, float ref) {
  // 保存输入
  ladrc->measure = measure;
  ladrc->ref = ref;

  // 计算真实 dt
  if (ladrc->use_real_dt) {
    uint32_t current_time = nowtime;
    float real_dt = Calculate_Real_DT(ladrc->last_update_time, current_time);
    ladrc->last_update_time = current_time;

    if (real_dt > 0.0f && real_dt < 1.0f) {
      ladrc->dt = real_dt;
    }
  }

  // 反馈滤波
  if (ladrc->improve & LADRC_Feedback_LPF) {
    Feedback_LPF(ladrc);
  } else {
    ladrc->filtered_measure = measure;
  }

  // 死区判断
  float err_for_deadband = ref - ladrc->filtered_measure;
  if ((ladrc->improve & LADRC_DeadBand) &&
      std::fabs(err_for_deadband) < ladrc->deadBand) {
    ladrc->output = ladrc->lastOutput;
    ladrc->lastMeasure = ladrc->filtered_measure;
    ladrc->lastErr = ladrc->err;
    return ladrc->output;
  }

  // 根据阶数选择 LESO 和控制律
  if (ladrc->order == LADRC_FIRST_ORDER) {
    // 一阶 LADRC
    LESO_2nd_Update(ladrc, ladrc->filtered_measure, ladrc->lastU);
    ladrc->output = LADRC_1st_Control(ladrc);
  } else {
    // 二阶 LADRC
    LESO_3rd_Update(ladrc, ladrc->filtered_measure, ladrc->lastU);
    ladrc->output = LADRC_2nd_Control(ladrc);
  }

  // 输出限幅
  Output_Limit(ladrc);

  // 保存历史
  ladrc->lastMeasure = ladrc->filtered_measure;
  ladrc->lastOutput = ladrc->output;
  ladrc->lastU = ladrc->u;
  ladrc->lastDout = ladrc->dout;
  ladrc->lastErr = ladrc->err;

  return ladrc->output;
}

void LADRC_Reset(LADRC_t *ladrc) {
  // 重置 LESO 状态
  ladrc->z1 = 0.0f;
  ladrc->z2 = 0.0f;
  ladrc->z3 = 0.0f;

  // 重置梯形积分历史
  ladrc->last_z1_dot = 0.0f;
  ladrc->last_z2_dot = 0.0f;
  ladrc->last_z3_dot = 0.0f;

  // 重置控制状态
  ladrc->err = 0.0f;
  ladrc->lastErr = 0.0f;
  ladrc->u0 = 0.0f;
  ladrc->u = 0.0f;
  ladrc->lastU = 0.0f;
  ladrc->dout = 0.0f;
  ladrc->lastDout = 0.0f;
  ladrc->output = 0.0f;
  ladrc->lastOutput = 0.0f;
  ladrc->filtered_measure = 0.0f;
  ladrc->lastMeasure = 0.0f;

  // 重置时间戳
  ladrc->last_update_time = nowtime;
}

void LADRC_Reset_With_State(LADRC_t *ladrc, float init_state) {
  LADRC_Reset(ladrc);

  // 用当前测量值初始化 LESO
  ladrc->z1 = init_state;
  ladrc->filtered_measure = init_state;
  ladrc->lastMeasure = init_state;
}

void LADRC_SetLESO(LADRC_t *ladrc, float w0, float b0) {
  ladrc->w0 = w0;
  ladrc->b0 = b0;
  Calculate_LESO_Gains(ladrc);
}

void LADRC_SetPD(LADRC_t *ladrc, float kp, float kd) {
  ladrc->kp = kp;
  ladrc->kd = kd;
}

void LADRC_SetMaxout(LADRC_t *ladrc, float maxout) { ladrc->maxout = maxout; }

void LADRC_SetOrder(LADRC_t *ladrc, ladrc_order_e order) {
  if (ladrc->order != order) {
    ladrc->order = order;
    Calculate_LESO_Gains(ladrc);
    LADRC_Reset(ladrc);
  }
}

void LADRC_Set_Real_DT_Mode(LADRC_t *ladrc, uint8_t enable) {
  ladrc->use_real_dt = (enable != 0) ? 1 : 0;
  if (enable) {
    ladrc->last_update_time = nowtime;
  }
}

float LADRC_GetDisturbance(const LADRC_t *ladrc) {
  if (ladrc->order == LADRC_FIRST_ORDER) {
    return ladrc->z2;
  } else {
    return ladrc->z3;
  }
}

float LADRC_GetVelocity(const LADRC_t *ladrc) { return ladrc->z2; }

} // extern "C"
