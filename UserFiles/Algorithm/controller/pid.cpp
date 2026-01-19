//
// Created by pomelo on 2025/11/29.
// 参考湖南大学的PID实现风格
// 支持位置式和增量式两种模式
//

#include "pid.h"
#include <cmath>
#include <cstring>

extern "C" {

// 从 main.c 导入 nowtime（基于定时器的系统计时，单位 100μs）
extern uint32_t nowtime;

/**
 * @brief 计算真实的时间间隔（秒），自动处理 nowtime 溢出
 * @param last_time 上次的时间戳
 * @param current_time 当前的时间戳
 * @return 时间间隔（秒）
 * @note 基于无符号整数的模运算，自动处理溢出
 *       nowtime 单位是 100μs，所以要乘以 100e-6f 转换为秒
 */
static inline float Calculate_Real_DT(uint32_t last_time,
                                      uint32_t current_time) {
  // 无符号减法自动处理溢出（基于模 2^32 运算）
  uint32_t dt_ticks = current_time - last_time;

  // 防护：检查 dt 是否合理
  if (dt_ticks == 0) {
    // 函数被调用了多次但定时器没有更新
    return 0.0f;
  }

  if (dt_ticks > 100000000u) {
    // dt > 10秒，严重异常
    return 0.001f; // 回退到 1ms
  }

  // 转换为秒（nowtime 单位是 100μs）
  float real_dt = (float)dt_ticks * 100e-6f;

  return real_dt;
}
/* ========================== 优化环节实现 ========================== */

/**
 * @brief 梯形积分
 * @note  使用梯形面积公式: (上底 + 下底) * 高 / 2
 */
static void Trapezoid_Integral(PID_t *pid) {
  pid->iTerm = pid->ki * ((pid->err + pid->lastErr) / 2.0f) * pid->dt;
}

/**
 * @brief 变速积分
 * @note  误差小时积分作用强，误差大时积分作用弱或为零
 *        ITerm *= (A - |err| + B) / A, when B < |err| < A + B
 */
static void Changing_Integration_Rate(PID_t *pid) {
  if (pid->err * pid->iout > 0) {
    // 积分呈累积趋势
    float abs_err = std::fabs(pid->err);
    if (abs_err <= pid->coefB) {
      // 误差很小，全积分
      return;
    }
    if (abs_err <= (pid->coefA + pid->coefB)) {
      // 误差中等，部分积分
      pid->iTerm *= (pid->coefA - abs_err + pid->coefB) / pid->coefA;
    } else {
      // 误差过大，不积分
      pid->iTerm = 0;
    }
  }
}

/**
 * @brief 积分限幅
 * @note  防止积分饱和，同时实现抗饱和（当输出饱和且积分累积时停止积分）
 */
static void Integral_Limit(PID_t *pid) {
  float temp_Iout = pid->iout + pid->iTerm;
  float temp_Output = pid->pout + pid->iout + pid->dout;

  // 抗饱和：输出饱和且积分仍在累积时，停止积分
  if (std::fabs(temp_Output) > pid->maxout) {
    if (pid->err * pid->iout > 0) {
      pid->iTerm = 0;
    }
  }

  // 积分限幅
  if (temp_Iout > pid->integralLimit) {
    pid->iTerm = 0;
    pid->iout = pid->integralLimit;
  } else if (temp_Iout < -pid->integralLimit) {
    pid->iTerm = 0;
    pid->iout = -pid->integralLimit;
  }
}

/**
 * @brief 微分先行
 * @note  对测量值微分而非误差，避免目标值突变时产生微分尖峰
 */
static void Derivative_On_Measurement(PID_t *pid) {
  pid->dout = pid->kd * (pid->lastMeasure - pid->measure) / pid->dt;
}

/**
 * @brief 微分滤波
 * @note  一阶低通滤波，滤除高频噪声
 */
static void Derivative_Filter(PID_t *pid) {
  float alpha = pid->dt / (pid->derivative_LPF_RC + pid->dt);
  pid->dout = alpha * pid->dout + (1.0f - alpha) * pid->lastDout;
}

/**
 * @brief 输出滤波
 * @note  一阶低通滤波，平滑输出
 */
static void Output_Filter(PID_t *pid) {
  float alpha = pid->dt / (pid->output_LPF_RC + pid->dt);
  pid->output = alpha * pid->output + (1.0f - alpha) * pid->lastOutput;
}

/**
 * @brief 输出限幅
 */
static void Output_Limit(PID_t *pid) {
  if (pid->output > pid->maxout) {
    pid->output = pid->maxout;
  } else if (pid->output < -pid->maxout) {
    pid->output = -pid->maxout;
  }
}

/* ========================== 位置式 PID ========================== */

static float PID_Position_Calculate(PID_t *pid) {
  // P 项
  pid->pout = pid->kp * pid->err;

  // I 项
  pid->iTerm = pid->ki * pid->err * pid->dt;

  // 梯形积分
  if (pid->improve & PID_Trapezoid_Integral) {
    Trapezoid_Integral(pid);
  }

  // 变速积分
  if (pid->improve & PID_ChangingIntegrationRate) {
    Changing_Integration_Rate(pid);
  }

  // D 项
  if (pid->improve & PID_Derivative_On_Measurement) {
    Derivative_On_Measurement(pid);
  } else {
    pid->dout = pid->kd * (pid->err - pid->lastErr) / pid->dt;
  }

  // 微分滤波
  if (pid->improve & PID_DerivativeFilter) {
    Derivative_Filter(pid);
  }

  // 积分限幅（含抗饱和）
  if (pid->improve & PID_Integral_Limit) {
    Integral_Limit(pid);
  }

  // 累加积分
  pid->iout += pid->iTerm;

  // 简单积分限幅（如果没有启用高级积分限幅）
  if (!(pid->improve & PID_Integral_Limit)) {
    if (pid->iout > pid->integralLimit) {
      pid->iout = pid->integralLimit;
    } else if (pid->iout < -pid->integralLimit) {
      pid->iout = -pid->integralLimit;
    }
  }

  // 输出
  pid->output = pid->pout + pid->iout + pid->dout;

  return pid->output;
}

/* ========================== 增量式 PID ========================== */

static float PID_Delta_Calculate(PID_t *pid) {
  // 增量式 PID 公式:
  // Δu(k) = Kp * [e(k) - e(k-1)] + Ki * e(k) + Kd * [e(k) - 2*e(k-1) + e(k-2)]
  // u(k) = u(k-1) + Δu(k)

  // P 增量
  pid->pout = pid->kp * (pid->err - pid->lastErr);

  // I 增量
  pid->iTerm = pid->ki * pid->err * pid->dt;

  // 变速积分
  if (pid->improve & PID_ChangingIntegrationRate) {
    Changing_Integration_Rate(pid);
  }

  pid->iout = pid->iTerm; // 增量式中 Iout 就是本次的 I 增量

  // D 增量
  if (pid->improve & PID_Derivative_On_Measurement) {
    // 微分先行（增量式版本）
    pid->dout = pid->kd *
                (2.0f * pid->lastMeasure - pid->measure - pid->lastLastErr) /
                pid->dt;
    // 注意：这里借用 Last_Last_Err 存储上上次的 Measure
  } else {
    // 标准增量式微分
    pid->dout =
        pid->kd * (pid->err - 2.0f * pid->lastErr + pid->lastLastErr) / pid->dt;
  }

  // 微分滤波
  if (pid->improve & PID_DerivativeFilter) {
    Derivative_Filter(pid);
  }

  // 输出增量
  float delta_output = pid->pout + pid->iout + pid->dout;

  // 累加到输出
  pid->output = pid->lastOutput + delta_output;

  return pid->output;
}

/* ========================== 外部接口 ========================== */

void PID_Init(PID_t *pid, const pid_config_t *config, float dt) {
  std::memset(pid, 0, sizeof(PID_t));

  // 复制配置参数
  pid->kp = config->kp;
  pid->ki = config->ki;
  pid->kd = config->kd;
  pid->maxout = config->maxout;
  pid->integralLimit = config->integralLimit;

  pid->mode = config->mode;
  pid->improve = config->improve;

  pid->deadBand = config->deadBand;
  pid->coefA = config->coefA;
  pid->coefB = config->coefB;
  pid->derivative_LPF_RC = config->derivative_LPF_RC;
  pid->output_LPF_RC = config->output_LPF_RC;

  pid->dt = dt;

  // 初始化真实时间积分相关字段
  pid->last_update_time = nowtime; // 初始化为当前时间戳
  pid->use_real_dt = 1;            // 默认启用真实 dt（1=启用，0=禁用）
}

float PID_Calculate(PID_t *pid, float measure, float ref) {
  // 保存当前值
  pid->measure = measure;
  pid->ref = ref;
  pid->err = ref - measure;

  // ⭐ 计算真实 dt（基于 nowtime）
  if (pid->use_real_dt) {
    uint32_t current_time = nowtime;
    float real_dt = Calculate_Real_DT(pid->last_update_time, current_time);
    pid->last_update_time = current_time;

    // 如果 dt 有效，使用真实 dt，否则保持原来的 dt
    if (real_dt > 0.0f && real_dt < 1.0f) {
      pid->dt = real_dt; // 更新为真实 dt
    }
  }

  // 死区判断
  if ((pid->improve & PID_DeadBand) && std::fabs(pid->err) < pid->deadBand) {
    pid->output = (pid->mode == PID_DELTA) ? pid->lastOutput : 0;
    pid->iTerm = 0;
    // 更新历史值
    pid->lastLastErr = pid->lastErr;
    pid->lastMeasure = measure;
    pid->lastErr = pid->err;
    return pid->output;
  }

  // 根据模式选择计算方法
  if (pid->mode == PID_POSITION) {
    PID_Position_Calculate(pid);
  } else {
    PID_Delta_Calculate(pid);
  }

  // 输出滤波
  if (pid->improve & PID_OutputFilter) {
    Output_Filter(pid);
  }

  // 输出限幅
  Output_Limit(pid);

  // 保存历史
  pid->lastLastErr = pid->lastErr;
  pid->lastMeasure = measure;
  pid->lastOutput = pid->output;
  pid->lastDout = pid->dout;
  pid->lastErr = pid->err;

  return pid->output;
}

void PID_Reset(PID_t *pid) {
  pid->err = 0;
  pid->lastErr = 0;
  pid->lastLastErr = 0;
  pid->iout = 0;
  pid->iTerm = 0;
  pid->dout = 0;
  pid->lastDout = 0;
  pid->pout = 0;
  pid->output = 0;
  pid->lastOutput = 0;
  pid->lastMeasure = 0;

  // 重置时间戳
  pid->last_update_time = nowtime;
}

void PID_SetParams(PID_t *pid, float kp, float ki, float kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

void PID_SetIntegral(PID_t *pid, float integral) { pid->iout = integral; }

void PID_SetMode(PID_t *pid, pid_mode_e mode) {
  if (pid->mode != mode) {
    pid->mode = mode;
    PID_Reset(pid); // 切换模式时重置状态
  }
}

/**
 * @brief 启用或禁用真实 dt 模式
 * @param pid    PID 实例指针
 * @param enable 1=启用真实 dt（基于 nowtime），0=禁用（使用固定 dt）
 * @note 在需要自适应控制周期时启用
 */
void PID_Set_Real_DT_Mode(PID_t *pid, uint8_t enable) {
  pid->use_real_dt = (enable != 0) ? 1 : 0;
  if (enable) {
    pid->last_update_time = nowtime; // 重新初始化时间戳
  }
}

} // extern "C"
