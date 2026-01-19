//
// Created by pomelo on 2025/11/17.
//

#include "buzzer.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;

void Buzzer_On(uint16_t psc, uint16_t pwm) {
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  __HAL_TIM_SET_PRESCALER(&htim4, psc);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm);
}

void Buzzer_Off(void) { __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); }

void Buzzer_SetFrequency(float freq_hz) {
  if (freq_hz <= 0.0f) {
    Buzzer_Off();
    return;
  }

  const uint32_t tim_clk = 84000000U; // TIM4 clock (APB1 timer clock)
  float cycles = tim_clk / freq_hz;

  uint32_t prescaler =
      (uint32_t)(cycles / 65536.0f) + 1U; // ensure ARR <= 0xFFFF
  if (prescaler < 1U)
    prescaler = 1U;
  if (prescaler > 0xFFFFU)
    prescaler = 0xFFFFU;

  uint32_t period = (uint32_t)((cycles / prescaler) + 0.5f);
  if (period < 1U)
    period = 1U;
  if (period > 0x10000U)
    period = 0x10000U;
  period -= 1U;

  HAL_TIM_Base_Stop(&htim4);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);

  __HAL_TIM_SET_PRESCALER(&htim4, prescaler - 1U);
  __HAL_TIM_SET_AUTORELOAD(&htim4, period);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (period + 1U) / 2U);

  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}
