#include "bsp_dwt.h"

static uint32_t cpu_freq_mhz = 168;

void DWT_Init(uint32_t _cpu_freq_mhz) {
  cpu_freq_mhz = _cpu_freq_mhz;
  // 使能 DWT 计数器
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// 获取两次调用间的时间差(秒)
float DWT_Get_Delta_T(uint32_t *cnt_last) {
  uint32_t cnt_now = DWT->CYCCNT;
  float dt = ((float)(cnt_now - *cnt_last)) / (cpu_freq_mhz * 1000000.0f);
  *cnt_last = cnt_now;
  return dt;
}

// 微秒级延时
void DWT_Delay_us(float us) {
  uint32_t ticks = us * cpu_freq_mhz;
  uint32_t start_tick = DWT->CYCCNT;
  while (DWT->CYCCNT - start_tick < ticks)
    ;
}