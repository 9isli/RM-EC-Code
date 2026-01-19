#ifndef BSP_DWT_H
#define BSP_DWT_H
#include "main.h"

void DWT_Init(uint32_t cpu_freq_mhz);
float DWT_Get_Delta_T(uint32_t *cnt_last);
void DWT_Delay_us(float us);

#endif