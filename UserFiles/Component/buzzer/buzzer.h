//
// Created by pomelo on 2025/11/17.
//

#ifndef __BUZZER_H
#define __BUZZER_H

#include "main.h"

extern void Buzzer_On(uint16_t psc, uint16_t pwm);
extern void Buzzer_Off(void);
extern void Buzzer_SetFrequency(float freq_hz);

#endif //__BUZZER_H