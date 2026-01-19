//
// Created by pomelo on 2025/9/5.
//

#ifndef __LED_H
#define __LED_H

#include "main.h"

/**
 * @brief LED状态枚举
 */
typedef enum { OFF = 0, ON = 1 } LED_State;

/**
 * @brief 初始化LED模块
 * @details 配置LED相关的GPIO引脚
 * @retval 无
 */
void LED_Init(void);

/**
 * @brief 控制红色LED
 * @param[in] state LED状态（ON或OFF）
 * @retval 无
 */
void LED_Red(LED_State state);

/**
 * @brief 控制绿色LED
 * @param[in] state LED状态（ON或OFF）
 * @retval 无
 */
void LED_Green(LED_State state);

/**
 * @brief 控制蓝色LED
 * @param[in] state LED状态（ON或OFF）
 * @retval 无
 */
void LED_Blue(LED_State state);

#endif //__LED_H