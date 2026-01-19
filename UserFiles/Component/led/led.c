/**
 * @file led.c
 * @brief LED控制模块实现
 * @details 提供LED的初始化和控制功能
 */

#include "led.h"

/**
 * @brief 初始化LED模块
 * @details 配置LED相关的GPIO引脚，初始状态为全部熄灭
 * @retval 无
 */
void LED_Init(void) {
  LED_Red(OFF);
  LED_Green(OFF);
  LED_Blue(OFF);
}

/**
 * @brief 控制红色LED
 * @param[in] state LED状态（ON或OFF）
 * @retval 无
 */
void LED_Red(LED_State state) {
  if (state == ON) {
    HAL_GPIO_WritePin(RGB_R_GPIO_Port, RGB_R_Pin, GPIO_PIN_SET); // 高电平点亮
  } else {
    HAL_GPIO_WritePin(RGB_R_GPIO_Port, RGB_R_Pin, GPIO_PIN_RESET); // 低电平熄灭
  }
}

/**
 * @brief 控制绿色LED
 * @param[in] state LED状态（ON或OFF）
 * @retval 无
 */
void LED_Green(LED_State state) {
  if (state == ON) {
    HAL_GPIO_WritePin(RGB_G_GPIO_Port, RGB_G_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(RGB_G_GPIO_Port, RGB_G_Pin, GPIO_PIN_RESET);
  }
}

/**
 * @brief 控制蓝色LED
 * @param[in] state LED状态（ON或OFF）
 * @retval 无
 */
void LED_Blue(LED_State state) {
  if (state == ON) {
    HAL_GPIO_WritePin(RGB_B_GPIO_Port, RGB_B_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(RGB_B_GPIO_Port, RGB_B_Pin, GPIO_PIN_RESET);
  }
}
