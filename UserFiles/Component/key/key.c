/**
 * @file key.c
 * @brief 按键输入模块实现
 * @details 提供按键检测和消抖功能
 */

#include "key.h"

/**
 * @brief 读取按键状态
 * @details 检测按键是否被按下，包含消抖处理
 * @return 按键状态（1表示按下，0表示未按下）
 */
uint8_t KEY(void) {
  // 按键按下（按下为低电平）
  if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) {
    HAL_Delay(10); // 消抖
    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) {
      return 1;
    }
    HAL_Delay(10);
  }
  return 0; // 按键没按下
}
