//
// Created by pomelo on 2025/10/14.
//

#ifndef __GETRPM_H
#define __GETRPM_H

#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief 初始化VOFA+数据采集模块
 * @details 初始化UART和相关数据采集功能
 * @retval 无
 */
extern void GetRpm_Init(void);

/**
 * @brief 更新VOFA+数据采集
 * @details 读取传感器数据并通过UART发送给VOFA+上位机
 * @retval 无
 * @note 建议定期调用（建议10ms周期）
 */
extern void GetRpm_Update(void);

#endif //__GETRPM_H
