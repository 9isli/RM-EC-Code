#ifndef __M2006_RPM_H
#define __M2006_RPM_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief M2006 LADRC 子系统初始化
 * @details 初始化4个M2006电机的LADRC速度控制器
 *          使用一阶LADRC（二阶LESO）
 *          默认开启以下优化：
 *          - LESO梯形积分
 *          - 反馈信号低通滤波
 *          - 扰动幅度限幅
 * @retval 无
 */
void M2006_RPM_Init(void);

/**
 * @brief 设置四个电机的目标转速
 * @param[in] rpm1 电机1目标转速（RPM）
 * @retval 无
 */
void M2006_Set_RPM(int16_t rpm1);

/**
 * @brief LADRC 控制循环
 * @details 读取电机反馈、计算LADRC输出、发送CAN命令
 * @retval 无
 * @note 建议在1kHz定时器中断中调用（1ms周期）
 */
void M2006_RPM_Control_Loop(void);

/**
 * @brief 获取电机实际转速
 * @param[in] motor_index 电机编号 [0-3]
 * @return 电机实际转速（RPM）
 */
int16_t M2006_Get_Actual_RPM(uint8_t motor_index);

/**
 * @brief 获取电机控制输出
 * @param[in] motor_index 电机编号 [0-3]
 * @return LADRC 控制输出值
 */
int16_t M2006_Get_Output(uint8_t motor_index);

/**
 * @brief 获取LESO扰动估计值
 * @param[in] motor_index 电机编号 [0-3]
 * @return 扰动估计值
 */
float M2006_Get_Disturbance(uint8_t motor_index);

/**
 * @brief 复位所有电机的LADRC状态
 * @retval 无
 */
void M2006_Reset_All(void);

#ifdef __cplusplus
}
#endif

#endif //__M2006_RPM_H