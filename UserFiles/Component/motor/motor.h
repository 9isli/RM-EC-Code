//
// Created by pomelo on 2025/11/30.
//

#ifndef __MOTOR_H
#define __MOTOR_H

#include "bsp_can.h"
#include "main.h"

/**
 * @brief 设置单个电机控制值
 * @param[in] msg_id 消息 ID（0x200, 0x1FF, 0x2FF）
 * @param[in] slot 电机在消息中的位置（0-3）
 * @param[in] value 控制值
 * @retval 无
 */
void CAN_SetMotor(can_msg_id_e msg_id, motor_slot_e slot, int16_t value);

/**
 * @brief 批量设置电机控制值
 * @param[in] motors 电机数组
 * @param[in] count 数量
 * @retval 无
 */
void CAN_SetMotorS(const motor_control_t *motors, uint8_t count);

/**
 * @brief 立即发送特定 CAN 消息
 */
int32_t CAN_SendImmediate(can_msg_id_e msg_id);

#endif //__MOTOR_H
