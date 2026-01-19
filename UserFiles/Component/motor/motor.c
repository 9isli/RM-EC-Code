//
// Created by pomelo on 2025/11/30.
// 这里是电机集中的对外设置接口，只需要
// 考虑将已经设置好的电机can id和控制量
// 输入即可

#include "motor.h"
#include "bsp_can.h"

/**
 * @brief 设置单个电机控制值
 */
void CAN_SetMotor(can_msg_id_e msg_id, motor_slot_e slot, int16_t value) {
  motor_control_t motor = {.msg_id = msg_id, .slot = slot, .value = value};
  CAN_Manager_SetMotor(&motor);
}

/**
 * @brief 批量设置电机控制值
 */
void CAN_SetMotorS(const motor_control_t *motors, uint8_t count) {
  CAN_Manager_SetMotors(motors, count);
}

/**
 * @brief 立即发送特定 CAN 消息，跳过定时发送
 */
int32_t CAN_SendImmediate(can_msg_id_e msg_id) {
  return CAN_Manager_SendImmediate(msg_id);
}
