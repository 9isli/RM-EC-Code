//
// Created by pomelo on 2025/11/30.
//

#include "dji_motor.h"
#include "bsp_can.h"
#include "main.h"

/**
 * @brief 电机数据索引说明
 * @details
 *   0: 底盘电机1 (3508)
 *   1: 底盘电机2 (3508)
 *   2: 底盘电机3 (3508)
 *   3: 底盘电机4 (3508)
 *   4: Yaw云台电机 (6020)
 *   5: Pitch云台电机 (6020)
 *   6: 拨弹电机 (2006)
 */
motor_measure_t dji_motor[7];

/*========DJI官方代码========*/

/**
 * @brief 设置底盘 M3508 电机控制电流（向后兼容）
 * @details 使用统一 CAN 管理器替代原有实现
 */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3,
                     int16_t motor4) {
  motor_control_t motors[] = {
      {CAN_MSG_M3508_ID, MOTOR_SLOT_0, motor1},
      {CAN_MSG_M3508_ID, MOTOR_SLOT_1, motor2},
      {CAN_MSG_M3508_ID, MOTOR_SLOT_2, motor3},
      {CAN_MSG_M3508_ID, MOTOR_SLOT_3, motor4},
  };
  CAN_Manager_SetMotors(motors, 4);
}

/**
 * @brief 设置云台 GM6020 电机控制电流（向后兼容）
 */
void CAN_cmd_gimbal(int16_t yaw1, int16_t pitch2, int16_t gm6020_3,
                    int16_t gm6020_4) {
  motor_control_t motors[] = {
      {CAN_MSG_GM6020_ID, MOTOR_SLOT_0, yaw1},
      {CAN_MSG_GM6020_ID, MOTOR_SLOT_1, pitch2},
      {CAN_MSG_GM6020_ID, MOTOR_SLOT_2, gm6020_3},
      {CAN_MSG_GM6020_ID, MOTOR_SLOT_3, gm6020_4},
  };
  CAN_Manager_SetMotors(motors, 4);
}

/**
 * @brief 设置拨弹 M2006 电机控制电流（向后兼容）
 */
void CAN_cmd_shoot(int16_t shoot1_current, int16_t shoot2_current,
                   int16_t shoot3_current, int16_t rev) {
  motor_control_t motors[] = {
      {CAN_MSG_M2006_ID, MOTOR_SLOT_0, shoot1_current},
      {CAN_MSG_M2006_ID, MOTOR_SLOT_1, shoot2_current},
      {CAN_MSG_M2006_ID, MOTOR_SLOT_2, shoot3_current},
  };
  CAN_Manager_SetMotors(motors, 3);
  (void)rev; // 未使用
}

/**
 * @brief 发送底盘电机 ID 重置命令
 */
void CAN_cmd_chassis_reset_ID(void) {
  extern CAN_HandleTypeDef hcan1;
  uint32_t send_mail_box;

  CAN_TxHeaderTypeDef tx_header = {
      .StdId = 0x700, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 0x08};

  uint8_t reset_data[8] = {0};

  HAL_CAN_AddTxMessage(&hcan1, &tx_header, reset_data, &send_mail_box);
}

/**
 * @brief 获取Yaw云台电机数据指针
 * @return 指向Yaw电机测量数据的指针
 */
const motor_measure_t *get_yaw_gimbal_gm6020_measure_point(void) {
  return &dji_motor[4];
}

/**
 * @brief 获取Pitch云台电机数据指针
 * @return 指向Pitch电机测量数据的指针
 */
const motor_measure_t *get_pitch_gimbal_gm6020_measure_point(void) {
  return &dji_motor[5];
}

/**
 * @brief 获取拨弹电机数据指针
 * @return 指向拨弹电机测量数据的指针
 */
const motor_measure_t *get_trigger_m2006_measure_point(void) {
  return &dji_motor[2]; // 底盘电机3，云台2006复用
}

/**
 * @brief 获取底盘电机数据指针
 * @param[in] i 电机编号，范围[0, 3]
 * @return 指向指定电机测量数据的指针
 */
const motor_measure_t *get_chassis_m3508_measure_point(uint8_t i) {
  return &dji_motor[(i & 0x03)];
}
