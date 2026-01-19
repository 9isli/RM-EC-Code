//
// DJI 电机 CAN 适配层
// 提供向后兼容的 API，底层使用统一 CAN 管理器
// Created by pomelo on 2025/11/30.
//

#ifndef __DJI_MOTOR_CAN_H
#define __DJI_MOTOR_CAN_H

#include "bsp_can.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {

#endif

/**
 * @brief 从CAN数据提取电机测量数据
 * @details 宏定义，用于从CAN接收的8字节数据中提取电机反馈信息
 */
#define get_motor_measure(ptr, data)                                           \
  {                                                                            \
    (ptr)->last_ecd = (ptr)->ecd;                                              \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);                       \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);                 \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);             \
    (ptr)->temperate = (data)[6];                                              \
  }

/**
 * @brief CAN消息ID枚举
 * @details 定义所有CAN消息的ID
 */
typedef enum {
  CAN_CHASSIS_ALL_ID = 0x200, /**< 底盘所有电机控制ID */
  CAN_3508_M1_ID = 0x201,     /**< 底盘电机1反馈ID */
  CAN_3508_M2_ID = 0x202,     /**< 底盘电机2反馈ID */
  CAN_3508_M3_ID = 0x203,     /**< 底盘电机3反馈ID */
  CAN_3508_M4_ID = 0x204,     /**< 底盘电机4反馈ID */

  CAN_YAW_MOTOR_ID = 0x205,     /**< 云台Yaw电机反馈ID */
  CAN_PIT_MOTOR_ID = 0x206,     /**< 云台Pitch电机反馈ID */
  CAN_TRIGGER_MOTOR_ID = 0x207, /**< 拨弹电机反馈ID 与底盘电机公用，注意区分 */
  CAN_GIMBAL_ALL_ID = 0x1FF,    /**< 云台所有电机控制ID */
} dji_msg_id_e;

/**
 * @brief 电机测量数据结构体
 * @details 存储从CAN总线接收的电机反馈数据
 */
typedef struct {
  uint16_t ecd;          /**< 编码器值（0-8191） */
  int16_t speed_rpm;     /**< 转速（RPM） */
  int16_t given_current; /**< 给定电流 */
  uint8_t temperate;     /**< 电机温度（°C） */
  int16_t last_ecd;      /**< 上一次编码器值 */
} motor_measure_t;

extern motor_measure_t dji_motor[7];

/**
 * @brief 设置底盘 M3508 电机控制电流（向后兼容）
 * @param[in] motor1 电机1控制电流，范围[-16384, 16384]
 * @param[in] motor2 电机2控制电流，范围[-16384, 16384]
 * @param[in] motor3 电机3控制电流，范围[-16384, 16384]
 * @param[in] motor4 电机4控制电流，范围[-16384, 16384]
 * @retval 无
 */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3,
                     int16_t motor4);

/**
 * @brief 设置云台 GM6020 电机控制电流（向后兼容）
 * @param[in] yaw1 电机1（Yaw）控制电流
 * @param[in] pitch2 电机2（Pitch）控制电流
 * @param[in] gm6020_3 电机3控制电流
 * @param[in] gm6020_4 电机4控制电流
 * @retval 无
 */
void CAN_cmd_gimbal(int16_t yaw1, int16_t pitch2, int16_t gm6020_3,
                    int16_t gm6020_4);

/**
 * @brief 设置拨弹 M2006 电机控制电流（向后兼容）
 * @param[in] shoot1_current 拨弹电机1控制电流
 * @param[in] shoot2_current 拨弹电机2控制电流
 * @param[in] shoot3_current 拨弹电机3控制电流
 * @param[in] rev 保留
 * @retval 无
 */
void CAN_cmd_shoot(int16_t shoot1_current, int16_t shoot2_current,
                   int16_t shoot3_current, int16_t rev);

/**
 * @brief 发送底盘电机 ID 重置命令
 * @retval 无
 */
void CAN_cmd_chassis_reset_ID(void);

/**
 * @brief 立即发送特定 CAN 消息
 * @param[in] msg_id 消息 ID
 * @retval 0 成功, -1 失败
 */
int32_t CAN_SendImmediate(can_msg_id_e msg_id);

/**
 * @brief 获取Yaw云台电机数据指针
 * @return 指向Yaw电机测量数据的指针
 */
extern const motor_measure_t *get_yaw_gimbal_gm6020_measure_point(void);

/**
 * @brief 获取Pitch云台电机数据指针
 * @return 指向Pitch电机测量数据的指针
 */
extern const motor_measure_t *get_pitch_gimbal_gm6020_measure_point(void);

/**
 * @brief 获取拨弹电机数据指针
 * @return 指向拨弹电机测量数据的指针
 */
extern const motor_measure_t *get_trigger_m2006_measure_point(void);

/**
 * @brief 获取底盘电机数据指针
 * @param[in] i 电机编号，范围[0, 3]
 * @return 指向指定电机测量数据的指针
 */
extern const motor_measure_t *get_chassis_m3508_measure_point(uint8_t i);

#ifdef __cplusplus
}
#endif

#endif //__DJI_MOTOR_CAN_H
