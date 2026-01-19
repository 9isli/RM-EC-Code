//
// Created by pomelo on 2025/11/29.
//

#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"

#ifdef __cplusplus
extern "C" {

#endif

extern void USER_CAN_Filter_Init(void);

/**
 * @brief CAN 总线枚举
 */
typedef enum {
  CAN_BUS_1 = 0, // can1
  CAN_BUS_2 = 1, // can2
  CAN_BUS_COUNT = 2
} can_bus_e;

/**
 * @brief CAN 消息 ID 枚举（支持多个总线）
 */
typedef enum {
  // 大疆电机ID设置，因为采用双板设计，M2006与M3508共用一个ID
  CAN_MSG_M3508_ID = 0x200,
  CAN_MSG_GM6020_ID = 0x1FF,
  CAN_MSG_M2006_ID = 0x200,
  // 大疆电机ID 重置命令
  CAN_MSG_RESET_ID = 0x700,

  // 底盘 IMU 数据（底盘板 -> 云台板）
  // 使用两帧传输完整 IMU 数据
  CAN_MSG_CHASSIS_IMU_1 = 0x501, // 帧1: Yaw角度 + Yaw角速度
  CAN_MSG_CHASSIS_IMU_2 = 0x502, // 帧2: Pitch + Roll + 状态标志

  // 云台板 -> 底盘板 的控制命令
  CAN_MSG_GIMBAL_TO_CHASSIS = 0x510, // 底盘目标转速命令

  // 心跳包
  CAN_MSG_HEARTBEAT_GIMBAL = 0x520,  // 云台板心跳
  CAN_MSG_HEARTBEAT_CHASSIS = 0x521, // 底盘板心跳
} can_msg_id_e;

/**
 * @brief 电机在 CAN 消息中的 slot 定义
 * 每个 CAN 消息最多 8 字节，每个电机占用 2 字节
 */
typedef enum {
  MOTOR_SLOT_0 = 0, // 字节 [0:1]
  MOTOR_SLOT_1 = 1, // 字节 [2:3]
  MOTOR_SLOT_2 = 2, // 字节 [4:5]
  MOTOR_SLOT_3 = 3, // 字节 [6:7]
} motor_slot_e;

/**
 * @brief CAN 消息缓冲结构体
 * 每个 CAN ID 对应一个缓冲区
 */
typedef struct {
  uint32_t can_id;           // CAN 消息 ID
  can_bus_e bus;             // 所属总线
  uint8_t data[8];           // 8 字节数据缓冲
  uint8_t slot_count;        // 该消息使用的 slot 数量
  uint32_t last_update_tick; // 最后一次更新的 tick
  uint32_t update_interval;  // 更新间隔（ms），0 表示每次都发送
  uint8_t dirty;             // 是否需要发送标志
} can_msg_buffer_t;

/**
 * @brief 电机控制值结构体
 */
typedef struct {
  can_msg_id_e msg_id; // 所属 CAN 消息 ID
  motor_slot_e slot;   // 在消息中的 slot 位置
  int16_t value;       // 控制值（电流或转速）
} motor_control_t;

/**
 * @brief 初始化 CAN 消息管理器
 * @param[in] 无
 * @retval 无
 */
void CAN_Manager_Init(void);

/**
 * @brief 设置电机控制值
 * @param[in] motor 电机控制信息
 * @retval 无
 * @note 该函数只是更新缓冲，不立即发送。发送由 CAN_Manager_SendAll 处理
 */
void CAN_Manager_SetMotor(const motor_control_t *motor);

/**
 * @brief 批量设置电机控制值
 * @param[in] motors 电机数组
 * @param[in] count 电机数量
 * @retval 无
 */
void CAN_Manager_SetMotors(const motor_control_t *motors, uint8_t count);

/**
 * @brief 发送所有需要更新的 CAN 消息
 * @param[in] 无
 * @retval 无
 * @note 应该在固定周期（如 1kHz）中调用
 */
void CAN_Manager_SendAll(void);

/**
 * @brief 立即发送特定 CAN 消息（跳过频率限制）
 * @param[in] msg_id 消息 ID
 * @retval 0 成功, -1 失败
 */
int32_t CAN_Manager_SendImmediate(can_msg_id_e msg_id);

/**
 * @brief 清空特定 CAN 消息缓冲
 * @param[in] msg_id 消息 ID
 * @retval 无
 */
void CAN_Manager_ClearBuffer(can_msg_id_e msg_id);

/**
 * @brief 获取 CAN 消息缓冲指针（用于调试）
 * @param[in] msg_id 消息 ID
 * @retval 缓冲指针，NULL 表示不存在
 */
const can_msg_buffer_t *CAN_Manager_GetBuffer(can_msg_id_e msg_id);

#ifdef __cplusplus
}
#endif

#endif //__BSP_CAN_H
