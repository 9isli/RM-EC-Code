//
// Created by pomelo on 2025/11/29.
//

#include "bsp_can.h"
#include "chassis_imu.h"
#include "cmsis_os.h"
#include "dji_motor.h"
#include "dual_board_comm.h"
#include "main.h"
#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
 * @brief   滤波器配置与初始化
 * @param   无
 * @retval  无
 */
void USER_CAN_Filter_Init(void) {
  CAN_FilterTypeDef can_filter_st = {0};

  // ========== 配置 CAN1 过滤器 ==========
  can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = 0x0000;
  can_filter_st.FilterIdLow = 0x0000;
  can_filter_st.FilterMaskIdHigh = 0x0000;
  can_filter_st.FilterMaskIdLow = 0x0000;

  can_filter_st.FilterBank = 0;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

  // 关键：配置 CAN1 时必须指定 CAN2 的起始过滤器组
  can_filter_st.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // ========== 配置 CAN2 过滤器 ==========
  // 复用上面的配置，只修改 FilterBank
  can_filter_st.FilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/*==================CAN消息管理==================*/

/**
 * @brief 支持的 CAN 消息配置
 */
static const can_msg_buffer_t msg_config[] = {
    // 底盘消息M3508
    {
        .can_id = 0x200,
        .bus = CAN_BUS_1,
        .slot_count = 4,      // 4 个 M3508 电机
        .update_interval = 0, // 每次都发送
    },
    // 云台消息GM6020
    {
        .can_id = 0x1FF,
        .bus = CAN_BUS_1,
        .slot_count = 4,      // 最多 4 个 GM6020 电机
        .update_interval = 0, // 每次都发送
    },
    // 发射消息M2006
    {
        .can_id = 0x200,
        .bus = CAN_BUS_1,
        .slot_count = 3, // M2006 拨弹电机等
        .update_interval = 0,
    },
};

#define MSG_CONFIG_COUNT (sizeof(msg_config) / sizeof(msg_config[0]))

/**
 * @brief CAN 消息缓冲管理实例
 */
typedef struct {
  can_msg_buffer_t buffers[MSG_CONFIG_COUNT];
  uint8_t buffer_count;
} can_manager_t;

static can_manager_t g_can_manager;

/**
 * @brief 根据 CAN ID 查找缓冲区索引
 */
static int32_t CAN_Manager_FindBuffer(can_msg_id_e msg_id) {
  for (uint8_t i = 0; i < g_can_manager.buffer_count; i++) {
    if (g_can_manager.buffers[i].can_id == msg_id) {
      return i;
    }
  }
  return -1;
}

/**
 * @brief 初始化 CAN 消息管理器
 */
void CAN_Manager_Init(void) {
  // 复制配置到运行时缓冲
  g_can_manager.buffer_count = MSG_CONFIG_COUNT;
  for (uint8_t i = 0; i < MSG_CONFIG_COUNT; i++) {
    memcpy(&g_can_manager.buffers[i], &msg_config[i], sizeof(can_msg_buffer_t));
    memset(g_can_manager.buffers[i].data, 0, 8);
    g_can_manager.buffers[i].dirty = 0;
    g_can_manager.buffers[i].last_update_tick = 0;
  }
}

/**
 * @brief 设置电机控制值
 */
void CAN_Manager_SetMotor(const motor_control_t *motor) {
  if (motor == NULL)
    return;

  int32_t buf_idx = CAN_Manager_FindBuffer(motor->msg_id);
  if (buf_idx < 0)
    return;

  can_msg_buffer_t *buffer = &g_can_manager.buffers[buf_idx];

  // 计算字节位置 (每个 slot 占用 2 字节)
  uint8_t byte_offset = motor->slot * 2;
  if (byte_offset + 1 >= 8)
    return; // 越界检查

  // 写入 16 位数据 (大端序)
  buffer->data[byte_offset] = (motor->value >> 8) & 0xFF;
  buffer->data[byte_offset + 1] = motor->value & 0xFF;

  buffer->dirty = 1;
}

/**
 * @brief 批量设置电机控制值
 */
void CAN_Manager_SetMotors(const motor_control_t *motors, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    CAN_Manager_SetMotor(&motors[i]);
  }
}

/**
 * @brief 发送单个 CAN 消息
 */
static int32_t CAN_Manager_SendMessage(can_msg_buffer_t *buffer) {
  if (buffer == NULL)
    return -1;

  CAN_HandleTypeDef *hcan = (buffer->bus == CAN_BUS_1) ? &hcan1 : &hcan2;

  CAN_TxHeaderTypeDef tx_header = {.StdId = buffer->can_id,
                                   .IDE = CAN_ID_STD,
                                   .RTR = CAN_RTR_DATA,
                                   .DLC = 8};

  uint32_t send_mail_box;
  if (HAL_CAN_AddTxMessage(hcan, &tx_header, buffer->data, &send_mail_box) !=
      HAL_OK) {
    return -1;
  }

  buffer->dirty = 0;
  buffer->last_update_tick = osKernelGetTickCount();

  return 0;
}

/**
 * @brief 发送所有需要更新的 CAN 消息
 */
void CAN_Manager_SendAll(void) {
  uint32_t current_tick = osKernelGetTickCount();

  for (uint8_t i = 0; i < g_can_manager.buffer_count; i++) {
    can_msg_buffer_t *buffer = &g_can_manager.buffers[i];

    if (!buffer->dirty)
      continue;

    // 检查更新频率限制
    if (buffer->update_interval > 0) {
      if ((current_tick - buffer->last_update_tick) < buffer->update_interval) {
        continue;
      }
    }

    CAN_Manager_SendMessage(buffer);
  }
}

/**
 * @brief 立即发送特定 CAN 消息
 */
int32_t CAN_Manager_SendImmediate(can_msg_id_e msg_id) {
  int32_t buf_idx = CAN_Manager_FindBuffer(msg_id);
  if (buf_idx < 0)
    return -1;

  return CAN_Manager_SendMessage(&g_can_manager.buffers[buf_idx]);
}

/**
 * @brief 清空特定 CAN 消息缓冲
 */
void CAN_Manager_ClearBuffer(can_msg_id_e msg_id) {
  int32_t buf_idx = CAN_Manager_FindBuffer(msg_id);
  if (buf_idx < 0)
    return;

  memset(g_can_manager.buffers[buf_idx].data, 0, 8);
}

/**
 * @brief 获取 CAN 消息缓冲指针
 */
const can_msg_buffer_t *CAN_Manager_GetBuffer(can_msg_id_e msg_id) {
  int32_t buf_idx = CAN_Manager_FindBuffer(msg_id);
  if (buf_idx < 0)
    return NULL;

  return &g_can_manager.buffers[buf_idx];
}

/**
 * @brief HAL库CAN接收回调函数
 * @details 在CAN接收到数据时被调用，解析电机反馈数据和底盘IMU数据
 * @param[in] hcan CAN句柄指针
 * @retval 无
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  // ========== CAN1: 电机反馈 ==========
  if (hcan == &hcan1) {
    switch (rx_header.StdId) {
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    case CAN_YAW_MOTOR_ID:
    case CAN_PIT_MOTOR_ID:
    case CAN_TRIGGER_MOTOR_ID: {
      uint8_t i = rx_header.StdId - CAN_3508_M1_ID;
      get_motor_measure(&dji_motor[i], rx_data);
      break;
    }
    default:
      break;
    }
  }
  // ========== CAN2: 双板通信 ==========
  else if (hcan == &hcan2) {
    switch (rx_header.StdId) {
    // 底盘 IMU 数据（从底盘主控板接收）
    case CAN_MSG_CHASSIS_IMU_1:
    case CAN_MSG_CHASSIS_IMU_2:
      Chassis_IMU_CAN_Callback(rx_header.StdId, rx_data);
      break;

    // 双板通信（心跳、控制命令）
    case CAN_MSG_HEARTBEAT_GIMBAL:
    case CAN_MSG_HEARTBEAT_CHASSIS:
    case CAN_MSG_GIMBAL_TO_CHASSIS:
      DualBoard_CAN_Callback(rx_header.StdId, rx_data);
      break;

    default:
      break;
    }
  }
}
