//
// Created by pomelo on 2025/12/04
// SBUS_RxCpltCallback() 在 UART 中断上下文中调用
// SBUS_Get_Data() 和 SBUS_Get_Normalized_Channel() 在任务上下文中调用

#include "bsp_sbus.h"
#include "cmsis_os.h"
#include "usart.h" // 包含 huart3 定义
#include <string.h>

// SBUS 接收缓冲区
static uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];

// SBUS 数据
static volatile SBUS_Data_t sbus_data;

// 外部引用的 UART 句柄
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

void SBUS_Init(void) {
  // 清空数据
  memset((void *)&sbus_data, 0, sizeof(sbus_data));
  memset(sbus_rx_buf, 0, sizeof(sbus_rx_buf));

  // 启动 DMA 接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_rx_buf, SBUS_RX_BUF_NUM);
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

static void SBUS_Decode(uint8_t *buf) {
  // 检查帧头 (S.BUS Header 0x0F)
  if (buf[0] != 0x0F) {
    return;
  }

  // 解析 16 个 11bit 通道
  sbus_data.channels[0] = ((buf[1] | buf[2] << 8) & 0x07FF);
  sbus_data.channels[1] = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF);
  sbus_data.channels[2] = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF);
  sbus_data.channels[3] = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF);
  sbus_data.channels[4] = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
  sbus_data.channels[5] = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
  sbus_data.channels[6] = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
  sbus_data.channels[7] = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
  sbus_data.channels[8] = ((buf[12] | buf[13] << 8) & 0x07FF);
  sbus_data.channels[9] = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);
  sbus_data.channels[10] = ((buf[14] >> 6 | buf[15] << 2 | buf[16] << 10) & 0x07FF);
  sbus_data.channels[11] = ((buf[16] >> 1 | buf[17] << 7) & 0x07FF);
  sbus_data.channels[12] = ((buf[17] >> 4 | buf[18] << 4) & 0x07FF);
  sbus_data.channels[13] = ((buf[18] >> 7 | buf[19] << 1 | buf[20] << 9) & 0x07FF);
  sbus_data.channels[14] = ((buf[20] >> 2 | buf[21] << 6) & 0x07FF);
  sbus_data.channels[15] = ((buf[21] >> 5 | buf[22] << 3) & 0x07FF);

  // 解析标志位
  uint8_t flags = buf[23];
  sbus_data.ch17 = (flags & 0x80) ? 1 : 0;
  sbus_data.ch18 = (flags & 0x40) ? 1 : 0;
  sbus_data.frame_lost = (flags & 0x20) ? 1 : 0;
  sbus_data.failsafe = (flags & 0x10) ? 1 : 0;

  // 更新状态
  sbus_data.last_update_tick = HAL_GetTick();
  
  // 多重检测断连状态：
  // 1. 检查 failsafe/frame_lost 标志位
  // 2. 检查摇杆通道值是否在合理范围内
  uint8_t is_disconnected = 0;
  
  if (sbus_data.failsafe || sbus_data.frame_lost) {
    is_disconnected = 1;
  }
  
  // 检查主要摇杆通道（0-3）是否有异常值
  // 正常情况下摇杆范围 321-1663
  // 断连时可能出现 0、1 或其他极端值
  for (uint8_t i = 0; i < 4; i++) {
    uint16_t ch = sbus_data.channels[i];
    // 如果任一通道值超出正常范围，视为断连
    if (ch < 300 || ch > 1700) {
      is_disconnected = 1;
      break;
    }
  }
  
  sbus_data.connect_status = is_disconnected ? 0 : 1;
}

void SBUS_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART3) {
    // 只有接收到完整帧（25字节）才解析
    if (Size == SBUS_FRAME_SIZE) {
      if (sbus_rx_buf[0] == 0x0F) {
        SBUS_Decode(sbus_rx_buf);
      }
    }

    // 重新启动 DMA 接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, sbus_rx_buf, SBUS_RX_BUF_NUM);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  }
}

const SBUS_Data_t *SBUS_Get_Data(void) {
  // 超时检测：超过 100ms 没收到有效帧则判定为断连
  uint32_t current_tick = HAL_GetTick();
  if (current_tick - sbus_data.last_update_tick > SBUS_TIMEOUT_MS) {
    sbus_data.connect_status = 0;
  }
  
  return (const SBUS_Data_t *)&sbus_data;
}

float SBUS_Get_Normalized_Channel(uint8_t ch_idx) {
  if (ch_idx >= SBUS_CHANNEL_NUMBER)
    return 0.0f;

  uint16_t val = sbus_data.channels[ch_idx];

  // 钳位
  if (val < SBUS_CH_MIN)
    val = SBUS_CH_MIN;
  if (val > SBUS_CH_MAX)
    val = SBUS_CH_MAX;

  // 归一化到 [-1, 1]
  return (float)(val - SBUS_CH_MID) / (float)(SBUS_CH_MAX - SBUS_CH_MID);
}

/**
 * @brief UART 错误回调函数
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) {
    // 清除错误标志
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);

    // 重新启动 DMA 接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, sbus_rx_buf, SBUS_RX_BUF_NUM);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  }
}
