#ifndef __BSP_SBUS_H
#define __BSP_SBUS_H

#include "main.h"

// S.BUS 协议常量
#define SBUS_FRAME_SIZE 25
#define SBUS_CHANNEL_NUMBER 16

// S.BUS 通道范围 (根据实测调整)
#define SBUS_CH_MIN 321
#define SBUS_CH_MAX 1663
#define SBUS_CH_MID 992

// 接收缓冲区大小 (2帧大小，用于DMA循环接收)
#define SBUS_RX_BUF_NUM 50

// 超时判定时间 (ms) - SBUS 正常帧率约 7ms/帧，100ms 无数据判定断连
#define SBUS_TIMEOUT_MS 100

/**
 * @brief S.BUS 解析后的数据结构
 */
typedef struct {
  uint16_t
      channels[SBUS_CHANNEL_NUMBER]; // 16个比例通道 (11bit, range 172-1811)
  uint8_t ch17;                      // 数字通道 17 (0 or 1)
  uint8_t ch18;                      // 数字通道 18 (0 or 1)
  uint8_t frame_lost;                // 丢帧标志
  uint8_t failsafe;                  // 失控保护标志
  uint32_t last_update_tick;         // 最后一次更新时间戳
  uint8_t connect_status;            // 连接状态 (1:已连接, 0:未连接/超时)
} SBUS_Data_t;

/**
 * @brief 初始化 S.BUS 接收
 */
void SBUS_Init(void);

/**
 * @brief 解析 S.BUS 数据
 * @param[in] raw_data 原始 25 字节数据
 */
void SBUS_Parse(uint8_t *raw_data);

/**
 * @brief 获取 S.BUS 数据指针
 */
const SBUS_Data_t *SBUS_Get_Data(void);

/**
 * @brief 获取指定的归一化通道值
 * @param ch_idx 通道索引 (0-15)
 * @return float 归一化值 [-1.0, 1.0]
 */
float SBUS_Get_Normalized_Channel(uint8_t ch_idx);

/**
 * @brief S.BUS 接收中断回调处理 (放在 UART IDLE 中断中调用)
 * @param huart UART 句柄
 * @param Size 接收到的字节数
 */
void SBUS_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif // __BSP_SBUS_H
