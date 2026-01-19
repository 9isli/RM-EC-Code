//
// Created by pomelo on 2025/11/29.
//

#include "bsp_uart.h"
#include "bsp_sbus.h"
#include "led.h"
#include "task_user.h"
#include <string.h>

/** @brief 接收数据缓冲区 */
static uint8_t receivedData_1[100];

/** @brief 接收数据长度 */
static uint16_t dataLength_1 = 0;

/** @brief UART1标志位 */
uint8_t usart1_flag = 0;

/** @brief UART1模式 */
uint8_t usart1_mode = 0;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

uint8_t uart1data[100];

/* ================UART6================ */

/** @brief 接收数据缓冲区 */
static uint8_t receivedData_6[100];

/** @brief 接收数据长度 */
static uint16_t dataLength_6 = 0;

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

uint8_t uart6data[100];

/* ================ UART_API ================ */
/* 下面都是UART1/6的API，大部分底层内容都在Core/src/usart.c中 */

/**
 * @brief 初始化UART1模块
 * @details 配置UART1参数，启动接收中断
 * @retval 无
 */
void UART1_Init(void) {
  // 关闭过半回调
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  // 启动接收
  UART1_Receive();
}

/**
 * @brief 使用DMA发送数据
 * @details 通过DMA方式发送数据，适合大数据量传输
 * @param[in] pData 指向数据缓冲区的指针
 * @param[in] Size 数据长度
 * @retval 无
 */
void UART1_Send_DMA(uint8_t *pData, uint16_t Size) {
  HAL_UART_Transmit_DMA(&huart1, pData, Size);
}

/**
 * @brief 使用中断发送数据
 * @details 通过中断方式发送数据，适合简单的数据传输
 * @param[in] pData 指向数据缓冲区的指针
 * @param[in] Size 数据长度
 * @retval 无
 */
void UART1_Send_IT(uint8_t *pData, uint16_t Size) {
  HAL_UART_Transmit_IT(&huart1, pData, Size);
}

/**
 * @brief 接收数据
 * @details 启动UART1接收，使用DMA和空闲中断
 * @retval 无
 */
void UART1_Receive(void) {
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receivedData_1, sizeof(receivedData_1));
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

/**
 * @brief UART1接收完成回调函数
 * @details 在接收完成时被调用，处理接收到的数据
 * @param[in] Size 接收数据长度
 * @retval 无
 */
void UART1_Callback(uint16_t Size) {
  // 优先接收数据
  memcpy(uart1data, receivedData_6, Size);
  dataLength_1 = Size;

  // 调用用户数据处理函数
  UART1_ProcessData(uart1data, Size);

  // 继续接收
  UART1_Receive();
}

/**
 * @brief 处理接收到的数据
 * @details 解析PID参数命令并更新全局变量
 * @param[in] buf 数据缓冲区指针
 * @param[in] size 数据长度
 * @retval 无
 */
void UART1_ProcessData(uint8_t *buf, uint16_t size) {
  // 旧的键盘遥控功能已废弃，现已使用 S.BUS 遥控器
  // 保留此函数用于调试输出或其他用途
  (void)buf;
  (void)size;

  // 现在调参已经使用调试软件修改局内变量，已废弃，仅供参考
  // 创建临时缓冲区来处理命令
  // char temp_buf[32] = {0};

  // 查找命令结束符 '!'
  /*uint8_t *exclamation_pos = (uint8_t *)memchr(buf, '!', size);
  if (exclamation_pos != NULL) {
    // 计算命令长度（包含!）
    uint16_t cmd_length = exclamation_pos - buf + 1;

    // 将命令复制到临时缓冲区
    if (cmd_length < sizeof(temp_buf)) {
      memcpy(temp_buf, buf, cmd_length);
      temp_buf[cmd_length] = '\0';

      // 检查是否为PID参数设置命令
      if (cmd_length >= 4 && temp_buf[1] == '=') {
        char param_char = temp_buf[0];
        char *number_str = temp_buf + 2; // 跳过"X="

        // 找到!的位置并替换为NULL
        char *excl_in_temp = strchr(number_str, '!');
        if (excl_in_temp)
          *excl_in_temp = '\0';

        // 解析数字
        char *endptr;
        float value = strtof(number_str, &endptr);

        if (endptr != number_str) {
          // 直接修改全局PID变量
          switch (param_char) {
          case 'P':
          case 'p':
            angle_kp = value; // 直接修改全局变量
            break;

          case 'I':
          case 'i':
            angle_ki = value; // 直接修改全局变量
            break;

          case 'D':
          case 'd':
            angle_kd = value; // 直接修改全局变量
            break;

          case 'F':
          case 'f':
            angle_kf = value; // 直接修改全局变量
            break;

          default: {
            return;
          }
          }
        }
      }
    }
  }*/

  // 如果不是PID命令，保持原来的回显逻辑
  UART1_Send_DMA(buf, size);
}

/* ================UART6================ */

/**
 * @brief 初始化UART6模块
 * @details 配置UART6参数，启动接收中断
 * @retval 无
 */
void UART6_Init(void) {
  // 关闭过半回调
  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);

  // 启动接收
  UART6_Receive();
}

/**
 * @brief 使用DMA发送数据
 * @details 通过DMA方式发送数据，适合大数据量传输
 * @param[in] pData 指向数据缓冲区的指针
 * @param[in] Size 数据长度
 * @retval 无
 */
void UART6_Send_DMA(uint8_t *pData, uint16_t Size) {
  HAL_UART_Transmit_DMA(&huart6, pData, Size);
}

/**
 * @brief 使用中断发送数据
 * @details 通过中断方式发送数据，适合简单的数据传输
 * @param[in] pData 指向数据缓冲区的指针
 * @param[in] Size 数据长度
 * @retval 无
 */
void UART6_Send_IT(uint8_t *pData, uint16_t Size) {
  HAL_UART_Transmit_IT(&huart6, pData, Size);
}

/**
 * @brief 接收数据
 * @details 启动UART6接收，使用DMA和空闲中断
 * @retval 无
 */
void UART6_Receive(void) {
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, receivedData_6, sizeof(receivedData_6));
  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
}

/**
 * @brief UART6接收完成回调函数
 * @details 在接收完成时被调用，处理接收到的数据
 * @param[in] Size 接收数据长度
 * @retval 无
 */
void UART6_Callback(uint16_t Size) {
  // 优先接收数据
  memcpy(uart6data, receivedData_6, Size);
  dataLength_6 = Size;

  // 调用用户数据处理函数
  UART6_ProcessData(uart6data, Size);

  // 继续接收
  UART6_Receive();
}

/**
 * @brief 处理接收到的数据
 * @details 用户自定义数据处理函数
 * @param[in] buf 数据缓冲区指针
 * @param[in] size 数据长度
 * @retval 无
 */
void UART6_ProcessData(uint8_t *buf, uint16_t size) {
  // 数据处理部分，当需要大量数据处理时，此处禁止使用阻塞

  // 回显数据,需要时可自行打开
  // UART6_Send_IT(buf, size);
}

/* ================回调分发================ */

/**
 * @brief UART接收事件回调函数
 * @details 分发UART接收事件到对应的UART模块处理
 * @param[in] huart UART句柄指针
 * @param[in] Size 接收数据长度
 * @retval 无
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  // 回调的分发
  if (huart->Instance == USART1) {
    UART1_Callback(Size);
  } else if (huart->Instance == USART6) {
    // UART6_Callback(Size);
    // SBUS_RxCpltCallback(huart); // 暂时征用 UART6 做 S.BUS 模拟
  } else if (huart->Instance == USART3) {
    SBUS_RxCpltCallback(huart, Size);
  }
}
