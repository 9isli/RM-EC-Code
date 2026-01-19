//
// Created by pomelo on 2025/11/29.
//

#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"

/** @brief UART接收数据缓冲区 */
extern uint8_t uart1data[100];
extern uint8_t uart6data[100];

/**
 * @brief 初始化UART1模块
 * @details 配置UART1参数，启动接收中断
 * @retval 无
 */
void UART1_Init(void);

/**
 * @brief 使用DMA发送数据
 * @param[in] pData 指向数据缓冲区的指针
 * @param[in] Size 数据长度
 * @retval 无
 */
void UART1_Send_DMA(uint8_t *pData, uint16_t Size);

/**
 * @brief 使用中断发送数据
 * @param[in] pData 指向数据缓冲区的指针
 * @param[in] Size 数据长度
 * @retval 无
 */
void UART1_Send_IT(uint8_t *pData, uint16_t Size);

/**
 * @brief 接收数据
 * @details 启动UART1接收
 * @retval 无
 */
void UART1_Receive(void);

/**
 * @brief UART1接收完成回调函数
 * @param[in] Size 接收数据长度
 * @retval 无
 */
void UART1_Callback(uint16_t Size);

/**
 * @brief 处理接收到的数据
 * @param[in] buf 数据缓冲区指针
 * @param[in] size 数据长度
 * @retval 无
 */
void UART1_ProcessData(uint8_t *buf, uint16_t size);

/* ================UART6================ */

/** @brief UART6接收数据缓冲区 */
extern uint8_t uart6data[100];

/**
 * @brief 初始化UART6模块
 * @details 配置UART6参数，启动接收中断
 * @retval 无
 */
void UART6_Init(void);

/**
 * @brief 使用DMA发送数据
 * @param[in] pData 指向数据缓冲区的指针
 * @param[in] Size 数据长度
 * @retval 无
 */
void UART6_Send_DMA(uint8_t *pData, uint16_t Size);

/**
 * @brief 使用中断发送数据
 * @param[in] pData 指向数据缓冲区的指针
 * @param[in] Size 数据长度
 * @retval 无
 */
void UART6_Send_IT(uint8_t *pData, uint16_t Size);

/**
 * @brief 接收数据
 * @details 启动UART6接收
 * @retval 无
 */
void UART6_Receive(void);

/**
 * @brief UART6接收完成回调函数
 * @param[in] Size 接收数据长度
 * @retval 无
 */
void UART6_Callback(uint16_t Size);

/**
 * @brief 处理接收到的数据
 * @param[in] buf 数据缓冲区指针
 * @param[in] size 数据长度
 * @retval 无
 */
void UART6_ProcessData(uint8_t *buf, uint16_t size);

#endif //__BSP_UART_H
