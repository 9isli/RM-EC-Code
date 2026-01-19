/**
 ******************************************************************************
 * @file           : bmi088_module.h
 * @brief          : BMI088传感器模块头文件
 * @description    : 为BMI088传感器提供高级接口
 ******************************************************************************
 */

#ifndef __BMI088_MODULE_H
#define __BMI088_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bmi08x.h"

/**
 * @brief BMI088传感器数据结构体
 */
typedef struct {
  struct bmi08x_sensor_data accel; /**< 加速度计数据（原始int16_t） */
  struct bmi08x_sensor_data gyro;  /**< 陀螺仪数据（原始int16_t） */
} BMI088_Data_t;

/**
 * @brief 初始化BMI088传感器模块
 * @return 成功返回0，失败返回非0值
 */
int8_t BMI088_Module_Init(void);

/**
 * @brief 读取BMI088传感器原始数据
 * @param[out] data 指向BMI088_Data_t结构体的指针
 * @return 成功返回0，失败返回非0值f
 */
int8_t BMI088_Module_Read(BMI088_Data_t *data);

/**
 * @brief 仅读取加速度计数据
 * @param[out] accel 指向加速度计数据结构体的指针
 * @return 成功返回0，失败返回非0值
 */
int8_t BMI088_Module_Read_Accel(struct bmi08x_sensor_data *accel);

/**
 * @brief 仅读取陀螺仪数据
 * @param[out] gyro 指向陀螺仪数据结构体的指针
 * @return 成功返回0，失败返回非0值
 */
int8_t BMI088_Module_Read_Gyro(struct bmi08x_sensor_data *gyro);

/**
 * @brief 读取温度数据
 * @param[out] temp_raw 指向温度原始值的指针（int32_t）
 * @return 成功返回0，失败返回非0值
 */
int8_t BMI088_Module_Read_Temp(int32_t *temp_raw);

/**
 * @brief 获取BMI088设备结构体指针
 * @return 指向bmi08x_dev结构体的指针
 */
struct bmi08x_dev *BMI088_Module_Get_Device(void);

#ifdef __cplusplus
}
#endif

#endif /* __BMI088_MODULE_H */
