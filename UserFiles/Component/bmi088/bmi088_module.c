/**
 ******************************************************************************
 * @file           : bmi088_module.c
 * @brief          : BMI088传感器模块实现
 * @description    : 为BMI088传感器提供高级接口
 ******************************************************************************
 */

#include "bmi088_module.h"
#include "drv_bmi088.h"

/**
 * @brief BMI088传感器设备结构体（静态，模块私有）
 */
static struct bmi08x_dev bmi088_device;

/**
 * @brief 初始化BMI088传感器模块
 * @details 初始化SPI接口、加速度计和陀螺仪，设置测量配置
 * @return 成功返回0，失败返回非0值
 */
int8_t BMI088_Module_Init(void) {
  int8_t rslt = BMI08X_OK;

  // Initialize BMI088 interface (SPI)
  rslt = bmi08x_interface_init(&bmi088_device, BMI08X_SPI_INTF, BMI088_VARIANT);
  if (rslt != BMI08X_OK) {
    return rslt;
  }

  // Initialize accelerometer
  rslt = bmi08a_init(&bmi088_device);
  if (rslt != BMI08X_OK) {
    return rslt;
  }

  // Set accelerometer power mode to ACTIVE
  bmi088_device.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
  rslt = bmi08a_set_power_mode(&bmi088_device);
  if (rslt != BMI08X_OK) {
    return rslt;
  }

  // Set accelerometer measurement configuration
  bmi088_device.accel_cfg.odr = BMI08X_ACCEL_ODR_800_HZ;
  bmi088_device.accel_cfg.range = BMI088_ACCEL_RANGE_6G;
  bmi088_device.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
  rslt = bmi08a_set_meas_conf(&bmi088_device);
  if (rslt != BMI08X_OK) {
    return rslt;
  }

  // Initialize gyroscope
  rslt = bmi08g_init(&bmi088_device);
  if (rslt != BMI08X_OK) {
    return rslt;
  }

  // Set gyroscope power mode to NORMAL
  bmi088_device.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
  rslt = bmi08g_set_power_mode(&bmi088_device);
  if (rslt != BMI08X_OK) {
    return rslt;
  }

  // Set gyroscope measurement configuration
  bmi088_device.gyro_cfg.odr = BMI08X_GYRO_BW_230_ODR_2000_HZ;
  bmi088_device.gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;
  bmi088_device.gyro_cfg.bw = BMI08X_GYRO_BW_230_ODR_2000_HZ;
  rslt = bmi08g_set_meas_conf(&bmi088_device);
  if (rslt != BMI08X_OK) {
    return rslt;
  }

  return BMI08X_OK;
}

/**
 * @brief 读取BMI088传感器原始数据（加速度计和陀螺仪）
 * @param[out] data 指向BMI088_Data_t结构体的指针
 * @return 成功返回0，失败返回非0值
 */
int8_t BMI088_Module_Read(BMI088_Data_t *data) {
  int8_t rslt = BMI08X_OK;

  if (data == NULL) {
    return BMI08X_E_NULL_PTR;
  }

  // Read accelerometer data (raw int16_t values)
  rslt = bmi08a_get_data(&data->accel, &bmi088_device);
  if (rslt != BMI08X_OK) {
    return rslt;
  }

  // Read gyroscope data (raw int16_t values)
  rslt = bmi08g_get_data(&data->gyro, &bmi088_device);
  if (rslt != BMI08X_OK) {
    return rslt;
  }

  return BMI08X_OK;
}

/**
 * @brief 仅读取加速度计数据
 * @param[out] accel 指向加速度计数据结构体的指针
 * @return 成功返回0，失败返回非0值
 */
int8_t BMI088_Module_Read_Accel(struct bmi08x_sensor_data *accel) {
  if (accel == NULL) {
    return BMI08X_E_NULL_PTR;
  }

  return bmi08a_get_data(accel, &bmi088_device);
}

/**
 * @brief 仅读取陀螺仪数据
 * @param[out] gyro 指向陀螺仪数据结构体的指针
 * @return 成功返回0，失败返回非0值
 */
int8_t BMI088_Module_Read_Gyro(struct bmi08x_sensor_data *gyro) {
  if (gyro == NULL) {
    return BMI08X_E_NULL_PTR;
  }

  return bmi08g_get_data(gyro, &bmi088_device);
}

/**
 * @brief 读取温度数据
 * @param[out] temp_raw 指向温度原始值的指针（int32_t）
 * @return 成功返回0，失败返回非0值
 */
int8_t BMI088_Module_Read_Temp(int32_t *temp_raw) {
  if (temp_raw == NULL) {
    return BMI08X_E_NULL_PTR;
  }

  return bmi08a_get_sensor_temperature(&bmi088_device, temp_raw);
}

/**
 * @brief 获取BMI088设备结构体指针
 * @return 指向bmi08x_dev结构体的指针
 */
struct bmi08x_dev *BMI088_Module_Get_Device(void) { return &bmi088_device; }
