/**
 * @file ist8310driver.c
 * @brief IST8310磁力计驱动实现
 * @details 提供IST8310磁力计的初始化和数据读取功能
 */

#include "ist8310driver.h"

#include <string.h>

#include "i2c.h"
#include "ist8310reg.h"

/**
 * @brief 初始化IST8310磁力计
 * @details 复位磁力计，配置工作模式和采样频率
 * @param[out] ist8310_data 指向IST8310数据结构体的指针
 * @retval 无
 */
void IST8310_Init(ist8310_data_t *ist8310_data) {
  memset(ist8310_data, 0, sizeof(ist8310_data_t));

  ist8310_data->meg_error = IST8310_NO_ERROR;

  // 把磁力计重启
  HAL_GPIO_WritePin(IST8310_GPIOx, IST8310_GPIOp, GPIO_PIN_RESET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(IST8310_GPIOx, IST8310_GPIOp, GPIO_PIN_SET);
  HAL_Delay(50);

  // 基础配置
  // 不使能中断，直接读取
  WriteSingleDataFromIST8310(IST8310_CNTL2_ADDR, IST8310_STAT2_NONE_ALL);
  // 平均采样四次
  WriteSingleDataFromIST8310(IST8310_AVGCNTL_ADDR, IST8310_AVGCNTL_FOURTH);
  // 200Hz的输出频率
  WriteSingleDataFromIST8310(IST8310_CNTL1_ADDR, IST8310_CNTL1_CONTINUE);

  ist8310_data->meg_error |= VerifyMegId(&ist8310_data->chip_id);
}

/**
 * @brief 从IST8310读取单个字节数据
 * @param[in] addr 寄存器地址
 * @return 读取的数据
 */
uint8_t ReadSingleDataFromIST8310(uint8_t addr) {
  uint8_t data;
  HAL_I2C_Mem_Read(&IST8310_I2C, (IST8310_I2C_ADDR << 1), addr,
                   I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
  return data;
}

/**
 * @brief 向IST8310写入单个字节数据
 * @param[in] addr 寄存器地址
 * @param[in] data 要写入的数据
 * @retval 无
 */
void WriteSingleDataFromIST8310(uint8_t addr, uint8_t data) {
  HAL_I2C_Mem_Write(&IST8310_I2C, (IST8310_I2C_ADDR << 1), addr,
                    I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
}

/**
 * @brief 从IST8310读取多个字节数据
 * @param[in] addr 起始寄存器地址
 * @param[out] data 数据缓冲区指针
 * @param[in] len 读取长度
 * @retval 无
 */
void ReadMultiDataFromIST8310(uint8_t addr, uint8_t *data, uint8_t len) {
  HAL_I2C_Mem_Read(&IST8310_I2C, (IST8310_I2C_ADDR << 1), addr,
                   I2C_MEMADD_SIZE_8BIT, data, len, 10);
}

/**
 * @brief 向IST8310写入多个字节数据
 * @param[in] addr 起始寄存器地址
 * @param[in] data 数据缓冲区指针
 * @param[in] len 写入长度
 * @retval 无
 */
void WriteMultiDataFromIST8310(uint8_t addr, uint8_t *data, uint8_t len) {
  HAL_I2C_Mem_Write(&IST8310_I2C, (IST8310_I2C_ADDR << 1), addr,
                    I2C_MEMADD_SIZE_8BIT, data, len, 10);
}

/**
 * @brief 读取IST8310磁场数据
 * @details 读取X、Y、Z三轴磁场数据并进行单位转换
 * @param[out] meg_data 指向磁场数据结构体的指针
 * @retval 无
 */
void ReadIST8310Data(ist8310_raw_data_t *meg_data) {
  uint8_t buf[6];
  int16_t temp_ist8310_data = 0;
  ReadMultiDataFromIST8310(IST8310_DATA_XL_ADDR, buf, 6);
  temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
  meg_data->x = MAG_SEN * temp_ist8310_data;
  temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
  meg_data->y = MAG_SEN * temp_ist8310_data;
  temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
  meg_data->z = MAG_SEN * temp_ist8310_data;
}

/**
 * @brief 验证IST8310芯片ID
 * @details 读取芯片ID并与预期值比较
 * @param[out] id 芯片ID指针
 * @return 错误代码（IST8310_NO_ERROR或MEG_ID_ERROR）
 */
ist8310_error_e VerifyMegId(uint8_t *id) {
  *id = ReadSingleDataFromIST8310(IST8310_CHIP_ID_ADDR);
  if (*id != IST8310_CHIP_ID_VAL) {
    return MEG_ID_ERROR;
  } else {
    return IST8310_NO_ERROR;
  }
}