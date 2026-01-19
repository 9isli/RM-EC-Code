#include <stdint.h>

/**
 * @brief IST8310原始数据结构体
 */
typedef struct ist8310_raw_data_t {
  float x; /**< X轴磁场强度 */
  float y; /**< Y轴磁场强度 */
  float z; /**< Z轴磁场强度 */
} ist8310_raw_data_t;

/**
 * @brief IST8310错误代码枚举
 */
typedef enum ist8310_error_e {
  IST8310_NO_ERROR = 0x00, /**< 无错误 */
  MEG_ID_ERROR = 0x01,     /**< 芯片ID错误 */
} ist8310_error_e;

/**
 * @brief IST8310数据结构体
 */
typedef struct ist8310_data_t {
  uint8_t chip_id;             /**< 芯片ID */
  ist8310_raw_data_t meg_data; /**< 磁场数据 */
  ist8310_error_e meg_error;   /**< 错误状态 */
} ist8310_data_t;

/** @brief 整形向uT转换系数 */
#define MAG_SEN 0.3f

/** @brief IST8310 I2C地址 */
#define IST8310_I2C_ADDR 0x0E

/** @brief IST8310 I2C接口 */
#define IST8310_I2C hi2c3

/** @brief IST8310 GPIO端口 */
#define IST8310_GPIOx GPIOG

/** @brief IST8310 GPIO引脚 */
#define IST8310_GPIOp GPIO_PIN_6

/**
 * @brief 初始化IST8310磁力计
 * @param[out] ist8310_data 指向IST8310数据结构体的指针
 * @retval 无
 */
void IST8310_Init(ist8310_data_t *ist8310_data);

/**
 * @brief 从IST8310读取单个字节数据
 * @param[in] addr 寄存器地址
 * @return 读取的数据
 */
uint8_t ReadSingleDataFromIST8310(uint8_t addr);

/**
 * @brief 向IST8310写入单个字节数据
 * @param[in] addr 寄存器地址
 * @param[in] data 要写入的数据
 * @retval 无
 */
void WriteSingleDataFromIST8310(uint8_t addr, uint8_t data);

/**
 * @brief 从IST8310读取多个字节数据
 * @param[in] addr 起始寄存器地址
 * @param[out] data 数据缓冲区指针
 * @param[in] len 读取长度
 * @retval 无
 */
void ReadMultiDataFromIST8310(uint8_t addr, uint8_t *data, uint8_t len);

/**
 * @brief 向IST8310写入多个字节数据
 * @param[in] addr 起始寄存器地址
 * @param[in] data 数据缓冲区指针
 * @param[in] len 写入长度
 * @retval 无
 */
void WriteMultiDataFromIST8310(uint8_t addr, uint8_t *data, uint8_t len);

/**
 * @brief 读取IST8310磁场数据
 * @param[out] meg_data 指向磁场数据结构体的指针
 * @retval 无
 */
void ReadIST8310Data(ist8310_raw_data_t *meg_data);

/**
 * @brief 验证IST8310芯片ID
 * @param[out] id 芯片ID指针
 * @return 错误代码
 */
ist8310_error_e VerifyMegId(uint8_t *id);