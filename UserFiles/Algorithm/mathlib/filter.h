//
// Created by pomelo on 2025/11/30.
//

#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== 滤波器类型枚举 ========================== */

typedef enum {
  FILTER_TYPE_LPF1,         // 一阶低通滤波
  FILTER_TYPE_ADAPTIVE_LPF, // 自适应低通滤波
  FILTER_TYPE_MAF,          // 滑动平均滤波
  FILTER_TYPE_MEDIAN,       // 中位值滤波
  FILTER_TYPE_MEDIAN_AVG,   // 中位值平均滤波
  FILTER_TYPE_KALMAN1D,     // 一维卡尔曼滤波
  FILTER_TYPE_LIMIT,        // 限幅滤波
} FilterType_e;

/* ========================== 一阶低通滤波器 ========================== */

/**
 * @brief 一阶低通滤波器 (First Order Low Pass Filter)
 * @note  公式: y[n] = α * x[n] + (1-α) * y[n-1]
 *        α = dt / (RC + dt)，其中 RC = 1/(2π*fc)
 */
typedef struct {
  float output; // 滤波输出
  float alpha;  // 滤波系数 (0-1)，越小越平滑
} LPF1_t;

void LPF1_Init(LPF1_t *filter, float alpha);
void LPF1_Init_RC(LPF1_t *filter, float rc, float dt);
void LPF1_Init_Cutoff(LPF1_t *filter, float cutoff_hz, float dt);
float LPF1_Calculate(LPF1_t *filter, float input);

/* ========================== 自适应低通滤波器 ========================== */

/**
 * @brief 自适应低通滤波器 (Adaptive Low Pass Filter)
 * @note  滤波系数根据输入变化率动态调整
 *        变化快时响应快，变化慢时滤波强
 */
typedef struct {
  float output;      // 滤波输出
  float min_alpha;   // 最小滤波系数（静止时）
  float max_alpha;   // 最大滤波系数（快速变化时）
  float sensitivity; // 灵敏度（越大越不敏感）
} AdaptiveLPF_t;

void AdaptiveLPF_Init(AdaptiveLPF_t *filter, float min_alpha, float max_alpha,
                      float sensitivity);
float AdaptiveLPF_Calculate(AdaptiveLPF_t *filter, float input);

/* ========================== 滑动平均滤波器 ========================== */

/**
 * @brief 滑动平均滤波器 (Moving Average Filter)
 * @note  对最近 N 个采样值取平均
 */
#define MAF_MAX_SIZE 32 // 最大窗口大小

typedef struct {
  float buffer[MAF_MAX_SIZE]; // 环形缓冲区
  float sum;                  // 当前和（用于快速计算）
  uint8_t size;               // 窗口大小
  uint8_t index;              // 当前写入位置
  uint8_t count;              // 已填充数量
} MAF_t;

void MAF_Init(MAF_t *filter, uint8_t size);
float MAF_Calculate(MAF_t *filter, float input);

/* ========================== 中位值滤波器 ========================== */

/**
 * @brief 中位值滤波器 (Median Filter)
 * @note  取最近 N 个采样值的中位数，有效滤除脉冲噪声
 */
#define MEDIAN_MAX_SIZE 9 // 最大窗口大小（建议奇数）

typedef struct {
  float buffer[MEDIAN_MAX_SIZE]; // 环形缓冲区
  uint8_t size;                  // 窗口大小
  uint8_t index;                 // 当前写入位置
  uint8_t count;                 // 已填充数量
} MedianFilter_t;

void MedianFilter_Init(MedianFilter_t *filter, uint8_t size);
float MedianFilter_Calculate(MedianFilter_t *filter, float input);

/* ========================== 中位值平均滤波器 ========================== */

/**
 * @brief 中位值平均滤波器 (Median Average Filter)
 * @note  去掉最大最小值后取平均，结合中位值和平均滤波的优点
 */
#define MEDIAN_AVG_MAX_SIZE 16

typedef struct {
  float buffer[MEDIAN_AVG_MAX_SIZE];
  uint8_t size;
  uint8_t index;
  uint8_t count;
} MedianAvgFilter_t;

void MedianAvgFilter_Init(MedianAvgFilter_t *filter, uint8_t size);
float MedianAvgFilter_Calculate(MedianAvgFilter_t *filter, float input);

/* ========================== 一维卡尔曼滤波器 ========================== */

/**
 * @brief 一维卡尔曼滤波器 (1D Kalman Filter)
 * @note  适用于单变量状态估计
 */
typedef struct {
  float x; // 状态估计值
  float p; // 估计误差协方差
  float q; // 过程噪声协方差（越大越信任测量）
  float r; // 测量噪声协方差（越大越信任预测）
  float k; // 卡尔曼增益
} Kalman1D_t;

void Kalman1D_Init(Kalman1D_t *filter, float q, float r, float init_x,
                   float init_p);
float Kalman1D_Calculate(Kalman1D_t *filter, float measurement);

/* ========================== 限幅滤波器 ========================== */

/**
 * @brief 限幅滤波器 (Amplitude Limiting Filter)
 * @note  如果新值与上次值差距超过阈值，则认为是噪声，保持上次值
 */
typedef struct {
  float output;        // 滤波输出
  float max_delta;     // 最大允许变化量
  uint8_t initialized; // 是否已初始化
} LimitFilter_t;

void LimitFilter_Init(LimitFilter_t *filter, float max_delta);
float LimitFilter_Calculate(LimitFilter_t *filter, float input);

/* ========================== 通用滤波器接口 ========================== */

/**
 * @brief 通用滤波器结构体
 * @note  统一接口，可使用 Filter_Calculate() 处理任意类型滤波器
 */
typedef struct {
  FilterType_e type; // 滤波器类型
  union {
    LPF1_t lpf1;
    AdaptiveLPF_t adaptive;
    MAF_t maf;
    MedianFilter_t median;
    MedianAvgFilter_t median_avg;
    Kalman1D_t kalman;
    LimitFilter_t limit;
  } impl; // 具体实现
} Filter_t;

/**
 * @brief 初始化通用滤波器为一阶低通
 */
void Filter_Init_LPF1(Filter_t *filter, float alpha);

/**
 * @brief 初始化通用滤波器为自适应低通
 */
void Filter_Init_AdaptiveLPF(Filter_t *filter, float min_alpha, float max_alpha,
                             float sensitivity);

/**
 * @brief 初始化通用滤波器为滑动平均
 */
void Filter_Init_MAF(Filter_t *filter, uint8_t size);

/**
 * @brief 初始化通用滤波器为中位值
 */
void Filter_Init_Median(Filter_t *filter, uint8_t size);

/**
 * @brief 初始化通用滤波器为中位值平均
 */
void Filter_Init_MedianAvg(Filter_t *filter, uint8_t size);

/**
 * @brief 初始化通用滤波器为一维卡尔曼
 */
void Filter_Init_Kalman1D(Filter_t *filter, float q, float r, float init_x,
                          float init_p);

/**
 * @brief 初始化通用滤波器为限幅滤波
 */
void Filter_Init_Limit(Filter_t *filter, float max_delta);

/**
 * @brief 通用滤波器计算（统一接口）
 * @param filter 滤波器指针
 * @param input 输入值
 * @return 滤波后的输出值
 */
float Filter_Calculate(Filter_t *filter, float input);

/**
 * @brief 获取滤波器当前输出（不更新状态）
 */
float Filter_GetOutput(const Filter_t *filter);

#ifdef __cplusplus
}
#endif

#endif //__FILTER_H
