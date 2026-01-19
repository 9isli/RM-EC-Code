//
// Created by pomelo on 2025/11/30.
// 通用滤波器库实现
//

#include "filter.h"
#include <math.h>
#include <string.h>

#ifndef PI
#define PI 3.14159265359f
#endif

extern "C" {

/* ========================== 一阶低通滤波器 ========================== */

void LPF1_Init(LPF1_t *filter, float alpha) {
  filter->output = 0.0f;
  filter->alpha = (alpha > 1.0f) ? 1.0f : ((alpha < 0.0f) ? 0.0f : alpha);
}

void LPF1_Init_RC(LPF1_t *filter, float rc, float dt) {
  filter->output = 0.0f;
  filter->alpha = dt / (rc + dt);
}

void LPF1_Init_Cutoff(LPF1_t *filter, float cutoff_hz, float dt) {
  float rc = 1.0f / (2.0f * PI * cutoff_hz);
  LPF1_Init_RC(filter, rc, dt);
}

float LPF1_Calculate(LPF1_t *filter, float input) {
  filter->output =
      filter->alpha * input + (1.0f - filter->alpha) * filter->output;
  return filter->output;
}

/* ========================== 自适应低通滤波器 ========================== */

void AdaptiveLPF_Init(AdaptiveLPF_t *filter, float min_alpha, float max_alpha,
                      float sensitivity) {
  filter->output = 0.0f;
  filter->min_alpha = min_alpha;
  filter->max_alpha = max_alpha;
  filter->sensitivity = (sensitivity > 0.0f) ? sensitivity : 1.0f;
}

float AdaptiveLPF_Calculate(AdaptiveLPF_t *filter, float input) {
  float error = fabsf(input - filter->output);
  float alpha = filter->min_alpha + (filter->max_alpha - filter->min_alpha) *
                                        (error / (error + filter->sensitivity));
  filter->output = alpha * input + (1.0f - alpha) * filter->output;
  return filter->output;
}

/* ========================== 滑动平均滤波器 ========================== */

void MAF_Init(MAF_t *filter, uint8_t size) {
  memset(filter, 0, sizeof(MAF_t));
  filter->size = (size > MAF_MAX_SIZE) ? MAF_MAX_SIZE : size;
  if (filter->size < 1)
    filter->size = 1;
}

float MAF_Calculate(MAF_t *filter, float input) {
  filter->sum -= filter->buffer[filter->index];
  filter->buffer[filter->index] = input;
  filter->sum += input;
  filter->index = (filter->index + 1) % filter->size;
  if (filter->count < filter->size)
    filter->count++;
  return filter->sum / (float)filter->count;
}

/* ========================== 中位值滤波器 ========================== */

void MedianFilter_Init(MedianFilter_t *filter, uint8_t size) {
  memset(filter, 0, sizeof(MedianFilter_t));
  filter->size = (size > MEDIAN_MAX_SIZE) ? MEDIAN_MAX_SIZE : size;
  if (filter->size < 1)
    filter->size = 1;
}

static void BubbleSort(float *arr, uint8_t len) {
  for (uint8_t i = 0; i < len - 1; i++) {
    for (uint8_t j = 0; j < len - 1 - i; j++) {
      if (arr[j] > arr[j + 1]) {
        float temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

float MedianFilter_Calculate(MedianFilter_t *filter, float input) {
  filter->buffer[filter->index] = input;
  filter->index = (filter->index + 1) % filter->size;
  if (filter->count < filter->size)
    filter->count++;

  float sorted[MEDIAN_MAX_SIZE];
  memcpy(sorted, filter->buffer, filter->count * sizeof(float));
  BubbleSort(sorted, filter->count);
  return sorted[filter->count / 2];
}

/* ========================== 中位值平均滤波器 ========================== */

void MedianAvgFilter_Init(MedianAvgFilter_t *filter, uint8_t size) {
  memset(filter, 0, sizeof(MedianAvgFilter_t));
  filter->size = (size > MEDIAN_AVG_MAX_SIZE) ? MEDIAN_AVG_MAX_SIZE : size;
  if (filter->size < 3)
    filter->size = 3;
}

float MedianAvgFilter_Calculate(MedianAvgFilter_t *filter, float input) {
  filter->buffer[filter->index] = input;
  filter->index = (filter->index + 1) % filter->size;
  if (filter->count < filter->size)
    filter->count++;

  if (filter->count < 3)
    return input;

  float max_val = filter->buffer[0];
  float min_val = filter->buffer[0];
  float sum = 0.0f;

  for (uint8_t i = 0; i < filter->count; i++) {
    float val = filter->buffer[i];
    sum += val;
    if (val > max_val)
      max_val = val;
    if (val < min_val)
      min_val = val;
  }

  return (sum - max_val - min_val) / (float)(filter->count - 2);
}

/* ========================== 一维卡尔曼滤波器 ========================== */

void Kalman1D_Init(Kalman1D_t *filter, float q, float r, float init_x,
                   float init_p) {
  filter->x = init_x;
  filter->p = init_p;
  filter->q = q;
  filter->r = r;
  filter->k = 0.0f;
}

float Kalman1D_Calculate(Kalman1D_t *filter, float measurement) {
  filter->p = filter->p + filter->q;
  filter->k = filter->p / (filter->p + filter->r);
  filter->x = filter->x + filter->k * (measurement - filter->x);
  filter->p = (1.0f - filter->k) * filter->p;
  return filter->x;
}

/* ========================== 限幅滤波器 ========================== */

void LimitFilter_Init(LimitFilter_t *filter, float max_delta) {
  filter->output = 0.0f;
  filter->max_delta = fabsf(max_delta);
  filter->initialized = 0;
}

float LimitFilter_Calculate(LimitFilter_t *filter, float input) {
  if (!filter->initialized) {
    filter->output = input;
    filter->initialized = 1;
    return input;
  }

  if (fabsf(input - filter->output) <= filter->max_delta) {
    filter->output = input;
  }
  return filter->output;
}

/* ========================== 通用滤波器接口 ========================== */

void Filter_Init_LPF1(Filter_t *filter, float alpha) {
  filter->type = FILTER_TYPE_LPF1;
  LPF1_Init(&filter->impl.lpf1, alpha);
}

void Filter_Init_AdaptiveLPF(Filter_t *filter, float min_alpha, float max_alpha,
                             float sensitivity) {
  filter->type = FILTER_TYPE_ADAPTIVE_LPF;
  AdaptiveLPF_Init(&filter->impl.adaptive, min_alpha, max_alpha, sensitivity);
}

void Filter_Init_MAF(Filter_t *filter, uint8_t size) {
  filter->type = FILTER_TYPE_MAF;
  MAF_Init(&filter->impl.maf, size);
}

void Filter_Init_Median(Filter_t *filter, uint8_t size) {
  filter->type = FILTER_TYPE_MEDIAN;
  MedianFilter_Init(&filter->impl.median, size);
}

void Filter_Init_MedianAvg(Filter_t *filter, uint8_t size) {
  filter->type = FILTER_TYPE_MEDIAN_AVG;
  MedianAvgFilter_Init(&filter->impl.median_avg, size);
}

void Filter_Init_Kalman1D(Filter_t *filter, float q, float r, float init_x,
                          float init_p) {
  filter->type = FILTER_TYPE_KALMAN1D;
  Kalman1D_Init(&filter->impl.kalman, q, r, init_x, init_p);
}

void Filter_Init_Limit(Filter_t *filter, float max_delta) {
  filter->type = FILTER_TYPE_LIMIT;
  LimitFilter_Init(&filter->impl.limit, max_delta);
}

float Filter_Calculate(Filter_t *filter, float input) {
  switch (filter->type) {
  case FILTER_TYPE_LPF1:
    return LPF1_Calculate(&filter->impl.lpf1, input);
  case FILTER_TYPE_ADAPTIVE_LPF:
    return AdaptiveLPF_Calculate(&filter->impl.adaptive, input);
  case FILTER_TYPE_MAF:
    return MAF_Calculate(&filter->impl.maf, input);
  case FILTER_TYPE_MEDIAN:
    return MedianFilter_Calculate(&filter->impl.median, input);
  case FILTER_TYPE_MEDIAN_AVG:
    return MedianAvgFilter_Calculate(&filter->impl.median_avg, input);
  case FILTER_TYPE_KALMAN1D:
    return Kalman1D_Calculate(&filter->impl.kalman, input);
  case FILTER_TYPE_LIMIT:
    return LimitFilter_Calculate(&filter->impl.limit, input);
  default:
    return input;
  }
}

float Filter_GetOutput(const Filter_t *filter) {
  switch (filter->type) {
  case FILTER_TYPE_LPF1:
    return filter->impl.lpf1.output;
  case FILTER_TYPE_ADAPTIVE_LPF:
    return filter->impl.adaptive.output;
  case FILTER_TYPE_MAF:
    return filter->impl.maf.sum / (float)filter->impl.maf.count;
  case FILTER_TYPE_MEDIAN:
  case FILTER_TYPE_MEDIAN_AVG:
    return 0.0f; // 需要重新计算，返回0
  case FILTER_TYPE_KALMAN1D:
    return filter->impl.kalman.x;
  case FILTER_TYPE_LIMIT:
    return filter->impl.limit.output;
  default:
    return 0.0f;
  }
}

} // extern "C"
