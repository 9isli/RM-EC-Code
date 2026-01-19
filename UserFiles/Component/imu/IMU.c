//
// Created by pomelo on 2025/11/5.
//

#include "IMU.h"
#include "bmi088_module.h"
#include "ist8310driver.h"
#include "robot_config.h"
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// PI控制器参数（来自 robot_config.h）
#define Kp IMU_AHRS_KP
#define Ki IMU_AHRS_KI

// 二阶低通Butterworth滤波器参数（来自 robot_config.h）
#define FILTER_CUTOFF_FREQ IMU_FILTER_CUTOFF_FREQ
#define FILTER_SAMPLE_FREQ IMU_FILTER_SAMPLE_FREQ

// 二阶Butterworth滤波器系数（根据截止频率和采样频率计算）
// 计算公式：wc = 2*pi*fc/fs
// 如需修改截止频率，需要重新计算系数
// fs = 1000Hz, fc = 50Hz (根据实际解算频率计算)
#define FILTER_B0 0.020083f
#define FILTER_B1 0.040167f
#define FILTER_B2 0.020083f
#define FILTER_A1 (-1.561018f)
#define FILTER_A2 0.641352f

// BMI088加速度计系数
#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

// BMI088陀螺仪系数
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

// 全局变量声明
volatile float exInt, eyInt, ezInt; // 误差积分
volatile float q0, q1, q2, q3;      // 全局四元数
volatile uint32_t lastUpdate, now;  // 采样周期计数 单位 0.1ms

// 累计角度相关变量
static float yaw_total_angle = 0.0f;
static float yaw_last_angle = 0.0f;
static int32_t yaw_round_count = 0;
static uint8_t yaw_first_flag = 1;

// 陀螺仪校准相关变量
static double Gyro_fill[3][300];
static double Gyro_total[3];
static double sqrGyro_total[3];
static int GyroinitFlag = 0;
static int GyroCount = 0;
static float gyro_offset[3] = {0};
static int CalCount = 0;

// 传感器数据缓存
static float TTangles_gyro[7]; // 传感器原始数据
static float gyroomega[3] = {0};
static float rawgyro[3] = {0};

// 滤波器状态变量（每个轴都需要两个延迟单元）
// 加速度计滤波器状态
static float accel_x_state[2] = {0.0f, 0.0f};
static float accel_y_state[2] = {0.0f, 0.0f};
static float accel_z_state[2] = {0.0f, 0.0f};

// 陀螺仪滤波器状态
static float gyro_x_state[2] = {0.0f, 0.0f};
static float gyro_y_state[2] = {0.0f, 0.0f};
static float gyro_z_state[2] = {0.0f, 0.0f};

// 外部时间变量（由定时器维护，每0.1ms累加一次）
extern volatile uint32_t nowtime; // 时间戳，单位0.1ms

/**
 * @brief 二阶低通Butterworth滤波器实现
 * @details 使用差分方程进行滤波：y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] -
 * a1*y[n-1] - a2*y[n-2]
 * @param[in] input 输入信号值
 * @param[in,out] state 指向状态数组的指针，用于存储延迟单元
 *                      state[0] = y[n-1]，state[1] = y[n-2]
 * @return 滤波后的输出值
 * @note 截止频率30Hz，采样频率200Hz
 */
static float butterworth_filter(float input, float *state) {
  float output;

  // 计算输出
  output = FILTER_B0 * input + FILTER_B1 * state[0] + FILTER_B2 * state[1] -
           FILTER_A1 * state[0] - FILTER_A2 * state[1];

  // 更新状态（延迟单元）
  state[1] = state[0];
  state[0] = output;

  return output;
}

/**
 * @brief 快速反平方根计算（Quake III算法）
 * @details 使用魔数0x5f3759df进行快速计算，精度高且速度快
 * @param[in] x 输入值
 * @return 1/sqrt(x)的近似值
 * @note 这是一个经典的优化算法，用于四元数规范化
 */
float invSqrt1(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

/**
 * @brief 由当前四元数和角速度计算四元数导数 q_dot
 * @note  采用标准形式 q_dot = 0.5 * q ⊗ ω
 */
static void quat_derivative(float q0, float q1, float q2, float q3, float gx,
                            float gy, float gz, float *dq0, float *dq1,
                            float *dq2, float *dq3) {
  *dq0 = -0.5f * (q1 * gx + q2 * gy + q3 * gz);
  *dq1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  *dq2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  *dq3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
}

/**
 * @brief 计算陀螺仪数据的方差（用于校准）
 * @details 使用滑动窗口方法计算三轴陀螺仪数据的方差和平均值
 * @param[in] data 输入的陀螺仪数据数组（3个元素：gx, gy, gz）
 * @param[in] length 滑动窗口长度（通常为100或300）
 * @param[out] sqrResult 输出的方差数组（3个元素）
 * @param[out] avgResult 输出的平均值数组（3个元素）
 * @retval 无
 * @note 方差小于0.02表示陀螺仪数据稳定，可以进行校准
 */
void calGyroVariance(float data[], int length, float sqrResult[],
                     float avgResult[]) {
  int i;
  double tmplen;
  if (GyroinitFlag == 0) {
    for (i = 0; i < 3; i++) {
      Gyro_fill[i][GyroCount] = data[i];
      Gyro_total[i] += data[i];
      sqrGyro_total[i] += data[i] * data[i];
      sqrResult[i] = 100;
      avgResult[i] = 0;
    }
  } else {
    for (i = 0; i < 3; i++) {
      Gyro_total[i] -= Gyro_fill[i][GyroCount];
      sqrGyro_total[i] -= Gyro_fill[i][GyroCount] * Gyro_fill[i][GyroCount];
      Gyro_fill[i][GyroCount] = data[i];
      Gyro_total[i] += Gyro_fill[i][GyroCount];
      sqrGyro_total[i] += Gyro_fill[i][GyroCount] * Gyro_fill[i][GyroCount];
    }
  }
  GyroCount++;
  if (GyroCount >= length) {
    GyroCount = 0;
    GyroinitFlag = 1;
  }
  if (GyroinitFlag == 0) {
    return;
  }
  tmplen = length;
  for (i = 0; i < 3; i++) {
    avgResult[i] = (float)(Gyro_total[i] / tmplen);
    sqrResult[i] =
        (float)((sqrGyro_total[i] - Gyro_total[i] * Gyro_total[i] / tmplen) /
                tmplen);
  }
}

/**
 * @brief 初始化IMU传感器模块
 * @details 初始化BMI088加速度计和陀螺仪，IST8310磁力计，
 *          以及四元数、积分项、陀螺仪校准和滤波器状态
 * @retval 无
 * @note 必须在系统启动时调用一次，在所有其他IMU函数之前调用
 */
void IMU_Init(void) {
  // 初始化传感器
  BMI088_Module_Init();
  IST8310_Init(NULL);

  // 初始化四元数
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;

  // 初始化积分项
  exInt = 0.0f;
  eyInt = 0.0f;
  ezInt = 0.0f;

  // 初始化陀螺仪校准
  GyroinitFlag = 0;
  GyroCount = 0;
  CalCount = 0;
  for (int i = 0; i < 3; i++) {
    Gyro_total[i] = 0;
    sqrGyro_total[i] = 0;
    gyro_offset[i] = 0;
  }

  // 初始化滤波器状态
  accel_x_state[0] = accel_x_state[1] = 0.0f;
  accel_y_state[0] = accel_y_state[1] = 0.0f;
  accel_z_state[0] = accel_z_state[1] = 0.0f;
  gyro_x_state[0] = gyro_x_state[1] = 0.0f;
  gyro_y_state[0] = gyro_y_state[1] = 0.0f;
  gyro_z_state[0] = gyro_z_state[1] = 0.0f;

  // 初始化时间戳
  lastUpdate = nowtime;
  now = nowtime;

  // 初始化累计角度
  yaw_total_angle = 0.0f;
  yaw_last_angle = 0.0f;
  yaw_round_count = 0;
  yaw_first_flag = 1;
}

/**
 * @brief 读取传感器数据并进行校准
 * @details 读取BMI088加速度计和陀螺仪数据，应用低通滤波，
 *          进行陀螺仪方差计算和零偏校准
 * @param[out] values 输出数据数组（7个元素）
 *                    values[0-2] = 加速度计数据(ax, ay, az)，单位：g
 *                    values[3-5] = 校准后的陀螺仪数据(gx, gy, gz)，单位：度/秒
 *                    values[6] = 磁力计数据（预留）
 * @retval 无
 * @note 此函数包含陀螺仪自动校准逻辑，校准完成后积分项会被重置
 */
void IMU_getValues(float *values) {
  float accgyroval[7];
  float sqrResult_gyro[3];
  float avgResult_gyro[3];

  // 读取BMI088数据
  BMI088_Data_t bmi088_data;
  if (BMI088_Module_Read(&bmi088_data) == BMI08X_OK) {
    // 转换加速度计数据 (6g量程)
    accgyroval[0] = bmi088_data.accel.x * BMI088_ACCEL_6G_SEN;
    accgyroval[1] = bmi088_data.accel.y * BMI088_ACCEL_6G_SEN;
    accgyroval[2] = bmi088_data.accel.z * BMI088_ACCEL_6G_SEN;

    // 转换陀螺仪数据到度/秒 (2000dps量程)
    accgyroval[3] = bmi088_data.gyro.x * BMI088_GYRO_2000_SEN * 180.0f / M_PI;
    accgyroval[4] = bmi088_data.gyro.y * BMI088_GYRO_2000_SEN * 180.0f / M_PI;
    accgyroval[5] = bmi088_data.gyro.z * BMI088_GYRO_2000_SEN * 180.0f / M_PI;

    // 保存未滤波的原始角速度数据
    rawgyro[0] = accgyroval[3];
    rawgyro[1] = accgyroval[4];
    rawgyro[2] = accgyroval[5];

    // 二阶低通Butterworth滤波器
    accgyroval[0] = butterworth_filter(accgyroval[0], accel_x_state);
    accgyroval[1] = butterworth_filter(accgyroval[1], accel_y_state);
    accgyroval[2] = butterworth_filter(accgyroval[2], accel_z_state);
    accgyroval[3] = butterworth_filter(accgyroval[3], gyro_x_state);
    accgyroval[4] = butterworth_filter(accgyroval[4], gyro_y_state);
    accgyroval[5] = butterworth_filter(accgyroval[5], gyro_z_state);
  }

  // 保存原始数据
  TTangles_gyro[0] = accgyroval[0];
  TTangles_gyro[1] = accgyroval[1];
  TTangles_gyro[2] = accgyroval[2];
  TTangles_gyro[3] = accgyroval[3];
  TTangles_gyro[4] = accgyroval[4];
  TTangles_gyro[5] = accgyroval[5];
  TTangles_gyro[6] = 0; // 磁力计数据

  // 陀螺仪校准
  calGyroVariance(&TTangles_gyro[3], 100, sqrResult_gyro, avgResult_gyro);
  if (sqrResult_gyro[0] < 0.02f && sqrResult_gyro[1] < 0.02f &&
      sqrResult_gyro[2] < 0.02f && CalCount >= 99) {
    gyro_offset[0] = avgResult_gyro[0];
    gyro_offset[1] = avgResult_gyro[1];
    gyro_offset[2] = avgResult_gyro[2];
    exInt = 0;
    eyInt = 0;
    ezInt = 0;
    CalCount = 0;
  } else if (CalCount < 100) {
    CalCount++;
  }

  // 返回校准后的数据
  values[0] = accgyroval[0];
  values[1] = accgyroval[1];
  values[2] = accgyroval[2];
  values[3] = accgyroval[3] - gyro_offset[0];
  values[4] = accgyroval[4] - gyro_offset[1];
  values[5] = accgyroval[5] - gyro_offset[2];
  values[6] = 0; // 磁力计数据

  gyroomega[0] = values[3];
  gyroomega[1] = values[4];
  gyroomega[2] = values[5];
}

/**
 * @brief Mahony AHRS算法更新四元数
 * @details 使用加速度计、陀螺仪和磁力计数据进行传感器融合，
 *          通过PI控制器修正陀螺仪零偏，更新四元数
 * @param[in] gx 陀螺仪X轴角速度，单位：弧度/秒
 * @param[in] gy 陀螺仪Y轴角速度，单位：弧度/秒
 * @param[in] gz 陀螺仪Z轴角速度，单位：弧度/秒
 * @param[in] ax 加速度计X轴加速度，单位：g
 * @param[in] ay 加速度计Y轴加速度，单位：g
 * @param[in] az 加速度计Z轴加速度，单位：g
 * @param[in] mx 磁力计X轴磁场，单位：uT
 * @param[in] my 磁力计Y轴磁场，单位：uT
 * @param[in] mz 磁力计Z轴磁场，单位：uT
 * @retval 无
 * @note 此函数会更新全局四元数q0, q1, q2, q3和积分项exInt, eyInt, ezInt
 */
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az,
                    float mx, float my, float mz) {
  float norm;
  float vx, vy, vz;
  float ex, ey, ez, halfT, dt;
  float tempq0, tempq1, tempq2, tempq3;
  float k1_0, k1_1, k1_2, k1_3;
  float k2_0, k2_1, k2_2, k2_3;
  float k3_0, k3_1, k3_2, k3_3;
  float k4_0, k4_1, k4_2, k4_3;
  float q_temp0, q_temp1, q_temp2, q_temp3;

  // 预计算四元数乘积
  float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  // float q0q3 = q0 * q3;
  float q1q1 = q1 * q1;
  // float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;

  // 计算时间差，得到采样周期 dt（单位：秒）
  now = nowtime;
  if (now < lastUpdate) {
    dt = ((float)(now + (0xffff - lastUpdate)) / 10000.0f);
  } else {
    dt = ((float)(now - lastUpdate) / 10000.0f);
  }
  lastUpdate = now;
  // halfT ：采样周期的一半，主要用于 PI 积分
  halfT = dt * 0.5f;

  // 归一化加速度计
  norm = invSqrt1(ax * ax + ay * ay + az * az);
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;

  // 归一化磁力计
  norm = invSqrt1(mx * mx + my * my + mz * mz);
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  // 计算重力向量
  vx = 2 * (q1q3 - q0q2);
  vy = 2 * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;

  // 计算误差
  ex = (ay * vz - az * vy);
  ey = (az * vx - ax * vz);
  ez = (ax * vy - ay * vx);

  // PI 控制器（仍然使用原有的积分离散形式）
  if (ex != 0.0f && ey != 0.0f && ez != 0.0f) {
    exInt = exInt + ex * Ki * halfT;
    eyInt = eyInt + ey * Ki * halfT;
    ezInt = ezInt + ez * Ki * halfT;

    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;
  }

  // 四元数微分方程：使用标准四阶 Runge-Kutta 积分
  // k1
  quat_derivative(q0, q1, q2, q3, gx, gy, gz, &k1_0, &k1_1, &k1_2, &k1_3);

  // k2
  q_temp0 = q0 + 0.5f * dt * k1_0;
  q_temp1 = q1 + 0.5f * dt * k1_1;
  q_temp2 = q2 + 0.5f * dt * k1_2;
  q_temp3 = q3 + 0.5f * dt * k1_3;
  quat_derivative(q_temp0, q_temp1, q_temp2, q_temp3, gx, gy, gz, &k2_0, &k2_1,
                  &k2_2, &k2_3);

  // k3
  q_temp0 = q0 + 0.5f * dt * k2_0;
  q_temp1 = q1 + 0.5f * dt * k2_1;
  q_temp2 = q2 + 0.5f * dt * k2_2;
  q_temp3 = q3 + 0.5f * dt * k2_3;
  quat_derivative(q_temp0, q_temp1, q_temp2, q_temp3, gx, gy, gz, &k3_0, &k3_1,
                  &k3_2, &k3_3);

  // k4
  q_temp0 = q0 + dt * k3_0;
  q_temp1 = q1 + dt * k3_1;
  q_temp2 = q2 + dt * k3_2;
  q_temp3 = q3 + dt * k3_3;
  quat_derivative(q_temp0, q_temp1, q_temp2, q_temp3, gx, gy, gz, &k4_0, &k4_1,
                  &k4_2, &k4_3);

  // Runge-Kutta 合成：q_{n+1} = q_n + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
  tempq0 = q0 + (dt / 6.0f) * (k1_0 + 2.0f * k2_0 + 2.0f * k3_0 + k4_0);
  tempq1 = q1 + (dt / 6.0f) * (k1_1 + 2.0f * k2_1 + 2.0f * k3_1 + k4_1);
  tempq2 = q2 + (dt / 6.0f) * (k1_2 + 2.0f * k2_2 + 2.0f * k3_2 + k4_2);
  tempq3 = q3 + (dt / 6.0f) * (k1_3 + 2.0f * k2_3 + 2.0f * k3_3 + k4_3);

  // 四元数规范化
  norm = invSqrt1(tempq0 * tempq0 + tempq1 * tempq1 + tempq2 * tempq2 +
                  tempq3 * tempq3);
  q0 = tempq0 * norm;
  q1 = tempq1 * norm;
  q2 = tempq2 * norm;
  q3 = tempq3 * norm;
}

/**
 * @brief 获取当前四元数
 * @details 读取传感器数据，更新AHRS算法，返回最新的四元数
 * @param[out] q 输出四元数数组（4个元素）
 *               q[0] = q0，q[1] = q1，q[2] = q2，q[3] = q3
 * @retval 无
 * @note 此函数会自动调用IMU_getValues()和IMU_AHRSupdate()
 */
float mygetqval[9];
void IMU_getQ(float *q) {
  IMU_getValues(mygetqval);
  // 将陀螺仪的测量值转成弧度每秒
  IMU_AHRSupdate(mygetqval[3] * M_PI / 180, mygetqval[4] * M_PI / 180,
                 mygetqval[5] * M_PI / 180, mygetqval[0], mygetqval[1],
                 mygetqval[2], mygetqval[6], mygetqval[7], mygetqval[8]);

  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

/**
 * @brief 获取欧拉角（偏航角、俯仰角、滚转角）
 * @details 从四元数计算欧拉角，返回设备的姿态信息
 * @param[out] angles 输出欧拉角数组（3个元素）
 *                    angles[0] = 偏航角(Yaw)，单位：度，范围：[-180, 180]
 *                    angles[1] = 俯仰角(Pitch)，单位：度，范围：[-90, 90]
 *                    angles[2] = 滚转角(Roll)，单位：度，范围：[-180, 180]
 * @retval 无
 * @note 此函数会自动调用IMU_getQ()更新四元数
 */
void IMU_getYawPitchRoll(float *angles) {
  float q[4];
  IMU_getQ(q);

  // 右手准则ENU
  angles[0] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3],
                    -2 * q[2] * q[2] - 2 * q[3] * q[3] + 1) *
              180 / M_PI;                                            // yaw
  angles[1] = asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180 / M_PI; // pitch
  angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1],
                    -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1) *
              180 / M_PI; // roll

  // 计算累计偏航角
  if (yaw_first_flag) {
    yaw_last_angle = angles[0];
    yaw_total_angle = angles[0];
    yaw_first_flag = 0;
  } else {
    // 检测跳变
    if (angles[0] - yaw_last_angle > 180.0f) {
      yaw_round_count--;
    } else if (angles[0] - yaw_last_angle < -180.0f) {
      yaw_round_count++;
    }
    yaw_total_angle = yaw_round_count * 360.0f + angles[0];
    yaw_last_angle = angles[0];
  }
}

/**
 * @brief 获取累计偏航角 (Total Yaw Angle)
 * @return 累计偏航角，单位：度，范围：无限制 (连续)
 */
float IMU_getYawTotalAngle(void) { return yaw_total_angle; }

/**
 * @brief 获取传感器原始数据
 * @details 返回最后一次读取的传感器原始数据（未经过AHRS处理）
 * @param[out] zsjganda 输出数据数组（7个元素）
 *                      zsjganda[0-2] = 加速度计数据(ax, ay, az)
 *                      zsjganda[3-5] = 陀螺仪数据(gx, gy, gz)
 *                      zsjganda[6] = 磁力计数据
 * @retval 无
 * @note 此函数返回的是缓存的数据，不会进行新的传感器读取
 */
void IMU_TT_getgyro(float *zsjganda) {
  zsjganda[0] = TTangles_gyro[0];
  zsjganda[1] = TTangles_gyro[1];
  zsjganda[2] = TTangles_gyro[2];
  zsjganda[3] = TTangles_gyro[3];
  zsjganda[4] = TTangles_gyro[4];
  zsjganda[5] = TTangles_gyro[5];
  zsjganda[6] = TTangles_gyro[6];
}

/**
 * @brief 获取陀螺仪角速度数据
 * @param[out] gyro 输出角速度数组（3个元素）
 *                  gyro[0] = X轴角速度(Roll方向)，单位：度/秒
 *                  gyro[1] = Y轴角速度(Pitch方向)，单位：度/秒
 *                  gyro[2] = Z轴角速度(Yaw方向)，单位：度/秒
 * @retval 无
 * @note 返回的是校准后的陀螺仪数据
 */
void IMU_getGyroData(float *gyro) {
  // gyro[0-2] 存储的是校准的陀螺仪数据
  // gyro[3-5] 存储的是原始的陀螺仪数据
  gyro[0] = gyroomega[0];
  gyro[1] = gyroomega[1];
  gyro[2] = gyroomega[2];
  gyro[3] = rawgyro[0] - gyro_offset[0];
  gyro[4] = rawgyro[1] - gyro_offset[1];
  gyro[5] = rawgyro[2] - gyro_offset[2];
}