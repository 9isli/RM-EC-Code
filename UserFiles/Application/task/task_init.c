//
// Created by pomelo on 2025/11/17.
//

#include "IMU.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_sbus.h"
#include "bsp_uart.h"
#include "buzzer.h"
#include "chassis_control.h"
#include "chassis_imu.h"
#include "cmsis_os.h"
#include "dual_board_comm.h"
#include "gm6020_imu.h"
#include "led.h"
#include "m3508_rpm.h"
#include "robot_config.h"
#include "robot_control.h"
#include "task_user.h"
#include "temperature_IMU_pid.h"

#if CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL
#include "shoot_control.h"
#endif

// 本地常量
#define MOTOR_COUNT 4
#define MOTOR_CHECK_INTERVAL_MS 50U
#define DUAL_BOARD_COMM_CHECK_INTERVAL_MS 50U

// 底盘四个电机的存活标志，1 表示该电机温度正常、认为存活
uint8_t g_chassis_motor_alive[MOTOR_COUNT] = {0};
// 所有底盘电机是否都正常
uint8_t g_chassis_all_motors_ok = 0;

// 双板通信是否就绪：1 表示对端板在线
volatile uint8_t g_dual_board_comm_ready = 0;

// 系统初始化完成标志：1 表示 System_InitTask 完成，允许其他任务正常运行
volatile uint8_t g_system_init_done = 0;

extern TIM_HandleTypeDef htim6;

/**
 * @brief 系统初始化任务，负责外设、控制算法和温度等初始化
 * @param argument 未使用
 */

void System_InitTask(void *argument) {
  (void)argument;

  // 初始化 DWT 计数器 (用于高精度计时)
  DWT_Init(168); // STM32F407 主频 168MHz

  // 初始化需要在 RTOS 环境下完成的模块
  HAL_TIM_Base_Start_IT(&htim6);
  USER_CAN_Filter_Init();
  CAN_Manager_Init();
  DualBoard_Comm_Init(); // 双板通信初始化
  IMU_Init();
  UART1_Init();
  UART6_Init();
  SBUS_Init(); // 初始化 S.BUS 接收
  LED_Init();

  // 初始化控制相关算法
  M3508_RPM_Init();
  GM6020_IMU_Init();
  Chassis_Init(); // 底盘运动学控制
  Robot_Init();   // 整车协调控制
  Temperature_PID_Init();

#if CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL
  Shoot_Init(); // 发射机构控制（仅云台板）
#endif

  // 在初始化阶段等待 IMU 达到目标温度附近
  if (g_imu_temp_check_enable) {
    Temperature_PID_Set_Target(IMU_TEMP_TARGET);

    uint32_t imu_elapsed = 0;

    while (imu_elapsed < IMU_TEMP_TIMEOUT_MS) {
      Temperature_PID_Loop();

      float current = Temperature_PID_Get_Current_Temp();
      float target = Temperature_PID_Get_Target_Temp();
      float diff = current - target;
      if (diff < 0.0f)
        diff = -diff;

      if (diff <= IMU_TEMP_TOLERANCE) {
        g_imu_temp_ready = 1;
        break;
      }

      osDelay(1);
      imu_elapsed += 1;
    }

    if (!g_imu_temp_ready) {
      g_imu_temp_ready = 1;
    }
  } else {
    // 不需要达温，直接认为已就绪，方便无电池等调试场景
    g_imu_temp_ready = 1;
  }

#if INIT_ENABLE_MOTOR_CHECK
  // 给电机和总线一点时间开始工作，让反馈温度先上来
  osDelay(100);

  uint32_t elapsed = 0;

  // 在超时时间内轮询检查四个底盘电机的温度是否处于正常范围
  while (elapsed < MOTOR_CHECK_TIMEOUT_MS) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
      const motor_measure_t *m = get_chassis_m3508_measure_point(i);
      uint8_t temp = m->temperate;

      // 简单判定：温度在合理物理范围内则认为该电机“存活”
      if (temp >= MOTOR_TEMP_MIN && temp <= MOTOR_TEMP_MAX) {
        g_chassis_motor_alive[i] = 1;
      }
    }

    g_chassis_all_motors_ok = 1;
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
      if (!g_chassis_motor_alive[i]) {
        g_chassis_all_motors_ok = 0;
        break;
      }
    }

    // 全部电机正常则提前结束检测
    if (g_chassis_all_motors_ok) {
      break;
    }

    osDelay(MOTOR_CHECK_INTERVAL_MS);
    elapsed += MOTOR_CHECK_INTERVAL_MS;
  }
#else
  // 调试时关闭电机自检：默认认为所有电机在线
  g_chassis_all_motors_ok = 1;
#endif

#if INIT_WAIT_DUAL_BOARD_COMM
  // 绿灯闪烁：正在等待对端板
  // 绿灯常亮：通信建立成功
  // 红灯常亮：通信超时（但仍允许启动）
  {
    uint32_t comm_elapsed = 0;
    uint8_t led_state = 0;
    uint32_t led_tick = 0;

    // 开始发送心跳，等待对端响应
    while (comm_elapsed < DUAL_BOARD_INIT_TIMEOUT_MS) {
      DualBoard_Send_Heartbeat();

      DualBoard_Comm_Update();

      // 检查对端是否在线
      if (DualBoard_Is_Peer_Online()) {
        g_dual_board_comm_ready = 1;
        // 通信成功
        LED_Green(ON);
        LED_Red(OFF);
        osDelay(200);
        LED_Green(OFF);
        break;
      }

      // 绿灯闪烁指示等待中
      if (HAL_GetTick() - led_tick > 100) {
        led_tick = HAL_GetTick();
        led_state = !led_state;
        LED_Green(led_state ? ON : OFF);
      }

      osDelay(DUAL_BOARD_COMM_CHECK_INTERVAL_MS);
      comm_elapsed += DUAL_BOARD_COMM_CHECK_INTERVAL_MS;
    }

    // 超时处理
    if (!g_dual_board_comm_ready) {
      // 通信超时，红灯闪烁 3 次警告，但仍允许系统启动
      LED_Green(OFF);
      for (uint8_t i = 0; i < 3; i++) {
        LED_Red(ON);
        osDelay(100);
        LED_Red(OFF);
        osDelay(100);
      }
      // 注意：g_dual_board_comm_ready 保持为 0，运行时会持续检测
    }
  }
#else
  // 单板调试模式：跳过双板通信等待
  g_dual_board_comm_ready = 1;
  LED_Green(OFF);
  LED_Red(OFF);
#endif

  // DJI经典提示音：）
  if (g_chassis_all_motors_ok) {
    Buzzer_SetFrequency(523.26f);
    osDelay(180);
    Buzzer_Off();
    osDelay(40);

    Buzzer_SetFrequency(587.32f);
    osDelay(180);
    Buzzer_Off();
    osDelay(40);

    Buzzer_SetFrequency(784.00f);
    osDelay(260);
    Buzzer_Off();
  } else {
    for (uint8_t k = 0; k < 3; k++) {
      Buzzer_SetFrequency(300.0f);
      osDelay(250);
      Buzzer_Off();
      osDelay(100);
    }
  }

  // 标记系统初始化完成，允许其他任务开始工作
  g_system_init_done = 1;
  osThreadTerminate(osThreadGetId());
}
