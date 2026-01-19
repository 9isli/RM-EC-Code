//
// Created by pomelo on 2025/11/17.
//

#include "cmsis_os.h"
#include "gm6020_imu.h"
#include "m3508_rpm.h"
#include "task_user.h"
#include "temperature_IMU_pid.h"

// IMU 任务温度就绪与达温开关标志
volatile uint8_t g_imu_temp_ready = 0;        // 1 = 已经达到目标温度附近
volatile uint8_t g_imu_temp_check_enable = 0; // 1 = 需要达温后才启动 IMU 任务
// 云台阶跃测试开关与参数
#define ENABLE_GIMBAL_STEP_TEST 0
#define GIMBAL_STEP_INTERVAL_MS 5000U // 每 2000ms 变一次角度
// static float throttle = 0.0f;

/**
 * @brief IMU 温度控制任务，维持 IMU 在目标温度附近
 * @param argument 未使用
 */

void Temperature_ControlTask(void *argument) {
  (void)argument;

  // 等待系统初始化任务完成
  while (!g_system_init_done) {
    osDelay(1);
  }

  // 初始化任务中已经完成温度 PID 初始化和目标温度设置
  // 这里仅负责维持温度控制闭环
  for (;;) {
    // 每隔固定时间对云台 Yaw 角度做一次阶跃变化，用于调试 LADRC 参数
    // static const float test_angles_deg[] = {180.0f, 720.0f};
    // static const uint8_t test_angles_count =
    //     sizeof(test_angles_deg) / sizeof(test_angles_deg[0]);
    // static uint8_t index = 0;
    // static uint32_t last_tick = 0;

    // uint32_t now = osKernelGetTickCount();
    // if ((now - last_tick) >= GIMBAL_STEP_INTERVAL_MS) {
    //   last_tick = now;
    //   IMU_Yaw_Target(test_angles_deg[index]);
    //   index++;
    //   if (index >= test_angles_count) {
    //     index = 0;
    //   }
    // }
    // GM6020_IMU_Yaw_Speed_Control_Mode(throttle, 0.001);
    //  CAN_cmd_gimbal_gm6020(5000, 0, 0, 0);
    //  GM6020_IMU_Set_Yaw_Target(90);
    //  IMU_Yaw_Target(90);
    Temperature_PID_Loop();
    osDelay(1);
  }
}
