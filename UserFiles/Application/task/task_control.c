//
// 统一控制任务
// Created by pomelo on 2025/11/17
//
// 根据 CURRENT_BOARD_ROLE 宏在编译时选择执行逻辑：
// BOARD_ROLE_GIMBAL: 云台板控制（遥控器读取、云台控制、底盘命令发送）
// BOARD_ROLE_CHASSIS: 底盘板控制（接收命令、M3508 控制、IMU 上报）
//
// CAN1: 电机控制（云台板->GM6020，底盘板->M3508）
// CAN2: 双板通信（底盘命令、IMU 数据、心跳）
//

#include "bsp_can.h"
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

#if CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL
#include "shoot_control.h"
#endif

// 在 freertos.c 中创建的信号量句柄
extern osSemaphoreId_t controlSemHandle;

/**
 * @brief 1kHz 统一控制任务
 * @param argument 未使用
 * @note 根据板子角色执行不同的控制逻辑
 *       - 云台板：遥控器 -> 运动决策 -> 云台控制 -> 底盘命令发送
 *       - 底盘板：接收命令 -> M3508 控制 -> IMU 上报
 */
void Control_1kHz_Task(void *argument) {
  (void)argument;

  // 等待系统初始化任务完成
  while (!g_system_init_done) {
    osDelay(1);
  }

  // LED 状态：
  // 通信正常：LED 熄灭
  // 通信异常：红灯常亮

  for (;;) {
    // TIM6 中断通过信号量唤醒（1kHz）
    osSemaphoreAcquire(controlSemHandle, osWaitForever);

    // 双板通信检查
    DualBoard_Comm_Update();

#if CURRENT_BOARD_ROLE == BOARD_ROLE_GIMBAL
    // ====================云台板控制逻辑====================

    // 更新底盘 IMU 数据（从 CAN 缓冲区同步）
    Chassis_IMU_Update();

    Robot_Control_Loop();

    Chassis_Control_Loop();

    GM6020_IMU_Control_Loop();

    Shoot_Control_Loop();

    CAN_Manager_SendAll();

#else // BOARD_ROLE_CHASSIS
    // ====================底盘板控制逻辑====================

    // 发送 IMU 数据到云台板 (CAN2)
    Chassis_IMU_CAN_Transmit();

    const Chassis_Cmd_t *cmd = DualBoard_Get_Chassis_Cmd();

    // 根据通信状态控制电机
    if (DualBoard_Is_Peer_Online()) {
      M3508_Set_RPM(cmd->wheel_rpm[0], cmd->wheel_rpm[1], cmd->wheel_rpm[2],
                    cmd->wheel_rpm[3]);
    } else {
      M3508_Set_RPM(0, 0, 0, 0);
    }

    M3508_RPM_Control_Loop();

    // 发送 CAN1 数据
    CAN_Manager_SendAll();

#endif // CURRENT_BOARD_ROLE

    // 发送心跳包 (50ms 间隔，由函数内部控制)
    DualBoard_Send_Heartbeat();

    // 通信状态 LED 指示
#if INIT_WAIT_DUAL_BOARD_COMM
    if (!DualBoard_Is_Peer_Online()) {
      LED_Red(ON);
      LED_Green(OFF);
    } else {
      LED_Red(OFF);
      LED_Green(OFF);
    }
#else
    // 单板模式下不亮红灯报警
    LED_Red(OFF);
    LED_Green(OFF);
#endif
  }
}
