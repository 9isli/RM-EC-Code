/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task_user.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t initTaskHandle;
const osThreadAttr_t initTask_attributes = {
  .name = "initTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t tempControlTaskHandle;
const osThreadAttr_t tempControlTask_attributes = {
  .name = "tempControlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t monitorTaskHandle;
const osThreadAttr_t monitorTask_attributes = {
  .name = "monitorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t)osPriorityRealtime,
};

osSemaphoreId_t controlSemHandle;
osSemaphoreId_t imuSemHandle;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void System_InitTask(void *argument);
void Temperature_ControlTask(void *argument);
void Monitor_Task(void *argument);
void IMU_Task(void *argument);
void Control_1kHz_Task(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  controlSemHandle = osSemaphoreNew(1, 0, NULL);
  imuSemHandle     = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  initTaskHandle = osThreadNew(System_InitTask, NULL, &initTask_attributes);
  tempControlTaskHandle = osThreadNew(Temperature_ControlTask, NULL, &tempControlTask_attributes);
  monitorTaskHandle = osThreadNew(Monitor_Task, NULL, &monitorTask_attributes);
  imuTaskHandle = osThreadNew(IMU_Task, NULL, &imuTask_attributes);
  controlTaskHandle = osThreadNew(Control_1kHz_Task, NULL, &controlTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

