/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "user_c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 任务ID 指针
osThreadId user_tset_task_handle;

osThreadId usb_task_handle;
osThreadId battery_voltage_handle;
osThreadId led_RGB_flow_handle;
osThreadId imuTaskHandle;

osThreadId chassisTaskHandle;
osThreadId calibrate_tast_handle;

osThreadId gimbalTaskHandle;
osThreadId servo_task_handle;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId testHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void test_tack(void const *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of test */

  osThreadDef(test, test_tack, osPriorityNormal, 0, 128);
  testHandle = osThreadCreate(osThread(test), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  //  // usb数据发送任务
  //  osThreadDef(USBTask, usb_task, osPriorityNormal, 0, 128);
  //  usb_task_handle = osThreadCreate(osThread(USBTask), NULL);

  // 电池电量检测任务
  osThreadDef(BATTERY_VOLTAGE, battery_voltage_task, osPriorityNormal, 0, 128);
  battery_voltage_handle = osThreadCreate(osThread(BATTERY_VOLTAGE), NULL);

  // 呼吸灯任务
  osThreadDef(led, led_RGB_flow_task, osPriorityNormal, 0, 256);
  led_RGB_flow_handle = osThreadCreate(osThread(led), NULL);

  // 姿态任务
  osThreadDef(imuTask, INS_task, osPriorityRealtime, 0, 1024);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  // 底盘控制
  osThreadDef(ChassisTask, chassis_task, osPriorityAboveNormal, 0, 512);
  chassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  // 校准任务
  osThreadDef(cali, calibrate_task, osPriorityNormal, 0, 512);
  calibrate_tast_handle = osThreadCreate(osThread(cali), NULL);

  // 云台任务
  osThreadDef(gimbalTask, gimbal_task, osPriorityHigh, 0, 512);
  gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);

// 舵机任务 未知 : o0?
//  osThreadDef(SERVO, servo_task, osPriorityNormal, 0, 128);
//  servo_task_handle = osThreadCreate(osThread(SERVO), NULL);

  // 测试任务
  osThreadDef(UserTest_Task, usere_test_task, osPriorityNormal, 0, 1024);
  usb_task_handle = osThreadCreate(osThread(UserTest_Task), NULL);

  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_test_tack */
/**
 * @brief  Function implementing the test thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_test_tack */
__weak void test_tack(void const *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN test_tack */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END test_tack */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
