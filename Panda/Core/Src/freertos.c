/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
/*'''

„Ä?„Ä?‚îè‚îì„Ä?„Ä?„Ä?‚îè‚îì+ +
„Ä?‚îè‚îõ‚îª‚îÅ‚îÅ‚îÅ‚îõ‚îª‚î? + +
„Ä?‚îÉ„??„Ä?„Ä?„Ä?„Ä?„Ä?„Ä?‚î? „Ä?
„Ä?‚îÉ„??„Ä?„Ä?‚îÅ„??„Ä?„Ä?‚î? ++ + + +
 ‚ñà‚ñà‚ñà‚ñà‚îÅ‚ñà‚ñà‚ñà‚ñ? ‚î?+
„Ä?‚îÉ„??„Ä?„Ä?„Ä?„Ä?„Ä?„Ä?‚î? +
„Ä?‚îÉ„??„Ä?„Ä?‚îª„??„Ä?„Ä?‚î?
„Ä?‚îÉ„??„Ä?„Ä?„Ä?„Ä?„Ä?„Ä?‚î? + +
„Ä?‚îó‚îÅ‚îì„??„Ä?„Ä?‚îè‚îÅ‚î?
„Ä?„Ä?„Ä?‚îÉ„??„Ä?„Ä?‚îÉ„??„Ä?„Ä?„Ä?„Ä?„Ä?„Ä?„Ä?„Ä?„Ä?„Ä?
„Ä?„Ä?„Ä?‚îÉ„??„Ä?„Ä?‚î? + + + +
„Ä?„Ä?„Ä?‚îÉ„??„Ä?„Ä?‚î?
„Ä?„Ä?„Ä?‚îÉ„??„Ä?„Ä?‚î? +  Á•ûÂÖΩ‰øù‰Ωë
„Ä?„Ä?„Ä?‚îÉ„??„Ä?„Ä?‚î?    ‰ª£Á†ÅÊó†bug„Ä?„Ä?
„Ä?„Ä?„Ä?‚îÉ„??„Ä?„Ä?‚îÉ„??„Ä?+„Ä?„Ä?„Ä?„Ä?„Ä?„Ä?„Ä?„Ä?„Ä?
„Ä?„Ä?„Ä?‚îÉ„?? „Ä?„Ä?‚îó‚îÅ‚îÅ‚îÅ‚î? + +
„Ä?„Ä?„Ä?‚î? „Ä?„Ä?„Ä?„Ä?„Ä?„Ä?„Ä?‚î£‚îì
„Ä?„Ä?„Ä?‚î? „Ä?„Ä?„Ä?„Ä?„Ä?„Ä?„Ä?‚îè‚îõ
„Ä?„Ä?„Ä?‚îó‚îì‚îì‚îè‚îÅ‚î≥‚îì‚îè‚î? + + + +
„Ä?„Ä?„Ä?„Ä?‚îÉ‚î´‚î´„??‚îÉ‚î´‚î?
„Ä?„Ä?„Ä?„Ä?‚îó‚îª‚îõ„??‚îó‚îª‚î?+ + + +
'''*/
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

/* USER CODE END Variables */
osThreadId REMOTE_TASKHandle;
osThreadId CHASSIS_TASKHandle;
osThreadId LED_FLOW_TASKHandle;
osThreadId DETECT_TASKHandle;
osThreadId SERVO_TASKHandle;
osThreadId WAYPOINT_TASKHandle;
osThreadId LIFT_TASKHandle;
osThreadId Paw_TaskHandle;
osThreadId IMU_TASKHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void remote_task(void const * argument);
void chassis_task(void const * argument);
void led_flow_task(void const * argument);
void detect_task(void const * argument);
void servo_task(void const * argument);
void Waypoint_task(void const * argument);
void lift_task(void const * argument);
void PAW_Task(void const * argument);
void imu_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of REMOTE_TASK */
  osThreadDef(REMOTE_TASK, remote_task, osPriorityNormal, 0, 256);
  REMOTE_TASKHandle = osThreadCreate(osThread(REMOTE_TASK), NULL);

  /* definition and creation of CHASSIS_TASK */
  osThreadDef(CHASSIS_TASK, chassis_task, osPriorityNormal, 0, 128);
  CHASSIS_TASKHandle = osThreadCreate(osThread(CHASSIS_TASK), NULL);

  /* definition and creation of LED_FLOW_TASK */
  osThreadDef(LED_FLOW_TASK, led_flow_task, osPriorityNormal, 0, 128);
  LED_FLOW_TASKHandle = osThreadCreate(osThread(LED_FLOW_TASK), NULL);

  /* definition and creation of DETECT_TASK */
  osThreadDef(DETECT_TASK, detect_task, osPriorityNormal, 0, 128);
  DETECT_TASKHandle = osThreadCreate(osThread(DETECT_TASK), NULL);

  /* definition and creation of SERVO_TASK */
  osThreadDef(SERVO_TASK, servo_task, osPriorityNormal, 0, 128);
  SERVO_TASKHandle = osThreadCreate(osThread(SERVO_TASK), NULL);

  /* definition and creation of WAYPOINT_TASK */
  osThreadDef(WAYPOINT_TASK, Waypoint_task, osPriorityNormal, 0, 128);
  WAYPOINT_TASKHandle = osThreadCreate(osThread(WAYPOINT_TASK), NULL);

  /* definition and creation of LIFT_TASK */
  osThreadDef(LIFT_TASK, lift_task, osPriorityNormal, 0, 128);
  LIFT_TASKHandle = osThreadCreate(osThread(LIFT_TASK), NULL);

  /* definition and creation of Paw_Task */
  osThreadDef(Paw_Task, PAW_Task, osPriorityNormal, 0, 128);
  Paw_TaskHandle = osThreadCreate(osThread(Paw_Task), NULL);

  /* definition and creation of IMU_TASK */
  osThreadDef(IMU_TASK, imu_task, osPriorityNormal, 0, 128);
  IMU_TASKHandle = osThreadCreate(osThread(IMU_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_remote_task */
/**
* @brief Function implementing the REMOTE_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_remote_task */
__weak void remote_task(void const * argument)
{
  /* USER CODE BEGIN remote_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END remote_task */
}

/* USER CODE BEGIN Header_chassis_task */
/**
* @brief Function implementing the Chassis_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_task */
__weak void chassis_task(void const * argument)
{
  /* USER CODE BEGIN chassis_task */
  /* Infinite loop */
  for(;;)
  {

  }
  /* USER CODE END chassis_task */
}

/* USER CODE BEGIN Header_led_flow_task */
/**
* @brief Function implementing the LED_FLOW_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_flow_task */
__weak void led_flow_task(void const * argument)
{
  /* USER CODE BEGIN led_flow_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END led_flow_task */
}

/* USER CODE BEGIN Header_detect_task */
/**
* @brief Function implementing the DETECT_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_detect_task */
__weak void detect_task(void const * argument)
{
  /* USER CODE BEGIN detect_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END detect_task */
}

/* USER CODE BEGIN Header_servo_task */
/**
* @brief Function implementing the SERVO_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servo_task */
__weak void servo_task(void const * argument)
{
  /* USER CODE BEGIN servo_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END servo_task */
}

/* USER CODE BEGIN Header_Waypoint_task */
/**
* @brief Function implementing the WAYPOINT_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Waypoint_task */
__weak void Waypoint_task(void const * argument)
{
  /* USER CODE BEGIN Waypoint_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Waypoint_task */
}

/* USER CODE BEGIN Header_lift_task */
/**
* @brief Function implementing the LIFT_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lift_task */
__weak void lift_task(void const * argument)
{
  /* USER CODE BEGIN lift_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END lift_task */
}

/* USER CODE BEGIN Header_PAW_Task */
/**
* @brief Function implementing the Paw_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PAW_Task */
__weak void PAW_Task(void const * argument)
{
  /* USER CODE BEGIN PAW_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END PAW_Task */
}

/* USER CODE BEGIN Header_imu_task */
/**
* @brief Function implementing the IMU_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_task */
__weak void imu_task(void const * argument)
{
  /* USER CODE BEGIN imu_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END imu_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
