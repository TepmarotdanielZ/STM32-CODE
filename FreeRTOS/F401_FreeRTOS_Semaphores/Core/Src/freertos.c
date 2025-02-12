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

#include "usart.h"

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
osThreadId LOWHandle;
osThreadId NORMALHandle;
osThreadId HIGHHandle;
osSemaphoreId BinSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void TaskLow(void const * argument);
void TaskNormal(void const * argument);
void TaskHigh(void const * argument);

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

  /* Create the semaphores(s) */
  /* definition and creation of BinSem */
  osSemaphoreDef(BinSem);
  BinSemHandle = osSemaphoreCreate(osSemaphore(BinSem), 1);

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
  /* definition and creation of LOW */
  osThreadDef(LOW, TaskLow, osPriorityBelowNormal, 0, 128);
  LOWHandle = osThreadCreate(osThread(LOW), NULL);

  /* definition and creation of NORMAL */
  osThreadDef(NORMAL, TaskNormal, osPriorityNormal, 0, 128);
  NORMALHandle = osThreadCreate(osThread(NORMAL), NULL);

  /* definition and creation of HIGH */
  osThreadDef(HIGH, TaskHigh, osPriorityAboveNormal, 0, 128);
  HIGHHandle = osThreadCreate(osThread(HIGH), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_TaskLow */
/**
  * @brief  Function implementing the LOW thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_TaskLow */
void TaskLow(void const * argument)
{
  /* USER CODE BEGIN TaskLow */
  /* Infinite loop */
  for(;;)
  {
//		char *str1 = "\nENTER LOW TASK\n";
//		HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen (str1), 100);
//
//
//		char *str2 = "\nLEAVING LOW TASK\n";
//		HAL_UART_Transmit(&huart2, (uint8_t *) str2, strlen (str2), 100);
//		osDelay(500);

		char *str1 = "Entered LOWTask and waiting for semaphore\n";
		HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen (str1), 100);

		osSemaphoreWait(BinSemHandle, osWaitForever);

		char *str3 = "Semaphore acquired by LOW Task\n";
		HAL_UART_Transmit(&huart2, (uint8_t *) str3, strlen (str3), 100);

		while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));  // wait till the pin go low

		char *str2 = "Leaving LOWTask and releasing Semaphore\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *) str2, strlen (str2), 100);

		osSemaphoreRelease(BinSemHandle);
		osDelay(500);

  }
  /* USER CODE END TaskLow */
}

/* USER CODE BEGIN Header_TaskNormal */
/**
* @brief Function implementing the NORMAL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskNormal */
void TaskNormal(void const * argument)
{
  /* USER CODE BEGIN TaskNormal */
  /* Infinite loop */
  for(;;)
  {

//		char *str1 = "\nENTER NORMOAL TASK\n";
//		HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen (str1), 100);
//
//
//		char *str2 = "\nLEAVING NORMOAL TASK\n";
//		HAL_UART_Transmit(&huart2, (uint8_t *) str2, strlen (str2), 100);
//		osDelay(500);



		char *str1 = "Entered MediumTask\n";
		HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen (str1), 100);


		char *str2 = "Leaving MediumTask\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *) str2, strlen (str2), 100);
		osDelay(500);


  }
  /* USER CODE END TaskNormal */
}

/* USER CODE BEGIN Header_TaskHigh */
/**
* @brief Function implementing the HIGH thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskHigh */
void TaskHigh(void const * argument)
{
  /* USER CODE BEGIN TaskHigh */
  /* Infinite loop */
  for(;;)
  {
//		char *str1 = "\nENTER HIGH TASK\n";
//		HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen (str1), 100);
//
//
//		char *str2 = "\nLEAVING HIGH TASK\n";
//		HAL_UART_Transmit(&huart2, (uint8_t *) str2, strlen (str2), 100);
//		osDelay(500);

		char *str1 = "Entered HighTask and waiting for Semaphore\n";
		HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen (str1), 100);

		osSemaphoreWait(BinSemHandle, osWaitForever);

		char *str3 = "Semaphore acquired by HIGH Task\n";
		HAL_UART_Transmit(&huart2, (uint8_t *) str3, strlen (str3), 100);

		char *str2 = "Leaving HighTask and releasing Semaphore\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *) str2, strlen (str2), 100);

		osSemaphoreRelease(BinSemHandle);
	    osDelay(500);

  }
  /* USER CODE END TaskHigh */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
