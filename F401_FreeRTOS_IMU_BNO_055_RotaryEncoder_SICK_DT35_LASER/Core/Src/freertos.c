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

#include "bno055_stm32.h"
#include "bno055.h"
#include "adc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

	/* EXTERNAL ROTAR ENCODER */

		typedef struct{
		  long counter;
		  GPIO_PinState lastA;
		  GPIO_PinState lastB;
		  double distand ;
		  double last_en_co;
		  double angle;
		}encoder;
		encoder enc1;

	/* IMU BNO 055 */

		/* EULER */

			bno055_vector_t E;

		/* QUATERNION */

			bno055_vector_t Q;

		/* ACCELEROMETER */

			bno055_vector_t A;

		/* MAGNETORMETER */

			bno055_vector_t M;

		/* CALIBRATION STATE */

			bno055_vector_t C;

		 /* GRAVITY */

			bno055_vector_t Gra;

		 /* GYROSCOPE */

			bno055_vector_t Gyr;

		/* EXTERNAL ROTAR ENCODER */

//			encoder enc1;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

	/* ROTARY ENCODER */

		#define EPR 2400
		#define PI 3.14159
	/*
		uint16_t chA_pin;
		uint16_t chB_pin;
	*/
		#define chA_pin GPIO_PIN_9
		#define chB_pin GPIO_PIN_10
		#define chAB_gpio_port GPIOA


		#define wheel_radius_encoder 0.03 /* MATER */

		uint32_t sample_time = 0;
		int get_value = 0;

	/* SICK DT35 LASER */

		uint16_t AD_RES = 0;
		int DISTANCE;
		int CENTIMETER;
		float METER;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

	/* READ COUNT ROTARY ENCODER */

		void read_encoder(encoder *Encoder, GPIO_PinState chA, GPIO_PinState ChB){
		  if(chA != Encoder->lastA){
			Encoder->lastA = chA;
			if(chA != ChB){
			  Encoder->counter++;
			}else {
			  Encoder->counter--;
			}
		  }
		  if(ChB != Encoder->lastB){
			Encoder->lastB = ChB;
			if(ChB == chA){
			  Encoder->counter++;
			}else {
			  Encoder->counter--;
			}
		  }
		}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for myTask_IMU_BNO_ */
osThreadId_t myTask_IMU_BNO_Handle;
const osThreadAttr_t myTask_IMU_BNO__attributes = {
  .name = "myTask_IMU_BNO_",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for myTaskRotaryEnc */
osThreadId_t myTaskRotaryEncHandle;
const osThreadAttr_t myTaskRotaryEnc_attributes = {
  .name = "myTaskRotaryEnc",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for myTask_SICK_DT3 */
osThreadId_t myTask_SICK_DT3Handle;
const osThreadAttr_t myTask_SICK_DT3_attributes = {
  .name = "myTask_SICK_DT3",
  .stack_size = 1000 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void IMU_BNO_055(void *argument);
void RotaryEncoder(void *argument);
void SICK_DT35_LASER(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask_IMU_BNO_ */
  myTask_IMU_BNO_Handle = osThreadNew(IMU_BNO_055, NULL, &myTask_IMU_BNO__attributes);

  /* creation of myTaskRotaryEnc */
  myTaskRotaryEncHandle = osThreadNew(RotaryEncoder, NULL, &myTaskRotaryEnc_attributes);

  /* creation of myTask_SICK_DT3 */
  myTask_SICK_DT3Handle = osThreadNew(SICK_DT35_LASER, NULL, &myTask_SICK_DT3_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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

  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_IMU_BNO_055 */
/**
* @brief Function implementing the myTask_IMU_BNO_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_BNO_055 */
void IMU_BNO_055(void *argument)
{
  /* USER CODE BEGIN IMU_BNO_055 */
  /* Infinite loop

   	/* IMU BNO 055 */

		  bno055_assignI2C(&hi2c1);
		  bno055_setup();
		  bno055_setOperationModeNDOF();

  for(;;)
  {
	  /* IMU BNO 055 */

		  /* EULER */

			  E	  = bno055_getVectorEuler();
					HAL_Delay(5);

		  /* QUATERNION */

			  Q   = bno055_getVectorQuaternion();
					HAL_Delay(5);

		  /* ACCELEROMETER */

			  A	  = bno055_getVectorAccelerometer();
					HAL_Delay(5);

		  /* MAGNETORMETER */

			  M   = bno055_getVectorMagnetometer();
					HAL_Delay(5);

		  /* GRAVITY */

			  Gra = bno055_getVectorGravity();
					HAL_Delay(5);

		  /* GYROSCOPE */

			  Gyr = bno055_getVectorGyroscope();
					HAL_Delay(5);

				osThreadYield();
				osDelay(1);

  }
  /* USER CODE END IMU_BNO_055 */
}

/* USER CODE BEGIN Header_RotaryEncoder */
/**
* @brief Function implementing the myTaskRotaryEnc thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RotaryEncoder */
void RotaryEncoder(void *argument)
{
  /* USER CODE BEGIN RotaryEncoder */
  /* Infinite loop */
  for(;;)
  {

	  if ( HAL_GetTick() - sample_time >= 10){
		  sample_time = HAL_GetTick();
		  get_value = enc1.counter;

		  /* DISTANCE ROTARY ENCODER */

			  enc1.distand = 2 * PI * wheel_radius_encoder * get_value / EPR;
	  }

	  osThreadYield();
	  osDelay(1);

  }
  /* USER CODE END RotaryEncoder */
}

/* USER CODE BEGIN Header_SICK_DT35_LASER */
/**
* @brief Function implementing the myTask_SICK_DT3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SICK_DT35_LASER */
void SICK_DT35_LASER(void *argument)
{
  /* USER CODE BEGIN SICK_DT35_LASER */
  /* Infinite loop */

	  HAL_ADC_Start_DMA(&hadc1, &AD_RES, 1);

  for(;;)
  {

  	  /*  ADC DMA MODE */


			  /*

				  DISTANCE = (AD_RES *0.1)  + 10;   // FIRST  TEST 1

				  DISTANCE = (AD_RES *0.09) + 43;   // SECOND TEST 2

				  DISTANCE = (AD_RES *0.078) + 82;  // THREE  TEST 3


			  */


	   /*  DISTANCE USE EQUATIONS LINEAR: y = ax + b */

		  /* CM (CENTIMETER) */

		  	  CENTIMETER = (AD_RES * 0.099) + 47.5;

		  /* M (METER) */

		  	  METER = CENTIMETER / 100.0f;

		  	  osThreadYield();
		  	  osDelay(1);

  	  }

  /* USER CODE END SICK_DT35_LASER */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

