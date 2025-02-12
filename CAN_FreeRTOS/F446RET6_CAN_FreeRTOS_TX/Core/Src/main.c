/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bno055_stm32.h"
#include "bno055.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//	/* EULER */
//
//		bno055_vector_t E;
//
//	/* QUATERNION */
//
//		bno055_vector_t Q;
//
//	/* ACCELEROMETER */
//
//		bno055_vector_t A;
//
//	/* MAGNETORMETER */
//
//		bno055_vector_t M;
//
//	/* CALIBRATION STATE */
//
//		bno055_vector_t C;
//
//	 /* GRAVITY */
//
//		bno055_vector_t Gra;
//
//	 /* GYROSCOPE */
//
//		bno055_vector_t Gyr;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t TxMailbox;

int datacheck = 0;

float pwm_M1 = 0 ;
float pwm_M2 = 0 ;

float RxData1 = 0;
float RxData2 = 0;
float RxData3 = 0;
float RxData4 = 0;

float V1 = 0;
float V2 = 0;
float V3 = 0;
float V4 = 0;

uint16_t V1_out = 0;
uint16_t V2_out = 0;
uint16_t V3_out = 0;
uint16_t V4_out = 0;

uint8_t flag = 0;
uint8_t cntt;
uint8_t cnt;

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if (GPIO_Pin == GPIO_PIN_13)
//	{
//		TxData[0] = 100;   // ms Delay
//		TxData[1] = 40;    // loop rep
//
//		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
//	}
//}
//
//
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
//	if (RxHeader.DLC == 2)
//	{
//		datacheck = 1;
//	}
//}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* MAP */

float map(float Input, float Min_Input, float Max_Input, float Min_Output, float Max_Output) {
	return (float) ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
}


/* STM32 INTERUPP RECEIVER FROM USB CAN */

	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
		cntt++;
		while (cntt - 100 > 0) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			cntt = 0;
		}

	/* DATA RECIEVER FROM USB CAN */

		if (RxHeader.DLC == 8) {
		RxData1 = (RxData[0] << 8) | RxData[1];
		RxData2 = (RxData[2] << 8) | RxData[3];
		RxData3 = (RxData[4] << 8) | RxData[5];
		RxData4 = (RxData[6] << 8) | RxData[7];

	/* TRANFER TO SPEED */

		/* M MOTOR */

			V1 = RxData1;
			V2 = RxData2;
			V3 = RxData3;
			V4 = RxData4;

			/* MAP TX DATA FROM 8BIT TO 16BIT (MOTOR ) */

				/* MAPPING MOTOR */
//
//					M1_out = map(M1, 0, 65535, -64, 64);
//					M2_out = map(M2, 0, 65535, -64, 64);
//
//					flag = 1;
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

	  HAL_CAN_Start(&hcan1);

	  // Activate the notification
	  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


	  TxHeader.DLC = 8;  // data length
	  TxHeader.IDE = CAN_ID_STD;
	  TxHeader.RTR = CAN_RTR_DATA;
	  TxHeader.StdId = 0x446;  // ID

	  HAL_TIM_Base_Start_IT(&htim2);

	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	  V1_out = 65535;
	  V2_out = 65535;
	  V3_out = 65535;
	  V4_out = 65535;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */




  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		TIM1->CCR1 = V1;
		TIM1->CCR2 = V2;
		TIM1->CCR3 = V3;

	/* TX DATA SEND TO PC MAP 16BIT TO 8BIT COZ DATA CAN 1DATA=8BIT */

		TxData[0] = ((V1_out & 0xFF00) >> 8);
		TxData[1] = (V1_out & 0x00FF);
		TxData[2] = ((V2_out & 0xFF00) >> 8);
		TxData[3] = (V2_out & 0x00FF);
		TxData[4] = ((V3_out & 0xFF00) >> 8);
		TxData[5] = (V3_out & 0x00FF);
		TxData[6] = ((V4_out & 0xFF00) >> 8);
		TxData[7] = (V4_out & 0x00FF);

		if (!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))) {
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

		}
	}

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
