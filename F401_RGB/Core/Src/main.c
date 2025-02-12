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
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

	#define NEOPIXEL_ZERO 32  /* ZERO=(ARR+1)0.32 */
	#define NEOPIXEL_ONE 64   /* ONE=(ARR+1)0.64 */
	#define NUM_PIXELS 32     /* Number of LEDs  model WS2812b */
	#define DMA_BUFF_SIZE 769 /* (NUM_PIXELS*24)+1 */



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

	 typedef union
	  {
		struct
		{
		  uint8_t b;
		  uint8_t r;
		  uint8_t g;
		} color;
		uint32_t data;
	  } PixelRGB_t;

	  typedef union
	  {
		struct
		{
		  uint8_t w;
		  uint8_t b;
		  uint8_t r;
		  uint8_t g;
		} color;
		uint32_t data;
	  }
		PixelRGBW_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

    PixelRGB_t pixel[NUM_PIXELS] = {0};
    uint32_t dmaBuffer[DMA_BUFF_SIZE] = {0};
    uint32_t *pBuff;
    int n, j, k;
    uint16_t stepSize;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

	void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
	{
		/*

	  	  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_2);

 	 	*/

 	 HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_4);
	}

	void RGB_Setup(){
		for (n = (NUM_PIXELS - 1); n > 0; n--)
			{
				  pixel[n].data = pixel[n-1].data;
			}

				  if (k<255)
					  {
						pixel[0].color.g =255;    // [254, 0]
						pixel[0].color.r =255-k;  // [1, 255]
						pixel[0].color.b =0;
						j++;
					  }
				  else if (k<510){
					  pixel[0].color.g =255;      // [254, 0]
					  pixel[0].color.r =0;  	  // [1, 255]
					  pixel[0].color.b =k-254;
					  j++;
				  }
				  else if(k<765){
					  pixel[0].color.g =765-k;     // [254, 0]
					  pixel[0].color.r =0;         // [1, 255]
					  pixel[0].color.b =255;
					  j++;
				  }
				  else if(k<1020){

					  pixel[0].color.g =0;		   // [254, 0]
					  pixel[0].color.r =k-764;     // [1, 255]
					  pixel[0].color.b =255;
					  j++;

				  }
				  else if(k<1275){

						  pixel[0].color.g =0;     // [254, 0]
						  pixel[0].color.r =255;   // [1, 255]
						  pixel[0].color.b =1275-k;
						  j++;
					  }
				  else if(k<1530){

					  pixel[0].color.g =k-1274; 	//[254, 0]
					  pixel[0].color.r =255;  		//[1, 255]
					  pixel[0].color.b =0;
					  j++;
						  }
						k = (k + stepSize) % 1530;

					/* NOT SO BRIGHT */

						pixel[0].color.g >>= 1;
						pixel[0].color.r >>= 1;
						pixel[0].color.b >>= 1;

						pBuff = dmaBuffer;
							  for (n = 0; n < NUM_PIXELS; n++)
							  {
								 for (j = 23; j >= 0; j--)
								 {
								   if ((pixel[n].data >> j) & 0x01)
								   {
									 *pBuff = NEOPIXEL_ONE;
								   }
								   else
								   {
									 *pBuff = NEOPIXEL_ZERO;
								   }
								   pBuff++;
								 }
							  }


							/*
							 	 HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_2, dmaBuffer, DMA_BUFF_SIZE);
							 	 HAL_Delay(10);

							 */


								 HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, dmaBuffer, DMA_BUFF_SIZE);
								 HAL_Delay(10);
	}



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
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

		HAL_TIM_Base_Start(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

		  k = 0;
		  stepSize = 4;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		  RGB_Setup();


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
