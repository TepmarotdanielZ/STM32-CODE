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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

uint32_t last_time = 0;
#define MAX_LED 8
#define USE_BRIGHTNESS 0


uint8_t LED_Data[MAX_LED][8];
uint8_t LED_Mod[MAX_LED][8];  /* FOR BRIGHTNESS */
uint16_t effStep = 0;
uint32_t K[3] = {0, 4, 8};
int GREEN[3] = {255, 0, 0};
int BLUE[3] = {0, 255, 0};
int RED[3] = {0, 0, 255};
int L = 4, M = 8,z;
int datasentflag=0;
int RGB_Blue = 0;
int RGB_Red = 255;
int RGB_Green =0;
int RGB_state = 0;
int RGB_Brightness = 255;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	datasentflag=1;
}

void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}

#define PI 3.14159265

void Set_Brightness (int brightness)  // 0-45
{
#if USE_BRIGHTNESS

	if (brightness > 45) brightness = 45;
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j=1; j<4; j++)
		{
			float angle = 90-brightness;  // in degrees
			angle = angle*PI / 180;  // in rad
			LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
		}
	}

#endif

}

uint16_t pwmData[(24*MAX_LED)+50];

void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
#if USE_BRIGHTNESS
		color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
#else
		color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
#endif

		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				pwmData[indx] = 70;  // 2/3 of 105/70
			}

			else pwmData[indx] = 35;  // 1/3 of 105/37

			indx++;
		}

	}

	for (int i=0; i<50; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
	while (!datasentflag){};
	datasentflag = 0;
}

void Reset_LED (void)
{
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Data[i][0] = i;
		LED_Data[i][1] = 0;
		LED_Data[i][2] = 0;
		LED_Data[i][3] = 0;
	}
}

double map(double x, double in_min, double in_max, double out_min,
		double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void RGB_Speed(){
	// if(HAL_GetTick() - last_time >= map(RxData[0],0,255,20,250)){
	  	last_time = HAL_GetTick();

	  	for(int i = 0;i<=2; i++){
			K[i]++;
			if(K[i] == 12){
				K[i]=0;
				Set_LED(K[i], RED[i], GREEN[i], BLUE[i]);
				Set_Brightness(0);
				Set_LED(11, 0, 0, 0);

				WS2812_Send();
			}
			if(K[i]>=1 && K[i]<=11){
				Set_LED(K[i],RED[i], GREEN[i], BLUE[i]);
				Set_LED(K[i]-1, 0, 0, 0);
				WS2812_Send();
			}
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
  MX_DMA_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  	  Reset_LED();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* RGB */

	  if(HAL_GetTick() - last_time >= 30 ){
	  	last_time = HAL_GetTick();

	  	if (RGB_state == 0){
		  	RGB_Red = RGB_Brightness;
		  	RGB_Green ++;
		  	RGB_Blue = 0;
		  	if(RGB_Green >= RGB_Brightness){
		  		RGB_state = 1;
		  	}
	  	}
	  	else if (RGB_state == 1){
		  	RGB_Red --;
		  	RGB_Green = RGB_Brightness;
		  	RGB_Blue = 0;
		  	if(RGB_Red <= 0){
		  		RGB_state = 2;
		  	}
	  	}
	  	else if (RGB_state == 2){
		  	RGB_Red = 0;
		  	RGB_Green = RGB_Brightness;
		  	RGB_Blue ++;
		  	if(RGB_Blue >= RGB_Brightness){
		  		RGB_state = 3;
		  	}
	  	}
	  	else if (RGB_state == 3){
		  	RGB_Red = 0;
		  	RGB_Green --;
		  	RGB_Blue = RGB_Brightness;
		  	if(RGB_Green <= 0){
		  		RGB_state = 4;
		  	}
	  	}
	  	else if (RGB_state == 4){
		  	RGB_Red ++;
		  	RGB_Green = 0;
		  	RGB_Blue = RGB_Brightness;
		  	if(RGB_Red >= RGB_Brightness){
		  		RGB_state = 5;
		  	}	  	}
	  	else if (RGB_state == 5){
		  	RGB_Red = RGB_Brightness;
		  	RGB_Green = 0;
		  	RGB_Blue --;
		  	if(RGB_Blue <= 0){
		  		RGB_state = 0;
		  	}	  	}

	  	else{

	  	}

	  }
		Set_LED(0, RGB_Red, RGB_Green, RGB_Blue);
		Set_LED(1, RGB_Red, RGB_Green, RGB_Blue);
		Set_LED(2, RGB_Red, RGB_Green, RGB_Blue);
		Set_LED(3, RGB_Red, RGB_Green, RGB_Blue);
		Set_LED(4, RGB_Red, RGB_Green, RGB_Blue);
		Set_LED(5, RGB_Red, RGB_Green, RGB_Blue);
		Set_LED(6, RGB_Red, RGB_Green, RGB_Blue);
		Set_LED(7, RGB_Red, RGB_Green, RGB_Blue);
		WS2812_Send();


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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
