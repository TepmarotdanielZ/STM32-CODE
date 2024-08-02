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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

	/* BEBOUNCE 1 */

		/*
			uint32_t previousMillis[2] = {0};
			uint32_t currentMillis[2]  = {0};
			uint32_t counterOutside[2] = {0}; //For testing only
			uint32_t counterInside[2]  = {0}; //For testing only

		*/


		/*

			uint32_t previousMillis[2] = {0};
			uint32_t currentMillis[2] = {0};
			uint32_t debounceDelay = 50; // Adjust debounce delay as needed

			volatile uint32_t counter[2] = {0}; // For testing only

		*/


		/*

			uint32_t previousMillis[2] = {0};
			uint32_t currentMillis[2] = {0};
			uint32_t counterOutside[2] = {0}; // For testing only
			uint32_t counterInside[2] = {0};

		*/


		/*

			#define DEBOUNCE_DELAY 10

			uint32_t previousMillis[2] = {0};
			uint32_t currentMillis[2] = {0};
			uint32_t counterButton1 = 0;
			uint32_t counterButton2 = 0;

		*/

	     /* BEBOUNCE V6 */

			#define BUTTON1_PIN GPIO_PIN_10
			#define BUTTON1_PORT GPIOA
			#define BUTTON2_PIN GPIO_PIN_11
			#define BUTTON2_PORT GPIOA


			#define DEBOUNCE_DELAY 200

			uint32_t previousMillis = 0;
			uint32_t currentMillis = 0;
			volatile uint32_t count = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */

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

	/* BEBOUNCE V1 */

		/*

			void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
			{
			  counterOutside[0]++; //For testing only
			  currentMillis[0] = HAL_GetTick();
			  if (GPIO_Pin == GPIO_PIN_8 && (currentMillis[0] - previousMillis[0] > 10))
			  {
				counterInside[0]++; //For testing only
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
				previousMillis[0] = currentMillis[0];
			  }

			  counterOutside[1]--; //For testing only
			  currentMillis[1] = HAL_GetTick();
			  if (GPIO_Pin == GPIO_PIN_8 && (currentMillis[1] - previousMillis[1] < 10))
			  {
				counterInside[1]--; //For testing only
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
				previousMillis[1] = currentMillis[1];
			  }
			}

		*/


	/* BEBOUNCE V2 */

	/*

		void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
		{
			if (GPIO_Pin == GPIO_PIN_10) // Button 1
			{
				currentMillis[0] = HAL_GetTick();
				if (currentMillis[0] - previousMillis[0] >= debounceDelay)
				{
					if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET)
					{
						counter[0]++; // Increment count when Button 1 is pressed
					}
					else
					{
						counter[0]--; // Decrement count when Button 1 is released
					}
					previousMillis[0] = currentMillis[0];
				}
			}

			if (GPIO_Pin == GPIO_PIN_11) // Button 2
			{
				currentMillis[1] = HAL_GetTick();
				if (currentMillis[1] - previousMillis[1] >= debounceDelay)
				{
					if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET)
					{
						counter[1]++; // Increment count when Button 2 is pressed
					}
					else
					{
						counter[1]--; // Decrement count when Button 2 is released
					}
					previousMillis[1] = currentMillis[1];
				}
			}
		}


	*/

  /* BEBOUNCE V3 */

	/*

		void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
			if (HAL_GetTick() - previousMillis[0] >= debounceDelay && GPIO_Pin == BUTTON1_PIN) {
				previousMillis[0] = HAL_GetTick();
				counterOutside[0]++;
			}

			if (HAL_GetTick() - previousMillis[1] >= debounceDelay && GPIO_Pin == BUTTON2_PIN) {
				previousMillis[1] = HAL_GetTick();
				counterInside[1]--;
			}
		}


	*/


  /* BEBOUNCE V4 */

	/*

		void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
		{
		    // Button 1
		    if (GPIO_Pin == GPIO_PIN_10)
		    {
		        if (HAL_GetTick() - previousMillis[0] > DEBOUNCE_DELAY)
		        {
		            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET)
		            {
		                counterButton1++;
		            }
		            else
		            {
		                counterButton1--;
		            }
		            HAL_Delay(50); // Debounce delay
		            previousMillis[0] = HAL_GetTick();
		        }
		    }

		    // Button 2
		    if (GPIO_Pin == GPIO_PIN_11)
		    {
		        if (HAL_GetTick() - previousMillis[1] > DEBOUNCE_DELAY)
		        {
		            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET)
		            {
		                counterButton2++;
		            }
		            else
		            {
		                counterButton2--;
		            }
		            HAL_Delay(50); // Debounce delay
		            previousMillis[1] = HAL_GetTick();
		        }
		    }
		}

	*/


  /* BEBOUNCE V5 */

	/*

		void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
		{
			if (GPIO_Pin == GPIO_PIN_10 || GPIO_Pin == GPIO_PIN_11)
			{
				int buttonIndex = 0;
				if (GPIO_Pin == GPIO_PIN_2)
				{
					buttonIndex = 1;
				}

				if (HAL_GetTick() - previousMillis[buttonIndex] > DEBOUNCE_DELAY)
				{
					if (HAL_GPIO_ReadPin(GPIOA, GPIO_Pin) == GPIO_PIN_SET)
					{
						// Increment count
						if (GPIO_Pin == GPIO_PIN_10)
						{
							counterButton1++;
						}
						else if (GPIO_Pin == GPIO_PIN_10)
						{
							counterButton2++;
						}
					}
					else
					{
						// Decrement count
						if (GPIO_Pin == GPIO_PIN_11)
						{
							counterButton1--;
						}
						else if (GPIO_Pin == GPIO_PIN_11)
						{
							counterButton2--;
						}
					}

					// Implement the debounce delay
					HAL_Delay(50);
					previousMillis[buttonIndex] = HAL_GetTick();
				}
			}
		}

	*/

  /* BEBOUNCE V6 */

		void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
			currentMillis = HAL_GetTick();

			if (GPIO_Pin == BUTTON1_PIN && (currentMillis - previousMillis > DEBOUNCE_DELAY)) {
				count++;
				previousMillis = currentMillis;
			}

			if (GPIO_Pin == BUTTON2_PIN && (currentMillis - previousMillis > DEBOUNCE_DELAY)) {
				count--;
				previousMillis = currentMillis;
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
