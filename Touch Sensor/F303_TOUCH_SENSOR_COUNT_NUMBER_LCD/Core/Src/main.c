/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "i2c-lcd.h"

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


int count = 0;


int buttonUpPressed = 0;
int buttonDownPressed = 0;
int buttonPushCounter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void delay(uint32_t ms);

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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

//	  lcd_init ();
//	  lcd_send_string ("ITC-01 Vs ITC-02");
//	  HAL_Delay(1000);
//	  lcd_put_cur(2, 2);

//  lcd_init();
//  lcd_send_string("Press Up/Down");
//  lcd_put_cur(2, 1);
//  lcd_put_cur(buttonPushCounter);

  /* Configure the LCD */
  lcd_init();
  lcd_send_string("Count: ");
  HAL_Delay(100);
  lcd_put_cur(0, 7);

  /* Configure the touch sensors as external interrupts */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  lcd_send_string(count);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	    if (buttonUpPressed)
	    {
	      buttonUpPressed = 0;
	      buttonPushCounter++;
	      lcd_clear();
	      lcd_put_cur(0, 0);
	      lcd_send_string("Count: ");
	      lcd_put_cur(0, 7);
	      lcd_send_data(buttonPushCounter + '0');
	      delay(200);
	    }

	    if (buttonDownPressed)
	    {
	      buttonDownPressed = 0;
	      buttonPushCounter--;
	      lcd_clear();
	      lcd_put_cur(0, 0);
	      lcd_send_string("Count: ");
	      lcd_put_cur(0, 7);
	      lcd_send_data(buttonPushCounter + '0' );
	      delay(200);
	    }



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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

	/* TOUCH SENSOR EXIT INTERRUPT */

//		void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//		{
//		  if (GPIO_Pin == GPIO_PIN_2)
//		  {
//			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
//
//		  }
//		  else{
//
//		  }
//
//		}

///////////////////////////////////////////////////////////

//		void EXTI1_IRQHandler(void)
//		{
////		  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
//		}

//		void EXTI2_IRQHandler(void)
//		{
//		  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
//		}
//
//		void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//		{
//		  if (GPIO_Pin == GPIO_PIN_1)
//		  {
//		    buttonUpPressed = 1;
//		    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
//		  }
//		  else if (GPIO_Pin == GPIO_PIN_2)
//		  {
//		    buttonDownPressed = 1;
//		    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
//		  }
//		}
//
//		void delay(uint32_t ms)
//		{
//		  HAL_Delay(ms);
//		}






///////////////////////////////////////////////////////////


//	// EXTI1 interrupt handler
//	void EXTI1_IRQHandler(void)
//	{
//	  // Clear the EXTI line pending bit
//	  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
//
//	  // Increment count
//	  count++;
//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
//	}
//
//	// EXTI2 interrupt handler
//	void EXTI2_IRQHandler(void)
//	{
//	  // Clear the EXTI line pending bit
//	  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
//
//	  // Decrement count (with a check to avoid negative values)
//	  if (count > 0)
//	  {
//		count--;
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
//	  }
//	}


////////////////////////////////////////////////////////////////////

	void delay(uint32_t ms)
	{
	  HAL_Delay(ms);
	}

	void EXTI1_IRQHandler(void)
	{
	  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
	}

	void EXTI2_IRQHandler(void)
	{
	  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
	}

	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
	  if (GPIO_Pin == GPIO_PIN_1)
	  {
		buttonUpPressed = 1;
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	  }
	  else if (GPIO_Pin == GPIO_PIN_2)
	  {
		buttonDownPressed = 1;
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
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
