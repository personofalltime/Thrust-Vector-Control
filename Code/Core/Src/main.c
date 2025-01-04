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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
unsigned long lastTime;
double InputX, OutputX, SetpointX;
double InputY, OutputY, SetpointY;
double errSumX, lastErrX;
double errSumY, lastErrY;
double kp, ki, kd;
int SampleTime = 10;
double currentX;
double currentY;
int ticks1, ticks2, ticks3, ticks4;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	readGyro();
	computeX();
	computeY();



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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M4Fwd_Pin|M4Rev_Pin|M2Rev_Pin|M2Fwd_Pin
                          |M1Rev_Pin|M1Fwd_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, M3Rev_Pin|M3Fwd_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M4Fwd_Pin M4Rev_Pin M2Rev_Pin M2Fwd_Pin
                           M1Rev_Pin M1Fwd_Pin */
  GPIO_InitStruct.Pin = M4Fwd_Pin|M4Rev_Pin|M2Rev_Pin|M2Fwd_Pin
                          |M1Rev_Pin|M1Fwd_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : A4_Pin */
  GPIO_InitStruct.Pin = A4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(A4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B4_Interrupt_Pin */
  GPIO_InitStruct.Pin = B4_Interrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B4_Interrupt_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M3Rev_Pin M3Fwd_Pin */
  GPIO_InitStruct.Pin = M3Rev_Pin|M3Fwd_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : A3_Pin A2_Pin A1_Pin */
  GPIO_InitStruct.Pin = A3_Pin|A2_Pin|A1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : B3_Interrupt_Pin B2_Interrupt_Pin B1_Interrupt_Pin */
  GPIO_InitStruct.Pin = B3_Interrupt_Pin|B2_Interrupt_Pin|B1_Interrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */

	if(GPIO_Pin == B1_Interrupt_Pin){
		if(HAL_GPIO_ReadPin(GPIOA, A1_Pin) == GPIO_PIN_SET){
			ticks1 += 1;
		}
		else{
			ticks1 -= 1;
		}
	}
	else if(GPIO_Pin == B2_Interrupt_Pin){
		if(HAL_GPIO_ReadPin(GPIOA, A2_Pin) == GPIO_PIN_SET){
			ticks2 += 1;
		}
		else{
			ticks2 -= 1;
		}
	}
	else if(GPIO_Pin == B3_Interrupt_Pin){
		if(HAL_GPIO_ReadPin(GPIOA, A3_Pin) == GPIO_PIN_SET){
			ticks3 += 1;
		}
		else{
			ticks3 -= 1;
		}
	}
	else if(GPIO_Pin == B4_Interrupt_Pin){
		if(HAL_GPIO_ReadPin(GPIOC, A4_Pin) == GPIO_PIN_SET){
			ticks4 += 1;
		}
		else{
			ticks4 -= 1;
		}
	}
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}

void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}

void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

void ComputeX()
{
   unsigned long now = HAL_GetTick();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = SetpointX - InputX;
      errSumX += error;
      double dErrX = (error - lastErrX);

      /*Compute PID Output*/
      OutputX = kp * error + ki * errSumX + kd * dErrX;

      /*Remember some variables for next time*/
      lastErrX = error;
      lastTime = now;

      moveToX1Tick(angleToTicks(OutputX));
      moveToX2Tick(angleToTicks(OutputX));
   }
}

void ComputeY()
{
   unsigned long now = HAL_GetTick();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = SetpointY - InputY;
      errSumY += error;
      double dErrY = (error - lastErrY);

      /*Compute PID Output*/
      OutputY = kp * error + ki * errSumY + kd * dErrY;

      /*Remember some variables for next time*/
      lastErrY = error;
      lastTime = now;

      moveToY1Tick(angleToTicks(OutputY));
      moveToY2Tick(angleToTicks(OutputY));
   }
}

void drive(int mot, bool fwd){
	switch(mot){
	case 1:
		if(fwd){
			HAL_GPIO_WritePin(GPIOB, M1Fwd_Pin, GPIO_PIN_SET);
			HAL_Delay(10);
			HAL_GPIO_WritePin(GPIOB, M1Fwd_Pin, GPIO_PIN_RESET);
			return;
		}
		else{
			HAL_GPIO_WritePin(GPIOB, M1Rev_Pin, GPIO_PIN_SET);
			HAL_Delay(10);
			HAL_GPIO_WritePin(GPIOB, M1Rev_Pin, GPIO_PIN_RESET);
			return;
		}
		break;
	case 2:
		if(fwd){
			HAL_GPIO_WritePin(GPIOB, M2Fwd_Pin, GPIO_PIN_SET);
			HAL_Delay(10);
			HAL_GPIO_WritePin(GPIOB, M2Fwd_Pin, GPIO_PIN_RESET);
			return;
		}
		else{
			HAL_GPIO_WritePin(GPIOB, M2Rev_Pin, GPIO_PIN_SET);
			HAL_Delay(10);
			HAL_GPIO_WritePin(GPIOB, M2Rev_Pin, GPIO_PIN_RESET);
			return;
		}
		break;
	case 3:
		if(fwd){
			HAL_GPIO_WritePin(GPIOC, M3Fwd_Pin, GPIO_PIN_SET);
			HAL_Delay(10);
			HAL_GPIO_WritePin(GPIOC, M3Fwd_Pin, GPIO_PIN_RESET);
			return;
		}
		else{
			HAL_GPIO_WritePin(GPIOC, M3Rev_Pin, GPIO_PIN_SET);
			HAL_Delay(10);
			HAL_GPIO_WritePin(GPIOC, M3Rev_Pin, GPIO_PIN_RESET);
			return;
		}
		break;
	case 4:
		if(fwd){
			HAL_GPIO_WritePin(GPIOB, M4Fwd_Pin, GPIO_PIN_SET);
			HAL_Delay(10);
			HAL_GPIO_WritePin(GPIOB, M4Fwd_Pin, GPIO_PIN_RESET);
			return;
		}
		else{
			HAL_GPIO_WritePin(GPIOB, M4Rev_Pin, GPIO_PIN_SET);
			HAL_Delay(10);
			HAL_GPIO_WritePin(GPIOB, M4Rev_Pin, GPIO_PIN_RESET);
			return;
		}
		break;
	}
}

void moveToX1Tick(double set){
	if(ticks1 == set){
		return;
	}
	while(ticks1 < set){
		drive(1, true);
	}
	while(ticks1 > set){
		drive(1, false);
	}
}

void moveToX2Tick(double set){
	if(ticks2 == set){
		return;
	}
	while(ticks2 < set){
		drive(2, true);
	}
	while(ticks2 > set){
		drive(2, false);
	}
}

void moveToY1Tick(double set){
	if(ticks3 == set){
		return;
	}
	while(ticks3 < set){
		drive(3, true);
	}
	while(ticks3 > set){
		drive(4, false);
	}
}

void moveToY2Tick(double set){
	if(ticks4 == set){
		return;
	}
	while(ticks4 < set){
		drive(4, true);
	}
	while(ticks4 > set){
		drive(4, false);
	}
}

void readGyro(){
	/*
	 *
	 * Open register x1B, write x20 for 1000deg/s - configuration only
	 * Store current time, calculate difference from stored value at previous measurement
	 * Open register x43 to x46, read all values
	 * every 2 bytes is X, Y - divide each one by 32.8
	 * offset by +0.56
	 * add this, *elapsed time to the existing angle
	 * set inputX and inputY values
	 *
	 */
}

int angleToTicks(int maxTicks, double angle){
	return (angle/180)*maxTicks;
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
