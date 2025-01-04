/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define M4Fwd_Pin GPIO_PIN_10
#define M4Fwd_GPIO_Port GPIOB
#define M4Rev_Pin GPIO_PIN_11
#define M4Rev_GPIO_Port GPIOB
#define M2Rev_Pin GPIO_PIN_12
#define M2Rev_GPIO_Port GPIOB
#define M2Fwd_Pin GPIO_PIN_13
#define M2Fwd_GPIO_Port GPIOB
#define M1Rev_Pin GPIO_PIN_14
#define M1Rev_GPIO_Port GPIOB
#define M1Fwd_Pin GPIO_PIN_15
#define M1Fwd_GPIO_Port GPIOB
#define A4_Pin GPIO_PIN_6
#define A4_GPIO_Port GPIOC
#define B4_Interrupt_Pin GPIO_PIN_7
#define B4_Interrupt_GPIO_Port GPIOC
#define M3Rev_Pin GPIO_PIN_8
#define M3Rev_GPIO_Port GPIOC
#define M3Fwd_Pin GPIO_PIN_9
#define M3Fwd_GPIO_Port GPIOC
#define A3_Pin GPIO_PIN_8
#define A3_GPIO_Port GPIOA
#define B3_Interrupt_Pin GPIO_PIN_9
#define B3_Interrupt_GPIO_Port GPIOA
#define A2_Pin GPIO_PIN_10
#define A2_GPIO_Port GPIOA
#define B2_Interrupt_Pin GPIO_PIN_11
#define B2_Interrupt_GPIO_Port GPIOA
#define A1_Pin GPIO_PIN_12
#define A1_GPIO_Port GPIOA
#define B1_Interrupt_Pin GPIO_PIN_13
#define B1_Interrupt_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
