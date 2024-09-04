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
#include "stm32f4xx_hal.h"

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
#define Blue_LED_Pin GPIO_PIN_13
#define Blue_LED_GPIO_Port GPIOC
#define UART_State_Pin GPIO_PIN_2
#define UART_State_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define Custom_Pin_2_Pin GPIO_PIN_1
#define Custom_Pin_2_GPIO_Port GPIOB
#define Custom_Pin_3_Pin GPIO_PIN_2
#define Custom_Pin_3_GPIO_Port GPIOB
#define Buzzer1_Pin GPIO_PIN_6
#define Buzzer1_GPIO_Port GPIOC
#define Buzzer2_Pin GPIO_PIN_7
#define Buzzer2_GPIO_Port GPIOC
#define UART5_State_Pin GPIO_PIN_11
#define UART5_State_GPIO_Port GPIOC
#define Error_LED_Pin GPIO_PIN_8
#define Error_LED_GPIO_Port GPIOB
#define Green_LED_Pin GPIO_PIN_9
#define Green_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
