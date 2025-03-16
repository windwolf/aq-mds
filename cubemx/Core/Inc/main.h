/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define ADC1_IN2_OPP1_Pin GPIO_PIN_2
#define ADC1_IN2_OPP1_GPIO_Port GPIOA
#define ADC1_IN3_OPP2_Pin GPIO_PIN_3
#define ADC1_IN3_OPP2_GPIO_Port GPIOA
#define ADC1_IN8_NTC_Pin GPIO_PIN_0
#define ADC1_IN8_NTC_GPIO_Port GPIOB
#define ADC1_IN9_OCP_Pin GPIO_PIN_1
#define ADC1_IN9_OCP_GPIO_Port GPIOB
#define TIM1_CH2_PWM_Pin GPIO_PIN_3
#define TIM1_CH2_PWM_GPIO_Port GPIOB
#define GPI_OTP_Pin GPIO_PIN_5
#define GPI_OTP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
