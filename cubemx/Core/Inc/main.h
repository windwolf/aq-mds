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
#define ADC1_IN0_OVP_Pin GPIO_PIN_0
#define ADC1_IN0_OVP_GPIO_Port GPIOA
#define GPO_PWEN_Pin GPIO_PIN_1
#define GPO_PWEN_GPIO_Port GPIOA
#define GPI_N_PWSW_Pin GPIO_PIN_2
#define GPI_N_PWSW_GPIO_Port GPIOA
#define GPO_LED1_Pin GPIO_PIN_3
#define GPO_LED1_GPIO_Port GPIOA
#define GPO_LED2_Pin GPIO_PIN_4
#define GPO_LED2_GPIO_Port GPIOA
#define GPO_LED3_Pin GPIO_PIN_5
#define GPO_LED3_GPIO_Port GPIOA
#define GPO_LED4_Pin GPIO_PIN_6
#define GPO_LED4_GPIO_Port GPIOA
#define GPO_LED5_Pin GPIO_PIN_7
#define GPO_LED5_GPIO_Port GPIOA
#define GPO_LED6_Pin GPIO_PIN_0
#define GPO_LED6_GPIO_Port GPIOB
#define GPO_LED7_Pin GPIO_PIN_8
#define GPO_LED7_GPIO_Port GPIOA
#define GPI_MTON_Pin GPIO_PIN_6
#define GPI_MTON_GPIO_Port GPIOC
#define ADC1_IN15_OCP_Pin GPIO_PIN_11
#define ADC1_IN15_OCP_GPIO_Port GPIOA
#define GPI_N_LTSW_Pin GPIO_PIN_15
#define GPI_N_LTSW_GPIO_Port GPIOA
#define TIM1_CH2_PWM_Pin GPIO_PIN_3
#define TIM1_CH2_PWM_GPIO_Port GPIOB
#define GPO_LTEN_Pin GPIO_PIN_6
#define GPO_LTEN_GPIO_Port GPIOB
#define ADC1_IN11_NTC_Pin GPIO_PIN_7
#define ADC1_IN11_NTC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
