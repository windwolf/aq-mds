/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPO_PWEN_Pin|GPO_LED1_Pin|GPO_LED2_Pin|GPO_LED3_Pin
                          |GPO_LED4_Pin|GPO_LED5_Pin|GPO_LED7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPO_LED6_Pin|GPO_LTEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPO_PWEN_Pin */
  GPIO_InitStruct.Pin = GPO_PWEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPO_PWEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPI_N_PWSW_Pin */
  GPIO_InitStruct.Pin = GPI_N_PWSW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPI_N_PWSW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPO_LED1_Pin GPO_LED2_Pin GPO_LED3_Pin GPO_LED4_Pin
                           GPO_LED5_Pin GPO_LED7_Pin */
  GPIO_InitStruct.Pin = GPO_LED1_Pin|GPO_LED2_Pin|GPO_LED3_Pin|GPO_LED4_Pin
                          |GPO_LED5_Pin|GPO_LED7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPO_LED6_Pin */
  GPIO_InitStruct.Pin = GPO_LED6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPO_LED6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPI_MTON_Pin */
  GPIO_InitStruct.Pin = GPI_MTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPI_MTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPI_N_LTSW_Pin */
  GPIO_InitStruct.Pin = GPI_N_LTSW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPI_N_LTSW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPO_LTEN_Pin */
  GPIO_InitStruct.Pin = GPO_LTEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPO_LTEN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
