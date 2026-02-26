/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    gpio.h
 * @brief   This file contains all the function prototypes for
 *          the gpio.c file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C"
{
#endif

  /* Includes ------------------------------------------------------------------*/
  /* USER CODE BEGIN Includes */
#include "stm32g4xx_hal.h"

  /* USER CODE END Includes */

  /* USER CODE BEGIN Private defines */
#define M_TIM_ARR 8400
#define LED_RUN_Pin GPIO_PIN_14
#define LED_RUN_GPIO_Port GPIOC
#define LED01_Pin GPIO_PIN_15
#define LED01_GPIO_Port GPIOC
  /* USER CODE END Private defines */

  void MX_GPIO_Init(void);

  /* USER CODE BEGIN Prototypes */

  /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */
