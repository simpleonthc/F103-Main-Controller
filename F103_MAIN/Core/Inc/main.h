/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HB_Pin GPIO_PIN_13
#define HB_GPIO_Port GPIOC
#define ACT2_Pin GPIO_PIN_14
#define ACT2_GPIO_Port GPIOC
#define ACT1_Pin GPIO_PIN_15
#define ACT1_GPIO_Port GPIOC
#define VBAT_Pin GPIO_PIN_0
#define VBAT_GPIO_Port GPIOA
#define A1_Pin GPIO_PIN_1
#define A1_GPIO_Port GPIOA
#define A2_Pin GPIO_PIN_2
#define A2_GPIO_Port GPIOA
#define A3_Pin GPIO_PIN_3
#define A3_GPIO_Port GPIOA
#define A4_Pin GPIO_PIN_5
#define A4_GPIO_Port GPIOA
#define A5_Pin GPIO_PIN_6
#define A5_GPIO_Port GPIOA
#define A6_Pin GPIO_PIN_7
#define A6_GPIO_Port GPIOA
#define SHUT2_Pin GPIO_PIN_5
#define SHUT2_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOB
#define IR_Pin GPIO_PIN_1
#define IR_GPIO_Port GPIOB
#define SHUT2B13_Pin GPIO_PIN_13
#define SHUT2B13_GPIO_Port GPIOB
#define SHUT1_Pin GPIO_PIN_14
#define SHUT1_GPIO_Port GPIOB
#define INT2_Pin GPIO_PIN_15
#define INT2_GPIO_Port GPIOB
#define INT1_Pin GPIO_PIN_6
#define INT1_GPIO_Port GPIOC
#define BTN1_Pin GPIO_PIN_11
#define BTN1_GPIO_Port GPIOA
#define BTN2_Pin GPIO_PIN_12
#define BTN2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
