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
#include "stm32h7xx_hal.h"

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
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOE
#define EN_DAT_Pin GPIO_PIN_4
#define EN_DAT_GPIO_Port GPIOA
#define U_OC_Pin GPIO_PIN_0
#define U_OC_GPIO_Port GPIOB
#define V_OC_Pin GPIO_PIN_1
#define V_OC_GPIO_Port GPIOB
#define W_OC_Pin GPIO_PIN_2
#define W_OC_GPIO_Port GPIOB
#define OE_Pin GPIO_PIN_14
#define OE_GPIO_Port GPIOE
#define U_HALL_CS_Pin GPIO_PIN_8
#define U_HALL_CS_GPIO_Port GPIOD
#define V_HALL_CS_Pin GPIO_PIN_9
#define V_HALL_CS_GPIO_Port GPIOD
#define W_HALL_CS_Pin GPIO_PIN_10
#define W_HALL_CS_GPIO_Port GPIOD
#define RGB_Pin GPIO_PIN_15
#define RGB_GPIO_Port GPIOA
#define SD_DETECT_Pin GPIO_PIN_0
#define SD_DETECT_GPIO_Port GPIOD
#define FAULT_Pin GPIO_PIN_1
#define FAULT_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
