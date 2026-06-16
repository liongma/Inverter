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
#include "stm32h7xx_ll_cordic.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_dma.h"

#include "stm32h7xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "defines.h"
#include "appMath.h"
#include "adc.h"
#include "hall.h"
#include "encoder.h"
#include "math.h"
#include "lut.h"
#include "mtpva.h"
#include "can.h"
#include "off_param.h"
#include "fault.h"
#include "off_detect.h"
#include "speed.h"
#include "warn.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;

extern FDCAN_HandleTypeDef hfdcan1;

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim16;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;

extern char msg[400];
extern uint16_t iu;
extern uint16_t iv;
extern uint16_t iw;


extern uint32_t motor;

extern uint16_t hallOffset[3];
extern uint16_t hallValue[3];

extern uint32_t adcOffset[10];
extern uint32_t adcValue[10];
extern uint32_t adc2_value[10];
extern uint32_t adc3_value[10];

extern float AMK_temp;
extern float board_temp[3];


extern float i_phase[3];
extern uint32_t tI;
extern float freq;
extern double Ts;

extern float theta;
extern float thetaI;
extern float omega;

extern float v_bus;
extern float power;
extern float theta_e;
extern float theta_e_1;
extern float omega_e;
extern float omega_e_1;
extern float accel_e;
extern float acl_e;
extern float accel_e_1;
extern float jerk_e;
extern float jrk_e;

extern float theta_e_est;

extern DQ_Components integral;
extern DQ_Components error;
extern float integral_w;
extern float error_w;

extern DQ_Components ref;
extern DQ_Components ref_aw;

extern DQ_Components v;
extern DQ_Components v_sat;
extern DQ_Components aw;

extern const uint8_t au8PermuataionMatrix[6][3];

extern int8_t can_transmit[8];
extern uint8_t can_recieve[8];
extern FDCAN_RxHeaderTypeDef RxHeader;


extern float ref_w;
extern int32_t pwm_max;
extern float cmd;
extern uint32_t dt;
extern float af;
extern uint32_t current_time;

extern DQ_Components error_i; // V/A
extern DQ_Components dError; // V/A

extern float error_w_i;

extern float vmag;
extern float imag;

extern float i_sat;
extern float i_set;

extern float psi_c;

extern DQ_Components Kp;
extern DQ_Components Ki;
extern DQ_Components Kd;

extern DQ_Components If;
extern DQ_Components dI;


extern DQ_Components i_dq;

extern DQ_Components f1;
extern DQ_Components f2;

extern AlphaBeta_Components v_static;

extern DQ_Components i_dq_k1_hat;
extern DQ_Components est_error;

extern DQ_Components integral_est;
extern float test_time;
extern float test_angle_1;
extern float test_angle;
extern float w_e;
extern int8_t w_sign;
extern float Rs_det;
extern float Ld_det;
extern float Lq_det;
extern float Psi_det;
extern float dV_det;

extern uint8_t inverter_state;
extern uint8_t inverter_fault;
extern uint16_t inverter_error;

extern can_msg_1 can_1;
extern can_msg_2 can_2;
extern can_msg_3 can_3;


extern float cmd_d;
extern float cmd_q;

extern float flux_q;
extern float flux_d;

extern uint8_t can_rx_fail;

extern inverter_cmd_decoded_t vcu_cmd;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define V2PWM PWM_MAX/v_bus
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
#define VSENSE__Pin GPIO_PIN_0
#define VSENSE__GPIO_Port GPIOC
#define VSENSE_C1_Pin GPIO_PIN_1
#define VSENSE_C1_GPIO_Port GPIOC
#define IU__Pin GPIO_PIN_0
#define IU__GPIO_Port GPIOA
#define IU_A1_Pin GPIO_PIN_1
#define IU_A1_GPIO_Port GPIOA
#define IV__Pin GPIO_PIN_4
#define IV__GPIO_Port GPIOC
#define IV_C5_Pin GPIO_PIN_5
#define IV_C5_GPIO_Port GPIOC
#define IW__Pin GPIO_PIN_0
#define IW__GPIO_Port GPIOB
#define IW_B1_Pin GPIO_PIN_1
#define IW_B1_GPIO_Port GPIOB
#define EN_DAT_Pin GPIO_PIN_2
#define EN_DAT_GPIO_Port GPIOB
#define OE_Pin GPIO_PIN_14
#define OE_GPIO_Port GPIOE
#define PROBE_Pin GPIO_PIN_13
#define PROBE_GPIO_Port GPIOB
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
#define UPHASE TIM1->CCR1
#define VPHASE TIM1->CCR2
#define WPHASE TIM1->CCR3
#define ENABLE_OUT LL_GPIO_SetOutputPin(GPIOE, GPIO_PIN_14);
#define DISABLE_OUT LL_GPIO_ResetOutputPin(GPIOE, GPIO_PIN_14);
#define IDLE_OUT TIM1->BDTR &= ~TIM_BDTR_MOE;
#define ACTIVE_OUT TIM1->BDTR |= TIM_BDTR_MOE;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
