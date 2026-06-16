/*
 * encoder.h
 *
 *  Created on: Aug 13, 2025
 *      Author: LiongMa
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_


#define ENDAT2RAD 0.00009587379



#include "main.h"
#include "appMath.h"
#include "string.h"
#include "stdio.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_spi.h"


#define ENDAT_DIR_WRITE HAL_GPIO_WritePin(EN_DAT_GPIO_Port, EN_DAT_Pin,1)
#define ENDAT_DIR_Read HAL_GPIO_WritePin(EN_DAT_GPIO_Port, EN_DAT_Pin,0)

float shortenAngle(float theta);
void readEncoder(uint32_t * motor);

#endif /* INC_ENCODER_H_ */
