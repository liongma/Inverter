/*
 * adc.h
 *
 *  Created on: Aug 13, 2025
 *      Author: LiongMa
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"
#define ADC2V 0.07663865545

void callibrateADC(ADC_HandleTypeDef *hadc, uint32_t * offset);

void pollVoltage(ADC_HandleTypeDef *hadc, float * voltage, uint32_t * ADC, uint32_t * offset);
#endif /* INC_ADC_H_ */
