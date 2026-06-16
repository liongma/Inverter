/*
 * adc.h
 *
 *  Created on: Aug 13, 2025
 *      Author: LiongMa
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"
//#define ADC2V 0.3002470588 //0.3 for torque
#define ADC2V 0.4002470588 //0.3 for torque

#define ADC2I 0.03968796992
float pollRef(uint32_t * value);
void callibrateADC(ADC_HandleTypeDef *hadc, uint32_t * value, uint32_t * offset);
#endif /* INC_ADC_H_ */
