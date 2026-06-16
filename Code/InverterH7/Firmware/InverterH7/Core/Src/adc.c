/*
 * adc.c
 *
 *  Created on: Aug 13, 2025
 *      Author: LiongMa
 */


#include "adc.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;

extern uint32_t adcValue[10];

void pollVoltage(ADC_HandleTypeDef *hadc, float * voltage, uint32_t * ADC, uint32_t * offset){
	*voltage = (-(float)(*ADC) + (float)(*offset))*ADC2V;

}


void callibrateADC(ADC_HandleTypeDef *hadc, uint32_t * offset){

	uint32_t total =0;

	for(int i = 0; i < 100;i++){
		total += adcValue[0];
		HAL_Delay(1);
	}
	*offset = total/100;

	*offset = 33270; //experimentally determined

}
