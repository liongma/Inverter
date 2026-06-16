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



void callibrateADC(ADC_HandleTypeDef *hadc, uint32_t * value, uint32_t * offset){

	uint32_t total =0;
	for(int j = 0; j < 5; j++){
		total = 0;
		for(int i = 0; i < 100;i++){
			total += value[j];
			HAL_Delay(1);
		}
		offset[j] = total/100;
	}
	//offsets: 29469, 32368, 34303, 34850
}



float pollRef(uint32_t * value){
	uint32_t offest = 200;
	uint32_t range = 600;
	if(value[0] < offest){
		return 0.0f;
	}
	else if(value[0] > offest + range){
		return 1.0f;
	}
	else{
		return (float)(value[0] - offest)/range;
	}
}


