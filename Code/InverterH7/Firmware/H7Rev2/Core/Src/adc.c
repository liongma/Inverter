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

float amk_temp_1 = 0;

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

void pollAMKT(uint32_t * value, float * output){
	float temp = ((1000*value[0]/(4096.0 - value[0]) - 200)*0.167746f -73.950197f);
	if(fabs(temp - amk_temp_1) < 10){
		*output = temp*0.01 + 0.99*(*output);
	}
	amk_temp_1 = temp;
}

void pollNTC(uint32_t * value, float * output){
	float sign = 1;
	float temp = ((float)(*value))/4096.0f-0.5f;
	if(*value > 2048)sign = -1;
	*output =  sign*400*temp*temp+30;
}

//static const float A = 0.9;
//static const float B_inv = 258.24111f;
//static const float R_up = 10000.0f;
//static const float R_25 = 1000.0f;


/*	PTC: TFPT0603L1001FV
 * Curve Fit: T = 0.376599 * (ADC) - 753.59941;
 *	MOTOR4: b1: 47.803261, b2: 48.179859, b3: 85.839760
 *	MOTOR3: b1: 36.881889, b2: 38.764885, b3: 62.490623
 *  MOTOR2: b1: 50.439453, b2: 49.309658, b3: 82.826965
 *	MOTOR1: b1: 34.622295, b2: 37.258492, b3: 57.218239
 *
 */

float board_temp_1[3] = {0};


void pollPTC(uint32_t * value,uint32_t * value2, float * output){
//
//	float vin = ((float)(*value)- 1064)/4096.0f;
//	float Rt = (vin*R_up)/(1.0f-vin);
//	float T = B_inv*(Rt/R_25 - A);
//	*output = T;
//	output[0] =  0.42256496936*(float)(value[0]) -452.55;
//	output[1] =  0.42256496936*(float)(value[1]) -452.55;
//	output[2] =  0.42256496936*(float)(value[2]) -452.55;

	float temp1 = (0.376599*(float)(value[0])-753.59941 - TEMP_OFFSET1);
	float temp2 = (0.376599*(float)(value[1])-753.59941 - TEMP_OFFSET2);
	float temp3 = (0.376599*(float)(value2[1])-753.59941 - TEMP_OFFSET3);

	if(temp1 > 200 || temp1 < 5) temp1 = output[0];
	if(temp2 > 200 || temp2 < 5) temp2 = output[1];
	if(temp3 > 200 || temp3 < 5) temp3 = output[2];

	if(fabs(temp1 - board_temp_1[0]) < 2){
		output[0] = temp1*0.0001 + 0.9999*(output[0]);
	}
	board_temp_1[0] = temp1;

	if(fabs(temp2 - board_temp_1[1]) < 2){
		output[1] = temp2*0.0001 + 0.9999*(output[1]);
	}
	board_temp_1[1] = temp2;

	if(fabs(temp3 - board_temp_1[2]) < 2){
		output[2] = temp3*0.0001 + 0.9999*(output[2]);
	}
	board_temp_1[2] = temp3;


}


void pollVI(uint32_t * value, uint32_t * offset,  float * v, float * i){
	*v = ((float)((int32_t)value[0]-(int32_t)offset[0]))*ADC2V*0.2+0.8*(*v);
	if(*v < 0) *v = 0;
	i[0] = (((float)((int32_t)value[1]-(int32_t)offset[1]))*ADC2I)*1 + 0*i[0];
	i[1] = (-((float)((int32_t)value[2]-(int32_t)offset[2]))*ADC2I)*1 + 0*i[1];
	i[2] = (((float)((int32_t)value[3]-(int32_t)offset[3]))*ADC2I)*1 + 0*i[2];

}

float pollRef(uint32_t * value, uint32_t * offset){
	if(value[4] < offset[4]){
		return 0.0f;
	}
	else if(value[4] > offset[4] + 10000){
		return 1.0f;
	}
	else{
		return (float)(value[4] - offset[4])*0.0001f;
	}
}


