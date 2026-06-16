/*
 * adc.h
 *
 *  Created on: Aug 13, 2025
 *      Author: LiongMa
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"
#include "defines.h"

#define ADC2V 0.29544405504//0.3 for torque
//#define ADC2I 0.02685546875 //TMCS1126B6 +/- 100A
#define ADC2I 0.0406901041 //TMCS1126BF +/- 150A
/*
 *	MOTOR4: b1: 47.803261, b2: 48.179859, b3: 85.839760
 *	MOTOR3: b1: 36.881889, b2: 38.764885, b3: 62.490623
 *  MOTOR2: b1: 50.439453, b2: 49.309658, b3: 82.826965
 *	MOTOR1: b1: 34.622295, b2: 37.258492, b3: 57.218239
 */

/*
 *	MOTOR4: b1: 22.803261, b2: 23.179859, b3: 60.839760
 *	MOTOR3: b1: 11.881889, b2: 13.764885, b3: 37.490623
 *  MOTOR2: b1: 25.439453, b2: 24.309658, b3: 57.826965
 *	MOTOR1: b1: 9.622295, b2: 12.258492, b3: 32.218239
 */

#if defined(MOTOR4)
#define TEMP_OFFSET1 22.8
#define TEMP_OFFSET2 23.2
#define TEMP_OFFSET3 60.8
#elif defined(MOTOR3)
#define TEMP_OFFSET1 11.8
#define TEMP_OFFSET2 13.7
#define TEMP_OFFSET3 37.5
#elif defined(MOTOR2)
#define TEMP_OFFSET1 25.4
#define TEMP_OFFSET2 24.3
#define TEMP_OFFSET3 57.8
#elif defined(MOTOR1)
#define TEMP_OFFSET1 9.62
#define TEMP_OFFSET2 12.25
#define TEMP_OFFSET3 32.21
#else
#define TEMP_OFFSET1 0
#define TEMP_OFFSET2 0
#define TEMP_OFFSET3 0
#endif

void pollAMKT(uint32_t * value, float * output);
float pollRef(uint32_t * value, uint32_t * offset);
void pollPTC(uint32_t * value,uint32_t * value2, float * output);
void pollNTC(uint32_t * value, float * output);
void pollAMKT(uint32_t * value, float * output);

void callibrateADC(ADC_HandleTypeDef *hadc, uint32_t * value, uint32_t * offset);
void pollVI(uint32_t * value, uint32_t * offset,  float * v, float * i);
#endif /* INC_ADC_H_ */
