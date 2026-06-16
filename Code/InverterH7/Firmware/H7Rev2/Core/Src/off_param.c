/*
 * off_param.c
 *
 *  Created on: Jan 21, 2026
 *      Author: lionm
 */

#include "main.h"
#include "off_param.h"
#include "appMath.h"
#include <stdint.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;

float estimate_Lq(float Iq, float Id){
	float ret = (-0.055233*Iq*Iq + -0.019069*Id*Id + -0.000247*Iq*Id + 0.188661*Iq + -1.436199*Id + 319.868321)*0.000001f;
//	float ret = (-2.311465*fabs(Iq) + -3.696990*fabs(Id) + 360)*0.000001f;
	if(ret > 0.0003) return 0.0003;
	if(ret < 0.00005) return 0.00005;

	return ret;
}

float estimate_Ld(float Iq, float Id){
	float ret = (-0.015165*Iq*Iq + -0.023356*Id*Id + -0.002252*Iq*Id + 0.115662*Iq + -2.994284*Id + 256.658539)*0.000001f;
	if(ret > 0.00040) return 0.00040;
	if(ret < 0.00012) return 0.00012;

//	float ret = (-2.485052*Id + 245)*0.000001f;
	return ret;
}
