/*
 * encoder.c
 *
 *  Created on: Aug 13, 2025
 *      Author: LiongMa
 */



#include "encoder.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim16;

char msgBuffer[30];

float shortenAngle(float theta){
	if(theta > M_PI2) return - M_2PI + theta;
	else if (theta < -M_PI2) return M_2PI + theta;
	return theta;
}


void readEncoder(uint32_t * motor){

    LL_GPIO_SetOutputPin(EN_DAT_GPIO_Port, EN_DAT_Pin);

	LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_10BIT);
	LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);

	uint16_t txData = 28U;
	uint16_t recieve[8] = {0};
	uint16_t dummytx[8] = {0};

	HAL_SPI_Transmit(&hspi1, (uint8_t*)&txData, 1, 100);

    LL_GPIO_ResetOutputPin(EN_DAT_GPIO_Port, EN_DAT_Pin);
    LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_16BIT);
    LL_SPI_SetTransferDirection(SPI1, LL_SPI_FULL_DUPLEX);
    LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_LSB_FIRST);

	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)dummytx, (uint8_t*)recieve, 2, 100);

	uint32_t SPI_buf = recieve[0] | (recieve[1]<<16);
//	sprintf(msgBuffer,"%x %x \n",recieve[1],recieve[0]);
	while ((SPI_buf & 1UL) == 1){
	  SPI_buf = SPI_buf >> 1;
	}
	SPI_buf = (SPI_buf >> 5) & (0xFFFFUL);
//	sprintf(msgBuffer,"%lu \n",SPI_buf);
//	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)msgBuffer, strlen(msgBuffer));
	*motor = SPI_buf + 4450;
}


void readAngleSpeed(float * angle, float * speed, float * oldAngle, uint32_t * oldTime, float * frequency){
	uint32_t endat;
	uint32_t dT = __HAL_TIM_GET_COUNTER(&htim16) - *oldTime;
	*oldTime = __HAL_TIM_GET_COUNTER(&htim16);
	*frequency = (1.0/(float)dT);
	readEncoder(&endat);
	*angle = endat*ENDAT2RAD;
	*speed = shortenAngle(*angle - *oldAngle)*(*frequency);

}


