/*
 * adc.c
 *
 *  Created on: Aug 12, 2025
 *      Author: LiongMa
 */

#include "hall.h"

#define U_HALL_CS_Pin GPIO_PIN_8
#define U_HALL_CS_GPIO_Port GPIOD

/*
HAL_StatusTypeDef SPI_ReadData(SPI_HandleTypeDef *hspi, uint16_t *pData, uint16_t Size, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    // For SPI, need to transmit dummy bytes (usually 0xFF or 0x00) to receive data
    uint8_t dummyTx = 0xFF;

	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0);

	if (HAL_SPI_TransmitReceive(hspi, &dummyTx, pData, Size, HAL_MAX_DELAY) != HAL_OK)
	{
		return HAL_ERROR;
	}
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 1);

    return HAL_OK;
}

void pollHall(SPI_HandleTypeDef *hspi, uint16_t pData[]){
    SPI_ReadData(hspi,&(pData[0]),1,U_HALL_CS_GPIO_Port, U_HALL_CS_Pin); //U
    SPI_ReadData(hspi,&(pData[1]),1,V_HALL_CS_GPIO_Port, V_HALL_CS_Pin); //V
    SPI_ReadData(hspi,&(pData[2]),1,W_HALL_CS_GPIO_Port, W_HALL_CS_Pin); //W
}

void callibrateHall(SPI_HandleTypeDef *hspi, uint16_t offset[])
{
    // For SPI, need to transmit dummy bytes (usually 0xFF or 0x00) to receive data
	uint16_t pData[3] = {0};
	uint32_t total[3] = {0};
	for(int i =0; i < 1000; i++){
		pollHall(hspi,pData);
		total[0] += pData[0];
		total[1] += pData[1];
		total[2] += pData[2];
	}
	offset[0] = total[0]/1000;
	offset[1] = total[1]/1000;
	offset[2] = total[2]/1000;
	//predetermined

	offset[0] = 2050;
	offset[1] = 2050;
	offset[2] = 2050;

}

void pollCurrent(SPI_HandleTypeDef *hspi, float i_phase[],uint16_t pData[], uint16_t offset[]){
    int n = 3;
	uint16_t data = 0;
	pData[0] = 0;
	pData[1] = 0;
	pData[2] = 0;
	for(int i = 0; i < n; i++){
		SPI_ReadData(hspi,&(data),1,U_HALL_CS_GPIO_Port, U_HALL_CS_Pin); //U
		pData[0] += data;
	}
	pData[0]/=n;
	for(int i = 0; i < n; i++){
		SPI_ReadData(hspi,&(data),1,V_HALL_CS_GPIO_Port, V_HALL_CS_Pin); //
		pData[1] += data;
	}
	pData[1]/=n;
	for(int i = 0; i < n; i++){
		SPI_ReadData(hspi,&(data),1,W_HALL_CS_GPIO_Port, W_HALL_CS_Pin); //U
		pData[2] += data;
	}
	pData[2]/=n;



    float split = 0.5;
    i_phase[0] = (-(pData[0] - offset[0])*CNT2AMP)*split + i_phase[0]*(1-split);
    i_phase[1] = (-(pData[1] - offset[1])*CNT2AMP)*split + i_phase[1]*(1-split);
    i_phase[2] = (-(pData[2] - offset[2])*CNT2AMP)*split + i_phase[2]*(1-split);

}



*/





