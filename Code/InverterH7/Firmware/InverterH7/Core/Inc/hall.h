/*
 * adc.h
 *
 *  Created on: Aug 12, 2025
 *      Author: LiongMa
 */

#ifndef INC_HALL_H_
#define INC_HALL_H_

#include "main.h"

#define CNT2AMP 0.0322265625

HAL_StatusTypeDef SPI_ReadData(SPI_HandleTypeDef *hspi, uint16_t *pData, uint16_t Size, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void pollHall(SPI_HandleTypeDef *hspi, uint16_t pData[]);
void callibrateHall(SPI_HandleTypeDef *hspi, uint16_t offset[]);
void pollCurrent(SPI_HandleTypeDef *hspi, float i_phase[],uint16_t pData[], uint16_t offset[]);

#endif /* INC_HALL_H_ */
