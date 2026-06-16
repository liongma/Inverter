/*
 * can.h
 *
 *  Created on: Dec 13, 2025
 *      Author: lionm
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include <stdint.h>
#include "stm32h7xx_hal.h"

void can_init();
void can_rx(uint8_t * RxData, FDCAN_RxHeaderTypeDef * RxHeader);
void can_tx( uint8_t * TxData,uint64_t can_id);
float can2float(uint16_t raw, float range, float offset);
uint16_t float2can(float val, float range, float offset);
uint8_t float2can8(float val, float range, float offset);



#endif /* INC_CAN_H_ */
