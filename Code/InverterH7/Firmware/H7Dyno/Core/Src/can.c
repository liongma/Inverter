/*
 * can.c
 *
 *  Created on: Dec 13, 2025
 *      Author: lionm
 */


#include "can.h"

extern FDCAN_HandleTypeDef hfdcan1;

void can_init(){
	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID; // vs. Extended ID
	sFilterConfig.FilterIndex = 0; // the number of the filter we are using
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // what to do filter match
	sFilterConfig.FilterID1 = 0; // filter
	sFilterConfig.FilterID2 = 0; // mask: 0=>accept all messages
	HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
	HAL_FDCAN_Start(&hfdcan1);


}

void can_tx(uint8_t * TxData,uint64_t can_id){
	FDCAN_TxHeaderTypeDef TxHeader;
	TxHeader.Identifier = can_id;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	if( HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0){
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData)!= HAL_OK)
		{
		Error_Handler();
		}
	}
//	 HAL_FDCAN_AddMessageToTxBuffer(&hfdcan1, &TxHeader, TxData,
//	                                                  const uint8_t *pTxData, uint32_t BufferIndex)

}

void can_rx(uint8_t * RxData, FDCAN_RxHeaderTypeDef * RxHeader){
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, RxHeader, RxData);
}

//float can2float(uint8_t upper, uint8_t lower, float scale, float offset){
//	uint16_t temp = (upper<<8) | lower;
//	float f = temp/65536.0*scale - offset;
//	return f;
//}
float can2float(uint16_t raw, float range, float offset) {
    return ((float)raw / 65535.0f) * range - offset;
}


uint16_t float2can(float val, float range, float offset) {
    float normalized = (val + offset) / range;

    // Clamp to [0, 1] to prevent overflow
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;

    return (uint16_t)(normalized * 65535.0f);
}

uint8_t float2can8(float val, float range, float offset) {
    float normalized = (val + offset) / range;
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;
    return (uint8_t)(normalized * 255.0f);
}

uint8_t decode_can(FDCAN_RxHeaderTypeDef * RxHeader, uint8_t * RxData){

	return 0;
}
