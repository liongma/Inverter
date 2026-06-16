/*
 * can.c
 *
 *  Created on: Dec 13, 2025
 *      Author: lionm
 */


#include "can.h"
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;





void can_init(){
	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID; // vs. Extended ID
	sFilterConfig.FilterIndex = 0; // the number of the filter we are using
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // what to do filter match
	sFilterConfig.FilterID1 = CAN_FILTER; // filter
	sFilterConfig.FilterID2 = CAN_FILTER; // mask: 0=>accept all messages
	HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
	    FDCAN_REJECT,         // reject non-matching standard frames
	    FDCAN_REJECT,         // reject non-matching extended frames
	    FDCAN_REJECT_REMOTE,  // reject remote standard frames
	    FDCAN_REJECT_REMOTE   // reject remote extended frames
	);


	HAL_FDCAN_Start(&hfdcan1);



}

void can_tx_buffer(uint8_t * TxData, uint32_t can_id, uint32_t BufferIndex){
	FDCAN_TxHeaderTypeDef TxHeader;
	TxHeader.Identifier = can_id;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	if(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1, BufferIndex) == 1) {
		Error_Handler(); // busy processing other request
	}

	HAL_FDCAN_AddMessageToTxBuffer(&hfdcan1, &TxHeader, TxData, BufferIndex);
	HAL_FDCAN_EnableTxBufferRequest(&hfdcan1, BufferIndex);
}


void can_tx(uint8_t * TxData, uint32_t can_id){
	FDCAN_TxHeaderTypeDef TxHeader;
	TxHeader.Identifier = can_id;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
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

}

void can_rx(uint8_t * RxData, uint8_t * failures){
	if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
		*failures = 0;
	}
	else{
		*failures += 0;
	}

}


void remote_rx(){

#ifdef CURRENT_CONTROL
	#ifdef DYNO_CONTROL_DIRECT_THROTTLE
		cmd = vcu_cmd.command;
	#elif defined(DYNO_CONTROL_CURRENT)
		cmd = vcu_cmd.command;
		if(fabs(vcu_cmd.command) < 0.001) cmd = 0;
		cmd_q = cmd*150.0;
		cmd_d = -0.1 * cmd_q;
		if(RxHeader.Identifier == CAN_ID_ALL){
			fault_manual();
		}
	#endif
#endif

}

void can_report(uint8_t msg_number) {

	if(msg_number == 0){
		can_1.state = inverter_state;
		can_1.fault = inverter_fault;
		can_1.error = inverter_error;
		can_1.omega = float2can(fabs(omega_e), 12000,0);
		can_1.vbus = float2can(v_bus, 700,0);
//		can_tx((uint8_t*) &can_1, CAN_ID_1);
		can_tx_buffer((uint8_t*) &can_1, CAN_ID_1,FDCAN_TX_BUFFER0);
	}
	if(msg_number == 1){
//		can_msg_2
		can_2.v_q = float2can(v.q, 800, 400.0f);
		can_2.v_d = float2can(v.d, 800, 400.0f);
		can_2.i_q = float2can(If.q, 300.0, 150.0f);
		can_2.i_d = float2can(If.d, 300.0, 150.0f);
//		can_tx((uint8_t*) &can_2, CAN_ID_2);
		can_tx_buffer((uint8_t*) &can_2, CAN_ID_2,FDCAN_TX_BUFFER1);
	}
	if(msg_number == 2){
		can_3.temp_m = float2can(AMK_temp, 150,0);
		can_3.temp_1 = float2can(board_temp[0], 150,0);
		can_3.temp_2 = float2can(board_temp[1], 150,0);
		can_3.temp_3 = float2can(board_temp[2], 150,0);
//		can_tx((uint8_t*) &can_3, CAN_ID_3);
		can_tx_buffer((uint8_t*) &can_3, CAN_ID_3,FDCAN_TX_BUFFER2);
	}
}



void inverter_can_receive(uint8_t *data) {

    const inverter_send_t *rx = (const inverter_send_t *)data;

    vcu_cmd.state               = rx->state;
    vcu_cmd.current_in_limit_p  = can2float8(rx->current_in_limit_p,   50.0f,  0.0f);
    vcu_cmd.current_in_limit_n  = can2float8(rx->current_in_limit_n,   50.0f,  0.0f);
    vcu_cmd.current_out_limit_d = can2float8(rx->current_out_limit_d, 150.0f,  0.0f);
    vcu_cmd.current_out_limit_q = can2float8(rx->current_out_limit_q, 150.0f,  0.0f);
    vcu_cmd.command             = can2float(rx->command, 2.0f, 1.0f);

}

void inverter_can_fault_call() {
    inverter_send_t tx;

    tx.state               = 0xA0 | VCU_STATE_FAULT;
    tx.current_in_limit_p  = 0;
    tx.current_in_limit_n  = 0;
    tx.current_out_limit_d = 0;
    tx.current_out_limit_q = 0;
    tx.command             = 0;

    can_tx_buffer((uint8_t *)(&tx), CAN_ID_ALL,FDCAN_TX_BUFFER4);
}



float can2float(uint16_t raw, float range, float offset) {
    return ((float)raw / 65535.0f) * range - offset;
}

float can2float8(uint8_t raw, float range, float offset) {
    return ((float)raw / 255.0f) * range - offset;
}

uint16_t float2can(float val, float range, float offset) {
    float normalized = (val + offset) / range;

    // Clamp to [0, 1] to prevent overflow
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;

    return (uint16_t)(normalized * 65535.0f);
}






