/*
 * can.h
 *
 *  Created on: Dec 13, 2025
 *      Author: lionm
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include <stdint.h>

#define VCU_STATE_FAULT 		(1 << 0)
#define VCU_STATE_IDLE2DRIVE	(1 << 1)
#define VCU_STATE_DRIVE2IDLE	(1 << 2)
#define VCU_STATE_RESET_HARD	(1 << 3)
#define VCU_STATE_RESET			(1 << 4)




void can_init();
void can_rx(uint8_t * RxData, uint8_t * failures);
void can_tx(uint8_t * TxData, uint32_t can_id);
void remote_rx();
void can_report(uint8_t msg_number);
void inverter_can_receive(uint8_t *data);
void inverter_can_fault_call();
float can2float(uint16_t raw, float range, float offset);
float can2float8(uint8_t raw, float range, float offset);
uint16_t float2can (float f, float scale, float offset);


typedef struct __attribute__((packed)) {
	uint8_t state;
	uint8_t fault;
	uint16_t error;
	uint16_t omega;
	uint16_t vbus;
} can_msg_1;

typedef struct __attribute__((packed)) {
	uint16_t i_q;
	uint16_t i_d;
	uint16_t v_q;
	uint16_t v_d;
} can_msg_2;

typedef struct __attribute__((packed)) {
	uint16_t temp_m;
	uint16_t temp_1;
	uint16_t temp_2;
	uint16_t temp_3;
} can_msg_3;

typedef struct __attribute__((packed)) {
	uint16_t state; //A, range 50, offset 0

	uint8_t current_in_limit_p; //+A, range 50, offset 0
	uint8_t current_in_limit_n; //-A, range 50, offset 0
	uint8_t current_out_limit_d; //+/- A, range 150, offset 0
	uint8_t current_out_limit_q; //+/- A, range 150, offset 0

	uint16_t command;

} inverter_send_t;

typedef struct {
    uint16_t state;
    float    current_in_limit_p;
    float    current_in_limit_n;
    float    current_out_limit_d;
    float    current_out_limit_q;
    float command;
} inverter_cmd_decoded_t;


#endif /* INC_CAN_H_ */
