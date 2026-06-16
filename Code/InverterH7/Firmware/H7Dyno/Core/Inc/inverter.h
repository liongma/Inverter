/*
 * inverter.h
 *
 *  Created on: May 9, 2026
 *      Author: lionm
 */

#ifndef INC_INVERTER_H_
#define INC_INVERTER_H_

#include "stdint.h"

typedef enum {
    MOTOR1 = 0,
    MOTOR2 = 1,
    MOTOR3 = 2,
    MOTOR4 = 3,
    NUM_MOTORS
} motor_id_t;


typedef struct __attribute__((packed)) {
	uint8_t state;
	uint8_t fault;
	uint16_t error;
    uint16_t omega;   // electrical speed
    uint16_t vbus;    // DC bus voltage
} can_msg_1_t;

typedef struct __attribute__((packed)) {
	uint16_t i_q;
	uint16_t i_d;
	uint16_t v_q;
	uint16_t v_d;
} can_msg_2_t;


typedef struct __attribute__((packed)) {
    uint16_t temp_m;  // motor temp
    uint16_t temp_1;  // board temp 0
    uint16_t temp_2;  // board temp 1
    uint16_t temp_3;  // board temp 2
} can_msg_3_t;

typedef struct __attribute__((packed)) {
	uint16_t state; //A, range 50, offset 0

	uint8_t current_in_limit_p; //+A, range 50, offset 0
	uint8_t current_in_limit_n; //-A, range 50, offset 0
	uint8_t current_out_limit_d; //+/- A, range 150, offset 0
	uint8_t current_out_limit_q; //+/- A, range 150, offset 0

	uint16_t command;

} inverter_send_t;



typedef struct {
    motor_id_t motor;

    uint16_t   state;
    float      current_in_limit_p;
    float      current_in_limit_n;
    float      current_out_limit_d;
    float      current_out_limit_q;
    float 	   command;
} inverter_cmd_t;

typedef struct {
    // --- MSG 1 ---
	uint8_t state;
	uint8_t fault;
	uint16_t error;

    float    omega_e;    // rad/s electrical, range 12000, offset 0
    float    v_bus;      // V,      range 700,   offset 0

    // --- MSG 2 ---
    float    v_q;        // V,      range 800,   offset 400
    float    v_d;        // V,      range 800,   offset 400
    float    i_q;        // A,      range 300,   offset 150
    float    i_d;        // A,      range 300,   offset 150

    // --- MSG 3 ---
    float    temp_motor; // °C,     range 150,   offset 0
    float    temp_board[3]; // °C,  range 150,   offset 0

    // --- Status flags ---
    uint8_t  msg1_valid;
    uint8_t  msg2_valid;
    uint8_t  msg3_valid;
} inverter_data_t;


extern inverter_data_t inverters[4];
extern inverter_send_t tx_frames[4];

int vcu_decode_inverter_can(uint16_t can_id, const uint8_t *data);

#define VCU_STATE_FAULT 		(1 << 0)
#define VCU_STATE_IDLE2DRIVE	(1 << 1)
#define VCU_STATE_DRIVE2IDLE	(1 << 2)
#define VCU_STATE_RESET_HARD	(1 << 3)
#define VCU_STATE_RESET			(1 << 4)

void vcu_transmit_inverter_can(const inverter_cmd_t *cmd);
void can_process(void);

#endif /* INC_INVERTER_H_ */
