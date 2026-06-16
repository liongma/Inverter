/*
 * inverter.c
 *
 *  Created on: May 9, 2026
 *      Author: lionm
 */

#include "inverter.h"
#include "can.h"

#define CAN_RX_BUFFER_SIZE 32  // must be power of 2

extern FDCAN_HandleTypeDef hfdcan1;


typedef struct {
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
} can_rx_frame_t;

//   can_id  : received CAN frame ID
//   data    : pointer to 8-byte CAN frame payload
//   dlc     : data length code (expected 8)


static const uint16_t motor_can_base[NUM_MOTORS] = {
    0x04,  // MOTOR1
    0x08,  // MOTOR2
    0x10,  // MOTOR3
    0x20,  // MOTOR4
};

#define CAN_ID_1_OFFSET     0
#define CAN_ID_2_OFFSET     1
#define CAN_ID_3_OFFSET     2
#define CAN_ID_RX_OFFSET    3  // TX to inverter — not decoded here

int vcu_decode_inverter_can(uint16_t can_id, const uint8_t *data) {

    for (int m = 0; m < NUM_MOTORS; m++) {
        uint16_t base = motor_can_base[m];

        if (can_id == base + CAN_ID_1_OFFSET) {
            const can_msg_1_t *msg = (const can_msg_1_t *)data;
            inverters[m].state = msg->state;
            inverters[m].fault = msg->fault;
            inverters[m].error = msg->error;
            inverters[m].omega_e        = can2float(msg->omega, 12000.0f, 0.0f);
            inverters[m].v_bus          = can2float(msg->vbus,    700.0f, 0.0f);
            inverters[m].msg1_valid     = 1;
            return 1;
        }

        if (can_id == base + CAN_ID_2_OFFSET) {
            const can_msg_2_t *msg = (const can_msg_2_t *)data;
            inverters[m].v_q        = can2float(msg->v_q, 800.0f, 400.0f);
            inverters[m].v_d        = can2float(msg->v_d, 800.0f, 400.0f);
            inverters[m].i_q        = can2float(msg->i_q, 300.0f, 150.0f);
            inverters[m].i_d        = can2float(msg->i_d, 300.0f, 150.0f);
            inverters[m].msg2_valid = 1;
            return 1;
        }

        if (can_id == base + CAN_ID_3_OFFSET) {
            const can_msg_3_t *msg = (const can_msg_3_t *)data;
            inverters[m].temp_motor    = can2float(msg->temp_m, 150.0f, 0.0f);
            inverters[m].temp_board[0] = can2float(msg->temp_1, 150.0f, 0.0f);
            inverters[m].temp_board[1] = can2float(msg->temp_2, 150.0f, 0.0f);
            inverters[m].temp_board[2] = can2float(msg->temp_3, 150.0f, 0.0f);
            inverters[m].msg3_valid    = 1;
            return 1;
        }
    }

    return 0; // not an inverter frame
}
inverter_send_t tx_frames[NUM_MOTORS];

void vcu_transmit_inverter_can(const inverter_cmd_t *cmd) {
    inverter_send_t *tx = &tx_frames[cmd->motor];

    tx->state               = cmd->state;
    tx->current_in_limit_p  = float2can8(cmd->current_in_limit_p,  50.0f,  0.0f);
    tx->current_in_limit_n  = float2can8(cmd->current_in_limit_n,  50.0f,  0.0f);
    tx->current_out_limit_d = float2can8(cmd->current_out_limit_d, 150.0f, 0.0f);
    tx->current_out_limit_q = float2can8(cmd->current_out_limit_q, 150.0f, 0.0f);
    tx->command             = float2can(cmd->command,2.0f,1.0f);

    can_tx((uint8_t *)tx, motor_can_base[cmd->motor]+CAN_ID_RX_OFFSET);
}

void can_process(void) {
    can_rx_frame_t frame;

    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0,
                                   &frame.header, frame.data) == HAL_OK) {
        	vcu_decode_inverter_can(frame.header.Identifier, frame.data);
        }
    }
}

