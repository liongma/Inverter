/*
 * accel.h
 *
 *  Created on: Feb 25, 2026
 *      Author: lionm
 */

#ifndef INC_ACCEL_H_
#define INC_ACCEL_H_

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

extern uint8_t can_transmit[8];

// test functions
void accel_test(void);
void decel_test(void);
void full_stop_test(void);

#endif /* INC_ACCEL_H_ */
