/*
 * fault.h
 *
 *  Created on: Feb 28, 2026
 *      Author: lionm
 */

#ifndef INC_FAULT_H_
#define INC_FAULT_H_

#include "main.h"


void fault_init(void);
void fault_buffer(void);
void fault_dump_output(uint8_t fault);

void fault_if_undervolt(void);
void fault_if_overvolt(void);
void fault_if_overcurrent(void);
void fault_if_low_power(void);
void fault_if_slow(void);
void fault_if_encoder(void);
uint8_t fault_if_overspeed(void);
uint8_t fault_if_overaccel(void);
uint8_t fault_if_overjerk(void);
void fault_if_over_predict(void);
void fault_if_voltage_drop(void);
void fault_if_diverge(void);
void fault_if_extern(void);
void fault_manual(void);
void fault_if_can_dead(void);
void fault_drive_voltage_drop(void);

void error_if_overvolt(void);
void error_if_undervolt(void);
void error_if_voltage_drop(void);
void error_if_overtemp(void);
void error_if_undertemp(void);
void error_if_lowpower(void);




/*
 *
 * FAULTS:
 * OVERVOLT
 * OVERCURRENT
 * SLOW
 * ENCODER
 * OVERPREDICT
 * VOLTAGE_DROP
 * CONTROLS_DIVERGE
 * EXTERNAL_FAULT
 * ERRORS:
 *
 * UNDERVOLT
 * VOLTAGE_DROP
 * OVERTEMP
 * UNDERTEMP
 * ENCODER
 * LOW_POWER
 *
 * */

#endif /* INC_FAULT_H_ */
