/*
 * warn.c
 *
 *  Created on: May 9, 2026
 *      Author: lionm
 */


#include "warn.h"
#include "main.h"

#define ERROR_OVERVOLT 		(1 << 8)
#define ERROR_UNDERVOLT 	(1 << 9)
#define ERROR_VOLTAGEDROP 	(1 << 10)
#define ERROR_OVERTEMP_BOARD 		(1 << 11)
#define ERROR_UNDERTEMP_BOARD 	(1 << 12)
#define ERROR_OVERTEMP_MOTOR 	(1 << 13)
#define ERROR_UNDERTEMP_MOTOR 	(1 << 14)
#define ERROR_LOWPOWER 		(1 << 15)



extern float dv_bus_dt;


void warn_if_overvolt(){
	if(vcu_cmd.state & ERROR_OVERVOLT){
		inverter_error &= ~ERROR_OVERVOLT;
	}

	if (v_bus > 605){
		inverter_error |= ERROR_OVERVOLT;
	}
}
void warn_if_undervolt(){
	if(vcu_cmd.state & ERROR_UNDERVOLT){
		inverter_error &= ~ERROR_UNDERVOLT;
	}
	if (v_bus < 350){
		inverter_error |= ERROR_UNDERVOLT;
	}
}
void warn_if_voltage_drop(){
	if(vcu_cmd.state & ERROR_VOLTAGEDROP){
		inverter_error &= ~ERROR_VOLTAGEDROP;
	}
	if (fabs(dv_bus_dt) > 0.02*v_bus){
		inverter_error |= ERROR_VOLTAGEDROP;
	}
}

void warn_if_overtemp_board(){
	if(vcu_cmd.state & ERROR_OVERTEMP_BOARD){
		inverter_error &= ~ERROR_OVERTEMP_BOARD;
	}
	if (board_temp[0] + board_temp[1] + board_temp[2] > 150){
		inverter_error |= ERROR_OVERTEMP_BOARD;
	}
}
void warn_if_undertemp_board(){
	if(vcu_cmd.state & ERROR_UNDERTEMP_BOARD){
		inverter_error &= ~ERROR_UNDERTEMP_BOARD;
	}
	if (board_temp[0] + board_temp[1] + board_temp[2] < 10){
		inverter_error |= ERROR_UNDERTEMP_BOARD;
	}
}

void warn_if_overtemp_motor(){
	if(vcu_cmd.state & ERROR_OVERTEMP_MOTOR){
		inverter_error &= ~ERROR_OVERTEMP_MOTOR;
	}
	if (AMK_temp > 50){
		inverter_error |= ERROR_OVERTEMP_MOTOR;
	}
}
void warn_if_undertemp_motor(){
	if(vcu_cmd.state & ERROR_UNDERTEMP_MOTOR){
		inverter_error &= ~ERROR_UNDERTEMP_MOTOR;
	}
	if (AMK_temp < 10){
		inverter_error |= ERROR_UNDERTEMP_MOTOR;
	}
}
void warn_if_lowpower(){
	if(vcu_cmd.state & ERROR_LOWPOWER){
		inverter_error &= ~ERROR_LOWPOWER;
	}
	if (v_bus < 10 && inverter_state > 1){
		inverter_error |= ERROR_LOWPOWER;
	}
}
