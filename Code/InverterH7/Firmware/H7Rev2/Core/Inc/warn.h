/*
 * warn.h
 *
 *  Created on: May 9, 2026
 *      Author: lionm
 */

#ifndef INC_WARN_H_
#define INC_WARN_H_

void warn_if_overvolt(void);
void warn_if_undervolt(void);
void warn_if_voltage_drop(void);
void warn_if_overtemp_board(void);
void warn_if_undertemp_board(void);
void warn_if_overtemp_motor(void);
void warn_if_undertemp_motor(void);
void warn_if_lowpower(void);


#endif /* INC_WARN_H_ */
