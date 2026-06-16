/*
 * defines.h
 *
 *  Created on: Mar 1, 2026
 *      Author: lionm
 */

#ifndef INC_DEFINES_H_
#define INC_DEFINES_H_

//#define DUMMY
//#define DUMMY_TESTPWM

#define DYNO_CONTROL
//#define DYNO_CONTROL_DIRECT_THROTTLE
#define DYNO_CONTROL_CURRENT

//#define THROTTLE
//#define IGNORE_EXTERN

//#define MOTOR1
//#define MOTOR2
//#define MOTOR3
#define MOTOR4

#define FOC
//#define DBPCC
//#define DBPFC

//#define RS_DET
//#define LQ_DET
//#define LD_DET
//#define LQ_DET_HF

//#define PSI_DET

//#define KALMAN


// TODO: Replace IDs with actual numbers.


#ifdef MOTOR1
#define CURRENT_CONTROL
//#define SPEED_CONTROL
#define CW
#define CAN_FILTER 0x04
#define CAN_ID_1 0x04
#define CAN_ID_2 0x05
#define CAN_ID_3 0x06
#define CAN_ID_RX 0x07
#endif

#ifdef MOTOR2
//#define SPEED_CONTROL
#define CURRENT_CONTROL
#define CW
#define CAN_FILTER 0x08
#define CAN_ID_1 0x08
#define CAN_ID_2 0x09
#define CAN_ID_3 0x0A
#define CAN_ID_RX 0x0B
#endif

#ifdef MOTOR3
//#define SPEED_CONTROL
#define CURRENT_CONTROL
//#define CW
#define CAN_FILTER 0x10
#define CAN_ID_1 0x10
#define CAN_ID_2 0x11
#define CAN_ID_3 0x12
#define CAN_ID_RX 0x13
#endif

#ifdef MOTOR4
//#define SPEED_CONTROL
#define CURRENT_CONTROL
//#define CW
#define CAN_FILTER 0x20
#define CAN_ID_1 0x20
#define CAN_ID_2 0x21
#define CAN_ID_3 0x22
#define CAN_ID_RX 0x23
#endif

#define CAN_ID_ALL 0x3C

//#else
//	#define CAN_ID 0x24
//
//	//#define SPEED_CONTROL
//	//#define CURRENT_SWEEP
//	#define CURRENT_CONTROL
//	//#define HF_INJECTION
//
//#endif


//#define CURRENT_FILTER

//#define SERIAL_PRINT
#define CAN

#define DYNO_CURRENT 10


#define CAN_FREQ 100
#define CAN_PERIOD 10000/CAN_FREQ

#endif /* INC_DEFINES_H_ */
