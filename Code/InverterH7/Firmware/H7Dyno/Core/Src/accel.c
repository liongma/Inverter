/*
 * accel.c
 *
 *  Created on: Feb 25, 2026
 *      Author: lionm
 */


#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "appMath.h"
#include "adc.h"
#include "math.h"
#include "main.h"
#include "can.h"
#include "accel.h"
#include "define.h"

//#define TEST

//#define ACCEL_TEST
// #define DECCEL_TEST
// #define FULL_STOP

#ifdef TEST
float decel_rate = 0.001f;   // per ms
float accel_rate = 0.001f;   // per ms  -- increase value -> faster accleration
float accel_cmd_cap = 1.0f;  // out of 1
float test_load  = 1.0f;     // Amps
int decel_delay = 5000;      // ms
float pot_cmd = 0.0f;

// void float2can(float f, int scale = 10, int index /*upper or lower half*/) {
//     int16_t temp = f/scale*65536; //2^16
//     if (index == 0) {
//         int8_t upper = temp >>8;
//         return upper;
//     }else {
//         int8_t lower = temp & 0x00FF;
//         return lower;
//     }
//
//     float current = 0.1
//     canbus[0] = float2can(current, 100, 0);
//     canbus[1] = float2can(current, 100, 0);
//
//
//     float can2float (int8_t upper, int8_t lower, int scale) {
//         int16_t temp = (upper<<8) | lower;
//         float f = temp/65536.0*scale;
//         return f;
//     }
//     //fixed point math for operators
//     float output = can2float(canbus)
//     // int8_t upper = temp >>8;
//     // int8_t  lower = temp & 0x00FF;
// }


const float current = 0.1;

//canbus[0] = float2can(current, 100, 0);
//canbus[1] = float2can(current, 100, 1);


const float current_q = 0.f; //I_q
const float current_d = current; //I_d

// ^^^^ SENDER ^^^^^
static bool has_initialized = false;
static uint32_t last_accel_tx = 0; // self-contained time tracker

void accel_test(void) {
    uint32_t now = HAL_GetTick();
    uint32_t dt_ms = now - last_accel_tx;

    // only run every 10ms to avoid flooding the CAN bus
    if (dt_ms < 10) {
        return;
    }

    static float curr_cmd = 0.0f;
    //uint8_t* fltptr; // pointer for converting float to bytes

    // initial Load Command
    if(!has_initialized) {
        can_transmit[0] = motor2_can_id; // CS
        can_transmit[1] = 2; // command 2
        can_transmit[2] = 0; // padding 0
        can_transmit[3] = 0; // padding 0

        can_transmit[4] = float2can(current_q, 100, 0);
        can_transmit[5] = float2can(current_q, 100, 1);
        can_transmit[6] = float2can(current_d, 100, 0);
        can_transmit[7] = float2can(current_d, 100, 1);

        can_tx(can_transmit,motor1_can_id);
        has_initialized = true;
    }

    else {
        // increment command smoothly based on elapsed time
        if(curr_cmd < accel_cmd_cap){
            curr_cmd += (accel_rate * (float)dt_ms);
        }

        //  transmit Throttle Command
        can_transmit[0] = motor1_can_id; // CS
        can_transmit[1] = 1; // command
        can_transmit[2] = 0; // padding
        can_transmit[3] = 0; // padding

        //fltptr = (uint8_t *)&curr_cmd;
        can_transmit[4] = float2can(curr_cmd, 100, 0);
        can_transmit[5] = float2can(curr_cmd, 100, 1);

        can_tx(can_transmit,motor1_can_id);

        // save timestamp for next loop
        last_accel_tx = now;
    }
}

static uint32_t last_decel_tx = 0;
void decel_test(void) {
    uint32_t now = HAL_GetTick();
    uint32_t dt_ms = now - last_decel_tx;

    // Only run every 10ms
    if (dt_ms < 10) {
        return;
    }

    static uint32_t test_start = 0;
    static float curr_cmd = 0.0f;
    //uint8_t* fltptr;

    // initial Load Command
    if(!has_initialized && pot_cmd >= 0.5f){
        can_transmit[0] = motor2_can_id; // CS
        can_transmit[1] = 2; // command
        can_transmit[2] = 0; // padding
        can_transmit[3] = 0; // padding

        //fltptr = (uint8_t *)&test_load;
        can_transmit[4] = float2can(current_q, 100, 0);
        can_transmit[5] = float2can(current_q, 100, 1);
        can_transmit[6] = float2can(current_d, 100, 0); //if index 0, write top half of float
        can_transmit[7] = float2can(current_d, 100, 1); //if index 1, write bottom half of float

        can_tx(can_transmit,motor1_can_id);
        has_initialized = true;
    }
    else {
        // accelerate up to the cap
        if(test_start == 0) {
            if(curr_cmd < accel_cmd_cap){
                curr_cmd += (accel_rate * (float)dt_ms);
            } else {
                test_start = HAL_GetTick(); // Reached max, start the delay timer
            }

            can_transmit[0] = motor1_can_id; // CS
            can_transmit[1] = 1; // command
            can_transmit[2] = 0; // padding
            can_transmit[3] = 0; // padding

            //fltptr = (uint8_t *)&curr_cmd;
            can_transmit[4] = float2can(curr_cmd, 100, 0);
            can_transmit[5] = float2can(curr_cmd, 100, 1);

            can_tx(can_transmit,motor1_can_id);
        }

        // decelerate after delay timer finishes
        if(test_start != 0 && (now - test_start > decel_delay)){
            if(curr_cmd > 0.0f){
                curr_cmd -= (decel_rate * (float)dt_ms);
                if(curr_cmd < 0.0f) curr_cmd = 0.0f; // Prevent negative wrap-around
            }

            can_transmit[0] = motor1_can_id; // CS
            can_transmit[1] = 1; // command
            can_transmit[2] = 0; // padding
            can_transmit[3] = 0; // padding

            //fltptr = (uint8_t *)&curr_cmd;
            can_transmit[4] = float2can(current, 100, 0);
            can_transmit[5] = float2can(current, 100, 1);

            can_tx(can_transmit,motor1_can_id);
        }

        // save timestamp for next loop
        last_decel_tx = now;
    }
}

static uint32_t last_stop_tx = 0;
void full_stop_test(void) {
    uint32_t now = HAL_GetTick();
    uint32_t dt_ms = now - last_stop_tx;

    // only run every 10ms to avoid flooding the bus
    if (dt_ms < 10) {
        return;
    }

    if(pot_cmd >= 0.5f){
        //uint8_t* fltptr;
        //float zero_val = 0.0f; // float variable to map to bytes

        // Message 1: CS=1, Cmd=0, Payload=0.0f
        can_transmit[0] = 1;
        can_transmit[1] = 0;
        can_transmit[2] = 0;
        can_transmit[3] = 0;

        //fltptr = (uint8_t *)&zero_val;
        can_transmit[4] = float2can(current, 100, 0);
        can_transmit[5] = float2can(current, 100, 1);

        can_tx(can_transmit,motor1_can_id);

        // Message 2: CS=2, Cmd=0, Payload=0.0f
        can_transmit[0] = 2;
        can_transmit[1] = 0;
        can_transmit[2] = 0;
        can_transmit[3] = 0;

        //fltptr = (uint8_t *)&zero_val;
        can_transmit[4] = float2can(current_q, 100, 0);
        can_transmit[5] = float2can(current_q, 100, 1);
        can_transmit[6] = float2can(current_d, 100, 0);
        can_transmit[7] = float2can(current_d, 100, 1);

        can_tx(can_transmit,motor1_can_id);
    }

    // save timestamp for next loop
    last_stop_tx = now;
}
#endif
//OLD FUNCTION IMPLEMENTATIONS
// Acceleration Test
// #ifdef ACCEL_TEST
//   static bool has_intilialized = false;
//   if(!has_intilialized && pot_cmd >=0.5){
//     fillTXBuffer(2,2,test_load);
//     can_tx(can_transmit);
//     has_intilialized = true;
// }
// static float curr_cmd = 0.0f;
// if(curr_cmd < accel_cmd_cap){
//     curr_cmd += accel_rate * (now - last_can_tx);
// }
// fillTXBuffer(1,1,curr_cmd);
// can_tx(can_transmit);
// #endif
//
// // Deceleration Test
// #ifdef DECCEL_TEST
// static uint32_t test_start = 0;
// static float curr_cmd = 0.0f;
// static bool has_intilialized = false;
// if(!has_intilialized && pot_cmd >=0.5){
//     fillTXBuffer(2,2,test_load);
//     can_tx(can_transmit);
//     has_intilialized = true;
// }
// if(test_start == 0 ){
//     if(curr_cmd < accel_cmd_cap){
//         curr_cmd += accel_rate * (now - last_can_tx);
//     }else{
//         test_start = HAL_GetTick();
//     }
//     fillTXBuffer(1,1,curr_cmd);
//     can_tx(can_transmit);
// }
// if(now-test_start > decel_delay){
//     if(curr_cmd>0){
//         curr_cmd -= decel_rate * (now - last_can_tx);
//     }else{
//         curr_cmd = 0;
//     }
//
//     fillTXBuffer(1,1,curr_cmd);
//     can_tx(can_transmit);
// }
//
// #endif
//
// // Full Stop Test
