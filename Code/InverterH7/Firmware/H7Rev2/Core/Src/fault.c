/*
 * fault.c
 *
 *  Created on: Feb 28, 2026
 *      Author: lionm
 */


#include "fault.h"

#define CMD_ON (fabs(cmd) > 0.05 && inverter_state < 10)

static uint16_t slow_counter = 0;
static float v_bus_1 = 0;
float dv_bus_dt = 0;

static int8_t diverge_q = 0;
static int8_t diverge_d = 0;

static int8_t est_cnt = 0;

static int8_t fault_cnt = 0;

int16_t membuf[35] = {0};
int16_t membuf2[35] = {0};
int16_t membuf3[35] = {0};
int16_t membuf4[35] = {0};
int16_t membuf5[35] = {0};
int16_t membuf6[35] = {0};
float predict_limit = 0.1;
const static int8_t miss_limit = 20;


void fault_buffer(void){
	for(int i = 31; i > 0; i--){
		membuf[i] = membuf[i-1];
		membuf2[i] = membuf2[i-1];
		membuf3[i] = membuf3[i-1];
		membuf4[i] = membuf4[i-1];
		membuf5[i] = membuf5[i-1];
		membuf6[i] = membuf6[i-1];

	}
	membuf[0] = (int)(theta_e*100);
//	membuf[0] = (int)(omega_e/100);
	membuf2[0] = ((int)(i_dq.q));
	membuf3[0] = ((int)(i_dq.d));
	membuf4[0] = ((int)(v_bus));
	membuf5[0] = ((int)(v.d));
	membuf6[0] = ((int)(test_angle*100));

}

static void fault_buffer_print(int16_t * buffer, int num){
	HAL_Delay(15);
	sprintf(msg,
			"------------------------------\n"
			"num: %d \n"
			"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d \n"
			"------------------------------"
			"\n",
			num,
			buffer[31],buffer[30],
						buffer[29],buffer[28],buffer[27],buffer[26],buffer[25],buffer[24],buffer[23],buffer[22],buffer[21],buffer[20],
						buffer[19],buffer[18],buffer[17],buffer[16],buffer[15],buffer[14],buffer[13],buffer[12],buffer[11],buffer[10],
						buffer[9],buffer[8],buffer[7],buffer[6],buffer[5],buffer[4],buffer[3],buffer[2],buffer[1],buffer[0]

	);
	HAL_Delay(30);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)msg, strlen(msg));
	HAL_Delay(15);
}

void fault_init(void){
	TIM1->CR2 &= ~(TIM_CR2_OIS1  | TIM_CR2_OIS1N |
	               TIM_CR2_OIS2  | TIM_CR2_OIS2N |
	               TIM_CR2_OIS3  | TIM_CR2_OIS3N |
	               TIM_CR2_OIS4);
	IDLE_OUT;

}



void fault_dump_output(uint8_t fault){
	IDLE_OUT;
	DISABLE_OUT;
//	UPHASE = 0; VPHASE = 0; WPHASE = 0;
	HAL_GPIO_WritePin(PROBE_GPIO_Port, PROBE_Pin, 1);
	HAL_GPIO_WritePin(PROBE_GPIO_Port, PROBE_Pin, 0);

	#ifdef CAN
	inverter_can_fault_call();

	inverter_fault = fault;
	can_report(0);
	HAL_Delay(5);
	can_report(1);
	HAL_Delay(5);
	can_report(2);
	HAL_Delay(5);


	#else
	HAL_Delay(10);
	#endif

	HAL_Delay(50);
	sprintf(msg,
		"\n\n"
		"angle:%2.2f, cmdw: %2.1f, cmd: %2.1f, refq:%+2.1f, iq:%+2.1f, refd:%+2.1f, id:%+2.1f, vq:%+2.1f, vd:%+3.1f, v:%2.1f, omega:%2.2f, f:%5.0f\n"
		"t_e_1: %2.2f, w_e: %5.1f, accl_e: 5.1f \n"

			,theta	,	ref_w	,	cmd,	ref.q, 		i_dq.q,		ref.d, 	i_dq.d, 	v.q,		 v.d, 	v_bus, 	omega_e	,	freq,
			theta_e_1, w_e, acl_e);

	HAL_Delay(50);
	HAL_Delay(50);


	fault_buffer();

	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)msg, strlen(msg));
	HAL_Delay(10);

	fault_buffer_print(membuf, 1);
	fault_buffer_print(membuf2,2);
	fault_buffer_print(membuf3,3);
	fault_buffer_print(membuf4,4);
	fault_buffer_print(membuf5,5);
	fault_buffer_print(membuf6,6);


	sprintf(msg,"\n est: %f, act: %f \n",
			theta_e_est, theta_e
	);
	HAL_Delay(10);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)msg, strlen(msg));
	HAL_Delay(10);
	sprintf(msg,"\nfault code: %d, state: %d \n\n\n",(int)fault, (int)inverter_state);
	HAL_Delay(10);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)msg, strlen(msg));



	Error_Handler();
}




void fault_if_undervolt(void){
	if(v_bus < -5){ //UNDERVOLT FAULT
		fault_dump_output(200);
	}
}
void fault_if_overvolt(void){
	if(v_bus > 650){//OVERVOLT FAULT
		fault_dump_output(201);
	}

}

void fault_if_overcurrent(void){
	if(fabs(i_phase[0]) > 140 || fabs(i_phase[1]) > 140 || fabs(i_phase[2]) > 140){ // OVERCURRENT FAULT
		fault_dump_output(202);
	}
	if(fabs(i_dq.q) > 140||fabs(i_dq.q) > 140){ // OVERCURRENT FAULT
		fault_dump_output(202);
	}
}
void fault_if_low_power(void){
	if(v_bus < 10 && CMD_ON ){ //LOW POWER FAULT
		fault_dump_output(203);

	}
}
void fault_if_slow(void){

	if(inverter_state >= 10) {
		slow_counter = 0;
		return;
	}
#ifdef CURRENT_CONTROL
	if(fabs(cmd) <= 0.01) {
		slow_counter = 0;
		return;
	}
#endif
	if(inverter_state == 2){
		if(fabs(i_dq.q - ref.q) > 100 || fabs(i_dq.d - ref.d) > 100){
			slow_counter+=15;
		}
		else if(fabs(i_dq.q - ref.q) > 50 || fabs(i_dq.d - ref.d) > 50){
			slow_counter+=5;
		}
		else if(fabs(i_dq.q - ref.q) > 30 || fabs(i_dq.d - ref.d) > 30){ //SLOW CONTROLS
			slow_counter+=3;
		}
		else if(fabs(i_dq.q - ref.q) > 10 || fabs(i_dq.d - ref.d) > 10){ //SLOW CONTROLS
			slow_counter++;
		}
		else{slow_counter=0;}
	}
	else{
		slow_counter = 0;
	}

	if(slow_counter > 2000){
		fault_dump_output(204);
	}

}
void fault_if_voltage_drop(void){
	dv_bus_dt = (v_bus - v_bus_1)*0.1 + 0.9*dv_bus_dt;
	v_bus_1 = v_bus;
	if(fabs(dv_bus_dt) > 0.03*v_bus && CMD_ON){
		fault_dump_output(205);
	}
}



void fault_if_encoder(void){

	float temp_omega_e = shortenAngle(theta_e - theta_e_1)*freq;
	float temp_accel_e = (omega_e - omega_e_1);


	if(	(fabs(temp_omega_e) > 12000 )){
		if(est_cnt > miss_limit)	fault_dump_output(216);
		else est_cnt++;
	}
	else if (fabs(temp_omega_e - w_e) > 800 && inverter_state < 10){
		if(est_cnt > miss_limit)	fault_dump_output(217);
		else est_cnt++;
	}
	else if (fabs(temp_accel_e - acl_e) > 500 && inverter_state < 10 ){
		if(est_cnt > miss_limit)	fault_dump_output(218);
		else est_cnt++;
	}
	else{
		est_cnt = 0;
	}
	if(est_cnt != 0){
		test_angle = normalizeAngle(test_angle_1 + w_e*Ts + 0.5*acl_e*Ts);// + 0.167*jrk_e*Ts*Ts*Ts;
	}

}

uint8_t fault_if_overspeed(void){
	if(fabs(omega_e) > 12000 && inverter_state != 3 && inverter_state != 1 && inverter_state <= 10){ //OVERSPEED
		fault_dump_output(206);
	}
	return 0;
}

uint8_t fault_if_overaccel(void){
	static const float accel_limit = 2000.0f;
	if(fabs(accel_e) > accel_limit && inverter_state <= 10 && inverter_state != 3 && inverter_state != 1){
		fault_dump_output(207);
	}
	return 0;
}

uint8_t fault_if_overjerk(void){
	static const float jerk_limit = 1000.0f;

	if(fabs(jrk_e) > jerk_limit && inverter_state <= 10 && inverter_state != 3 && inverter_state != 1){
		fault_dump_output(208);
	}
	return 0;
}

void fault_if_over_predict(void){
//	float predict_limit_replace = 0.2;
	predict_limit = 0.1;
//	if(w_e>1000) predict_limit = 2.0f*(fabs(w_e)*0.0001f);
//	predict_limit = 0.1;
	if(fault_cnt > miss_limit){
		fault_dump_output(209);
	}
	theta_e_est = theta_e_1 + w_e*0.0001f + 0.5*acl_e*0.0001f;// + 0.167*jrk_e*Ts*Ts*Ts;
	if(fabs(shortenAngle(theta_e - theta_e_est)) > predict_limit && inverter_state < 10){
//		if (fabs(shortenAngle(theta_e - theta_e_est)) < predict_limit_replace) {
//			theta_e = (theta_e + theta_e_est) / 2;
//		}
//	else{
		test_angle = theta_e_est;
		fault_cnt ++;
//	}
	}
	else fault_cnt = 0;
}



void fault_if_diverge(void){

	const int fault_limit = 50;
	const int error_limit = 50;

	if(
			diverge_q > fault_limit || diverge_q < -fault_limit ||
			diverge_d > fault_limit || diverge_d < -fault_limit)
	{
		fault_dump_output(210);
	}
	if(CMD_ON){
		if(diverge_q > 0){
			if(error.q > error_limit) diverge_q++;
			else diverge_q/=2;
		}
		else{
			if(error.q < -error_limit) diverge_q--;
			else diverge_q/=2;
		}

		if(diverge_d > 0){
			if(error.d > error_limit) diverge_d++;
			else diverge_d/=2;
		}
		else{
			if(error.d < -error_limit) diverge_d--;
			else diverge_d/=2;
		}
	}

}


void fault_if_extern(void){
	if(vcu_cmd.state & VCU_STATE_FAULT){
		fault_dump_output(211);
	}
}

void fault_manual(void){
	fault_dump_output(212);
}

void fault_if_can_dead(void){
	if(can_rx_fail > 100 && inverter_state < 10){
		fault_dump_output(213);
	}
}

void fault_drive_voltage_drop(void){
	fault_dump_output(214);
}
