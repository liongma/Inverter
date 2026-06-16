/*
 * speed.c
 *
 *  Created on: Mar 1, 2026
 *      Author: lionm
 */


#include "speed.h"
#include "main.h"
#include "defines.h"

#ifdef SPEED_CONTROL
static uint32_t speedCounter = 0;
static float w_val = 0;

#endif

#define REGEN_LIMIT 0


void speed_control(){
#ifdef SPEED_CONTROL
	if(speedCounter++ > 5){
		speedCounter = 0;



		float target_w = -cmd*10000;
//		if(target_w > w_val + 1.5) w_val+= 1;
//		else if(target_w < w_val - 1.5) w_val-= 1;
//		else w_val = target_w;
		w_val = target_w;
		if(fabs(target_w) > 100){
			ref_w = w_val*0.001 + 0.999*ref_w;
		}
		else{
			ref_w = w_val*0.003 + 0.997*ref_w;
		}
		if(fabs(ref_w) > v_bus/600.0f *10000.0f){
			ref_w = -v_bus/600.0f *10000.0f;
		}
		if (cmd == 0){
			error_w = 0;
		}
		else{
			error_w =  ref_w - omega_e;
		}

		float diff_w = error_w - error_w_i;
		error_w_i = error_w;

//		float Kp_w = 0.508;//0.02;
		float Kp_w = 0.01;
//		float Ki_w = 0;//0.0341;//3.41;//0.005;
		float Ki_w = 0.034;
	    float Kd_w = 0.000 ;

	    float Kaw_w= 10.0;

		integral_w += error_w * (Ts) * Ki_w;


		float integral_limit_w = DYNO_CURRENT/1.5f;
		if(fabs(w_e) <500){
			integral_limit_w = 5;
		}
		if(integral_w > integral_limit_w) integral_w = integral_limit_w;
		if(integral_w < -integral_limit_w) integral_w = -integral_limit_w;


		i_set = Kp_w * error_w + integral_w + Kd_w * diff_w + (float)can_recieve[7];
		i_sat = clip(i_set,-DYNO_CURRENT, DYNO_CURRENT);
		if(fabs(w_e) <500){
			i_sat = clip(i_set,-10, 10);
		}
		if(fabs(w_e) < 2 && ref_w == 0){
			i_sat = 0;
		}
		if (fabs(i_sat) < 1) i_sat = 0;

		if(w_e > 50){
			if(i_sat < -REGEN_LIMIT) i_sat = -REGEN_LIMIT;
		}
		else if (w_e < -50){
			if(i_sat > REGEN_LIMIT) i_sat = REGEN_LIMIT;
		}



		float i_aw = i_sat - i_set;
		integral_w += i_aw * (Ts) *Ki_w* Kaw_w;

//		ref.q = i_sat*mtpva_q(i_sat,omega_e,v_bus);
//		ref.d = i_sat*mtpva_d(i_sat,omega_e,v_bus);
		float cmd_d = 0.2*i_sat;
		float cmd_q = i_sat;

//	    float i_step_size_q_i = 0.05;
//	    float i_step_size_q = 0.05;

	    ref.d = cmd_d;
	    if (can_recieve[7] <= 1 && fabs(ref_w) < 5.0f && omega_e < 5.0f) {
	    	ref.q = 0;
	    	integral_w = 0;
	    } else {
	    	ref.q = cmd_q;
	    }

	    //	    if(fabs(cmd_q) > 1){
//			if(cmd_q > ref.q+i_step_size_q_i)ref.q+=i_step_size_q_i;
//			else if(cmd_q < ref.q-i_step_size_q)ref.q-=i_step_size_q;
//			else if(ref.q < i_step_size_q_i && ref.q > -i_step_size_q && cmd_q == 0)ref.q = 0;
//	    }
//	    else{
//	    	ref.q = cmd_q;
//	    }


//		ref.q = mtpa_q(i_sat, omega_e);
//		ref.d = mtpa_d(i_sat, omega_e);

	}
#endif
}
