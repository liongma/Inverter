/*
 * off_detect.c
 *
 *  Created on: Mar 1, 2026
 *      Author: lionm
 */


#include "off_detect.h"
#include "defines.h"
#include "main.h"

#ifdef RS_DET
	const float P0 = 1000;
	const float RS0 = 0.0675;

	mat22 K = 		{{{0,0},
					{0,0}}};
	mat22 K_1 = 	{{{0,0},
					{0,0}}};

	mat22 P = 		{{{P0,0},
					{0,P0}}};
	mat22 P_1 = 	{{{P0,0},
					{0,P0}}};

	mat22 PHI = 	{{{0,1},
				  	{0,1}}};
	mat22 PHI_T = 	{{{0,1},
				    {0,1}}};

	mat21 THETA = 	{{RS0,0}};
	mat21 THETA_1 = {{RS0,0}};

	mat22 I = 		{{{1,0},
					{0,1}}};

	mat21 E = 		{{0,0}};
	mat21 Y = 		{{0,0}};
#endif


#ifdef LQ_DET
	const float P0 = 1000.0f;
	const float LQ0 = 0.000330f;
	mat21 K = 		{{0,0}};
	mat21 K_1 = 	{{0,0}};

	mat22 P = 		{{{P0,0},
					{0,P0}}};
	mat22 P_1 = 	{{{P0,0},
					{0,P0}}};

	mat21 PHI =		{{0,1}};
	mat21 PHI_T =	{{0,1}};

	mat21 THETA = 	{{LQ0,0}};
	mat21 THETA_1 = {{LQ0,0}};

	mat22 I =		{{{1,0},
					{0,1}}};

	float E = 		0;
	float Y = 		0;
#endif

#if defined(LD_DET) || defined(LQ_DET_HF)
	const float P0 = 1000.0f;
	const float LD0 = 0.000330f;
	mat41 K = 		{{0,0,0,0}};
	mat41 K_1 = 	{{0,0,0,0}};

	mat44 P = 		{{
			{P0,0,0,0},
			{0,P0,0,0},
			{0,0,P0,0},
			{0,0,0,P0}
	}};
	mat44 P_1 = 		{{
			{P0,0,0,0},
			{0,P0,0,0},
			{0,0,P0,0},
			{0,0,0,P0}
	}};

	mat41 PHI =		{{0,1,0,0}};
	mat41 PHI_T =	{{0,1,0,0}};

	mat41 THETA = 	{{LD0,0,0,0}};
	mat41 THETA_1 = {{LD0,0,0,0}};

	mat44 I = 		{{
			{1,0,0,0},
			{0,1,0,0},
			{0,0,1,0},
			{0,0,0,1}
	}};

	float E = 		0;
	float Y = 		0;

	float omega_inj = 1000.0f;
	float t_inj = 0.0f;
	float sin_inj = 0.0f;
	float cos_inj = 5.0f;
	float amp_inj = 5.0f;

	float dIdt_d = 1;
	float dIdt_q = 1;

	float local_max = 0;
	float local_max_counter = 0;

#endif

#ifdef PSI_DET
	const float P0 = 1000;
	const float PSI0 = 0.020;

	mat22 K = 		{{{0,0},
					{0,0}}};
	mat22 K_1 = 	{{{0,0},
					{0,0}}};

	mat22 P = 		{{{P0,0},
					{0,P0}}};
	mat22 P_1 = 	{{{P0,0},
					{0,P0}}};

	mat22 PHI = 	{{{0,1},
				  	{0,1}}};
	mat22 PHI_T = 	{{{0,1},
				    {0,1}}};

	mat21 THETA = 	{{PSI0,0}};
	mat21 THETA_1 = {{PSI0,0}};

	mat22 I = 		{{{1,0},
					{0,1}}};

	mat21 E = 		{{0,0}};
	mat21 Y = 		{{0,0}};
#endif

void run_detect(){

#ifdef RS_DET
	  if(imag > 0){
		  const float forget = 0.999f;
		  const float forget_inv = 1.0f/forget;


		  //Observation
		  PHI_T.m[0][0] = If.d; PHI_T.m[0][1] = 1;
		  PHI_T.m[1][0] = If.q; PHI_T.m[1][1] = 1;

		  PHI.m[0][0] = If.d; 	PHI.m[0][1] = If.q;
		  PHI.m[1][0] = 1;		PHI.m[1][1] = 1;

		  Y.m[0] = v_sat.d;		Y.m[1] = v_sat.q;

		  //Kalman Gain
		  mat22 Ktop = mat_mul(&P_1,&PHI);

		  mat22 Kbot1 = mat_sca(&I,forget);
		  mat22 Kbot2 = mat_mul(&PHI_T,&Ktop);
		  mat22 Kbot = mat_add(&Kbot1,&Kbot2);

		  mat22 Kbot_inv = mat_inv(&Kbot);

		  K = mat_mul(&Ktop,&Kbot_inv);

		  //Covariance Error Update
		  mat22 Proot1_term = mat_mul(&K,&PHI_T);
		  mat22 Proot1 = mat_sub(&I,&Proot1_term);
		  mat22 Proot2 = mat_sca(&P_1,forget_inv);

		  P = mat_mul(&Proot1,&Proot2);

		  //Error Update
		  mat21 Eterm = mat21_mul(&PHI_T,&THETA_1);

		  E = mat21_sub(&Y,&Eterm);

		  //State Prediction
		  mat21 THETAterm = mat21_mul(&K,&E);

		  THETA = mat21_add(&THETA_1,&THETAterm);
		  Rs_det = THETA.m[0]; dV_det = THETA.m[1];

		  // Z^-1
		  K_1 = K;
		  P_1 = P;
		  THETA_1 = THETA;
	  }

#endif
#ifdef LQ_DET
	  if(imag > 1){

	  const float forget = 0.9999f;
	  const float forget_inv = 1.0f/forget;

	  PHI_T.m[0] = -If.q * omega_e; PHI_T.m[1]= 1; // 1x2
	  PHI.m[0] = -If.q * omega_e; 	PHI.m[1] = 	1; // 2x1

	  Y = v_sat.d - Rs_det * If.d;

	  //Kalman Gain
	  mat21 Ktop = mat21_mul(&P_1,&PHI);
	  float Kbot = forget + PHI.m[0] * Ktop.m[0] + PHI.m[1] * Ktop.m[1];
	  float Kbot_inv = 1.0f/Kbot;

	  K = mat21_sca(&Ktop,Kbot_inv); //2x1

	  //Covariance Error Update
	  //term1
	  mat22 Pterm1;
	  Pterm1.m[0][0] = K.m[0]*PHI.m[0];
	  Pterm1.m[0][1] = K.m[0]*PHI.m[1];
	  Pterm1.m[1][0] = K.m[1]*PHI.m[0];
	  Pterm1.m[1][1] = K.m[1]*PHI.m[1];

	  mat22 Proot1 = mat_sub(&I,&Pterm1);
	  mat22 Proot = mat_mul(&Proot1,&P_1);
	  P = mat_sca(&Proot,forget_inv);

	  //Error Update
	  E = Y - (PHI.m[0] * THETA.m[0] + PHI.m[1] * THETA.m[1]);

	  //State Prediction
	  THETA.m[0] += K.m[0] * E;
	  THETA.m[1] += K.m[1] * E;

	  Lq_det = THETA.m[0];
	  dV_det = THETA.m[1];

//	  if(THETA.m[0] > 0.000400) THETA.m[0] = 0.000400; // UPPER LIMIT
//	  if(THETA.m[0] < 0.000050) THETA.m[0] = 0.000050; // LOWER LIMIT


	  // Z^-1
	  K_1 = K;
	  P_1 = P;
	  THETA_1 = THETA;

	  }
#endif

#ifdef LD_DET
	  if(imag > 5){
	  const float forget = 0.997f;
	  const float forget_inv = 1.0f/forget;
//	  const float scaling_factor = 1;
	  const float fudge_factor = 1;
	  if(local_max_counter++ > 100){
		  local_max_counter = 0;
		  local_max = 0;
	  }
	  dIdt_d = (fudge_factor * (dI.d * freq))*0.3 + 0.7*dIdt_d;
	  if(fabs(dIdt_d) > local_max) local_max = fabs(dIdt_d);
	  Lq_det = estimate_Lq(If.q,If.d);

	  PHI_T.m[0] = dIdt_d;	 	PHI_T.m[1]= 1; 	PHI_T.m[2]= dIdt_d*sin_inj;	PHI_T.m[3]= dIdt_d*cos_inj;// 1x4
	  PHI.m[0] = dIdt_d; 		PHI.m[1]= 1; 	PHI.m[2]= dIdt_d*sin_inj;	PHI.m[3]= dIdt_d*cos_inj;// 4x1

	  Y = v_sat.d - Rs_det*If.d + Lq_det*If.q*omega_e;


	  //Kalman Gain
	  mat41 Ktop;
	  for (int i = 0; i < 4; i++) {
		  Ktop.m[i] = 0.0f;
	      for (int j = 0; j < 4; j++) {
	    	  Ktop.m[i] += P.m[i][j] * PHI.m[j];
	      }
	  }

	  float Kbot = forget + PHI.m[0]*Ktop.m[0] + PHI.m[1]*Ktop.m[1] + PHI.m[2]*Ktop.m[2] + PHI.m[3]*Ktop.m[3];
	  float Kbot_inv = 1.0f/Kbot;

	  K = mat41_sca(&Ktop,Kbot_inv); //2x1

	  //Covariance Error Update
	  mat44 Pterm1;

	  for (int i = 0; i < 4; i++){
		  for(int j = 0; j < 4; j++){
			  Pterm1.m[i][j] = -K.m[i]*PHI.m[j];
			  if (i==j)	Pterm1.m[i][j] = 1 + Pterm1.m[i][j];
			  Pterm1.m[i][j] = Pterm1.m[i][j];
		  }
	  }

	  mat44 P_temp;

	  for (int i = 0; i < 4; i++) {
	      for (int j = 0; j < 4; j++) {
	    	  P_temp.m[i][j] = 0.0f;
			  for (int k = 0; k < 4; k++) {
				  P_temp.m[i][j] += Pterm1.m[i][k] * P_1.m[k][j];
			  }
			  P_temp.m[i][j] *= forget_inv;
	      }
	  }
	  P = P_temp;

	  //Error Update
	  E = Y - (PHI.m[0] * THETA.m[0] + PHI.m[1] * THETA.m[1] + PHI.m[2] * THETA.m[2] + PHI.m[3] * THETA.m[3]);

	  //State Prediction
	  THETA.m[0] += K.m[0] * E;
	  THETA.m[1] += K.m[1] * E;
	  THETA.m[2] += K.m[2] * E;
	  THETA.m[3] += K.m[3] * E;


	  Ld_det = THETA.m[0];
	  dV_det = THETA.m[1];

	  // Z^-1
	  K_1 = K;
	  P_1 = P;
	  THETA_1 = THETA;
	  //Update High Frequency Injection
	  t_inj = normalizeAngle(t_inj + omega_inj * Ts);
	  sin_inj = sinf(t_inj);
	  cos_inj = cosf(t_inj);
	  }

#endif

#ifdef LQ_DET_HF
	  if(imag > 5){
	  const float forget = 0.999f;
	  const float forget_inv = 1.0f/forget;
	  const float fudge_factor = 1.0f;
	  if(local_max_counter++ > 100){
		  local_max_counter = 0;
		  local_max = 0;
	  }
	  dIdt_q = (fudge_factor * (dI.q * freq))*0.3 + 0.7*dIdt_q;
	  if(fabs(dIdt_q) > local_max) local_max = fabs(dIdt_q);

	  PHI_T.m[0] = dIdt_q;	 	PHI_T.m[1]= 1; 	PHI_T.m[2]= dIdt_q*sin_inj;	PHI_T.m[3]= dIdt_q*cos_inj;// 1x4
	  PHI.m[0] = dIdt_q; 		PHI.m[1]= 1; 	PHI.m[2]= dIdt_q*sin_inj;	PHI.m[3]= dIdt_q*cos_inj;// 4x1

	  Y = v_sat.q - Rs_det*If.q - estimate_Ld(If.q,If.d)*If.d*omega_e - omega_e*Psi_det;


	  //Kalman Gain
	  mat41 Ktop;
	  for (int i = 0; i < 4; i++) {
		  Ktop.m[i] = 0.0f;
	      for (int j = 0; j < 4; j++) {
	    	  Ktop.m[i] += P.m[i][j] * PHI.m[j];
	      }
	  }

	  float Kbot = forget + PHI.m[0]*Ktop.m[0] + PHI.m[1]*Ktop.m[1] + PHI.m[2]*Ktop.m[2] + PHI.m[3]*Ktop.m[3];
	  float Kbot_inv = 1.0f/Kbot;

	  K = mat41_sca(&Ktop,Kbot_inv); //2x1

	  //Covariance Error Update
	  mat44 Pterm1;

	  for (int i = 0; i < 4; i++){
		  for(int j = 0; j < 4; j++){
			  Pterm1.m[i][j] = -K.m[i]*PHI.m[j];
			  if (i==j)	Pterm1.m[i][j] = 1 + Pterm1.m[i][j];
			  Pterm1.m[i][j] = Pterm1.m[i][j];
		  }
	  }

	  mat44 P_temp;

	  for (int i = 0; i < 4; i++) {
	      for (int j = 0; j < 4; j++) {
	    	  P_temp.m[i][j] = 0.0f;
			  for (int k = 0; k < 4; k++) {
				  P_temp.m[i][j] += Pterm1.m[i][k] * P_1.m[k][j];
			  }
			  P_temp.m[i][j] *= forget_inv;
	      }
	  }
	  P = P_temp;

	  //Error Update
	  E = Y - (PHI.m[0] * THETA.m[0] + PHI.m[1] * THETA.m[1] + PHI.m[2] * THETA.m[2] + PHI.m[3] * THETA.m[3]);

	  //State Prediction
	  THETA.m[0] += K.m[0] * E;
	  THETA.m[1] += K.m[1] * E;
	  THETA.m[2] += K.m[2] * E;
	  THETA.m[3] += K.m[3] * E;


	  Lq_det = THETA.m[0];
	  dV_det = THETA.m[1];

	  // Z^-1
	  K_1 = K;
	  P_1 = P;
	  THETA_1 = THETA;
	  //Update High Frequency Injection
	  t_inj = normalizeAngle(t_inj + omega_inj * Ts);
	  sin_inj = sinf(t_inj);
	  cos_inj = cosf(t_inj);
	  }

#endif


#ifdef PSI_DET
	  if(imag > 0 && fabs(omega_e) > 500){
//		  const float forget = 0.999f;
		  const float forget = 0.99999f;
		  const float forget_inv = 1.0f/forget;

		  Lq_det = estimate_Lq(If.q,If.d);
		  Ld_det = estimate_Lq(If.q,If.d);

		  //Observation
		  PHI_T.m[0][0] = 0; 		PHI_T.m[0][1] = 1;
		  PHI_T.m[1][0] = omega_e; 	PHI_T.m[1][1] = 1;

		  PHI.m[0][0] = 0; 			PHI.m[0][1] = omega_e;
		  PHI.m[1][0] = 1;			PHI.m[1][1] = 1;

		  Y.m[0] = v_sat.d - Rs_det*If.d + Lq_det*If.q*omega_e;
		  Y.m[1] = v_sat.q - Rs_det*If.q - Ld_det*If.d*omega_e;

		  //Kalman Gain
		  mat22 Ktop = mat_mul(&P_1,&PHI);

		  mat22 Kbot1 = mat_sca(&I,forget);
		  mat22 Kbot2 = mat_mul(&PHI_T,&Ktop);
		  mat22 Kbot = mat_add(&Kbot1,&Kbot2);

		  mat22 Kbot_inv = mat_inv(&Kbot);

		  K = mat_mul(&Ktop,&Kbot_inv);

		  //Covariance Error Update
		  mat22 Proot1_term = mat_mul(&K,&PHI_T);
		  mat22 Proot1 = mat_sub(&I,&Proot1_term);
		  mat22 Proot2 = mat_sca(&P_1,forget_inv);

		  P = mat_mul(&Proot1,&Proot2);

		  //Error Update
		  mat21 Eterm = mat21_mul(&PHI_T,&THETA_1);

		  E = mat21_sub(&Y,&Eterm);

		  //State Prediction
		  mat21 THETAterm = mat21_mul(&K,&E);

		  THETA = mat21_add(&THETA_1,&THETAterm);

		  Psi_det = THETA.m[0];
		  dV_det = 	THETA.m[1];

		  // Z^-1
		  K_1 = K;
		  P_1 = P;
		  THETA_1 = THETA;
	  }

#endif

}
