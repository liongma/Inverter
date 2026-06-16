/*
 * appMath.c
 *
 *  Created on: Aug 13, 2025
 *      Author: LiongMa
 */


#include "appMath.h"
#include "math.h"
#include "main.h"

#define RADtoQ31 683565275
#define Q31to1 0.0000000004656612873077392578125


char msg1[64];
char msg2[64];
char msg3[64];

//CORDIC MATH
void appMath_Init() {
	LL_CORDIC_Config(CORDIC,
	  LL_CORDIC_FUNCTION_SINE, /* cosine function */
	  LL_CORDIC_PRECISION_6CYCLES, /* max precision for q1.31 cosine */
	  LL_CORDIC_SCALE_0, /* no scale */
	  LL_CORDIC_NBWRITE_1, /* One input data: angle. Second input data (modulus) is 1 after cordic reset */
	  LL_CORDIC_NBREAD_2, /* Two output data: cosine, then sine */
	  LL_CORDIC_INSIZE_32BITS, /* q1.31 format for input data */
	  LL_CORDIC_OUTSIZE_32BITS); /* q1.31 format for output data */
}

float wrap_angle_cordic(float theta) {
	float t = theta;
	if (t >= M_PI) {
		t -= M_2PI;
	} else if (t < -M_PI) {
		t += M_2PI;
	}
	return t;

}
int32_t rad_to_cordic_q31(float theta_rad){
	float t = wrap_angle_cordic(theta_rad);
	return (int32_t)(t * RADtoQ31);
}

void cordic_sin_cos(float theta, float* s, float* c) {
	int32_t angle_q31 = rad_to_cordic_q31(theta);
	LL_CORDIC_WriteData(CORDIC, (uint32_t)angle_q31);
	int32_t sin_q31 = (int32_t)LL_CORDIC_ReadData(CORDIC);
	int32_t cos_q31 = (int32_t)LL_CORDIC_ReadData(CORDIC);
	*s = (float)sin_q31 * Q31to1;
	*c = (float)cos_q31 * Q31to1;
}

float cordic_sin(float theta){
	int32_t angle_q31 = rad_to_cordic_q31(theta);
	LL_CORDIC_WriteData(CORDIC, (uint32_t)angle_q31);
	int32_t sin_q31 = (int32_t)LL_CORDIC_ReadData(CORDIC);
	LL_CORDIC_ReadData(CORDIC);
	return ((float)sin_q31 * Q31to1);
}

void CORDICTest() {
//	float tests[] = {
//			0.0f,
//			M_PI/6.0f,
//			M_PI/4.0f,
//			M_PI/2.0f,
//			M_PI,
//			-M_PI/2.0f,
//			3.0f,
//			-4.0f,
//			10.0f
//	};
//	for (int i = 0; i < (int)(sizeof(tests)/sizeof(tests[0])); i++) {
//	        float s, c;
//	        cordic_sin_cos(tests[i], &s, &c);   // this prints all the internal steps
//
//			int n2 = snprintf(msg2, sizeof(msg2), "cos value: %f\r\n", c);
//			HAL_Delay(10);
//			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)msg2, (uint16_t)n2);
//			HAL_Delay(10);
//			int n3 = snprintf(msg3, sizeof(msg3), "sin value: %f\r\n", s);
//			HAL_Delay(10);
//			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)msg3, (uint16_t)n3);
//	}
}

//MATH FUNCS
float clip(float val, float min, float max){
	if(val > max) return max;
	if(val < min) return min;
	return val;
}


//MATRIX MATH
mat41 mat41_sca(mat41 * a, float b){
	mat41 ret;
	ret.m[0] = a->m[0] * b;
	ret.m[1] = a->m[1] * b;
	ret.m[2] = a->m[2] * b;
	ret.m[3] = a->m[3] * b;
	return ret;
}

mat21 mat21_add(mat21 * a, mat21 * b){
	mat21 ret;
	ret.m[0] = a->m[0] + b->m[0];
	ret.m[1] = a->m[1] + b->m[1];
	return ret;
}

mat21 mat21_sub(mat21 * a, mat21 * b){
	mat21 ret;
	ret.m[0] = a->m[0] - b->m[0];
	ret.m[1] = a->m[1] - b->m[1];
	return ret;
}

mat21 mat21_mul(mat22 * a, mat21 * b){
	mat21 ret;
	ret.m[0] = a->m[0][0] * b->m[0] + a->m[0][1] * b->m[1];
	ret.m[1] = a->m[1][0] * b->m[0] + a->m[1][1] * b->m[1];
	return ret;
}

mat21 mat21_sca(mat21 * a, float b){
	mat21 ret;
	ret.m[0] = a->m[0] * b;
	ret.m[1] = a->m[1] * b;
	return ret;
}

mat22 mat_add(mat22 * a, mat22 * b){
	mat22 ret;
    ret.m[0][0] = a->m[0][0] + b->m[0][0];
    ret.m[0][1] = a->m[0][1] + b->m[0][1];
    ret.m[1][0] = a->m[1][0] + b->m[1][0];
    ret.m[1][1] = a->m[1][1] + b->m[1][1];
    return ret;
}

mat22 mat_sub(mat22 * a, mat22 * b){
    mat22 ret;
	ret.m[0][0] = a->m[0][0] - b->m[0][0];
    ret.m[0][1] = a->m[0][1] - b->m[0][1];
    ret.m[1][0] = a->m[1][0] - b->m[1][0];
    ret.m[1][1] = a->m[1][1] - b->m[1][1];
    return ret;
}

mat22 mat_mul(mat22 * a, mat22 * b){
    mat22 ret;  // Use temporary in case result aliases a or b

	ret.m[0][0] = a->m[0][0] * b->m[0][0] + a->m[0][1] * b->m[1][0];
	ret.m[0][1] = a->m[0][0] * b->m[0][1] + a->m[0][1] * b->m[1][1];
	ret.m[1][0] = a->m[1][0] * b->m[0][0] + a->m[1][1] * b->m[1][0];
	ret.m[1][1] = a->m[1][0] * b->m[0][1] + a->m[1][1] * b->m[1][1];

    return ret;
}

float mat_det(mat22 * a) {
    return a->m[0][0] * a->m[1][1] - a->m[0][1] * a->m[1][0];
}

mat22 mat_inv(mat22 * a) {
    float det = mat_det(a);
    mat22 ret = {{{0,0},{0,0}}};

    if (fabsf(det) < 1e-10f) return ret;

    float inv_det = 1.0f / det;
    ret.m[0][0] = a->m[1][1] * inv_det;
    ret.m[0][1] = -a->m[0][1] * inv_det;
    ret.m[1][0] = -a->m[1][0] * inv_det;
    ret.m[1][1] = a->m[0][0] * inv_det;
    return ret;

}

mat22 mat_tra(mat22 * a) {

    mat22 ret;
    ret.m[0][0] = a->m[0][0];
    ret.m[0][1] = a->m[1][0];
    ret.m[1][0] = a->m[0][1];
    ret.m[1][1] = a->m[1][1];
    return ret;
}

mat22 mat_sca(mat22 * a, float scalar) {
	mat22 ret;
    ret.m[0][0] = a->m[0][0] * scalar;
    ret.m[0][1] = a->m[0][1] * scalar;
    ret.m[1][0] = a->m[1][0] * scalar;
    ret.m[1][1] = a->m[1][1] * scalar;
    return ret;

}




//FOC MATH
AlphaBeta_Components sat_ab_circle(AlphaBeta_Components v, float vmax){
	AlphaBeta_Components ret;

	float mag = hypotf(v.alpha,v.beta);
	float scale = 1;
	if(mag >vmax && mag != 0){
		scale = vmax/mag;
	}
	ret.alpha = v.alpha*scale;
	ret.beta = v.beta*scale;
	return ret;
}

AlphaBeta_Components sat_ab_hexagon(AlphaBeta_Components v_ab, float v_bus) {
	// 1. Calculate equivalent line-to-line voltages
	float sqrt3_over_2 = 0.86602540378f;
	float sqrt3        = 1.73205080757f;

	float v_ll_ab =  1.5f * v_ab.alpha - sqrt3_over_2 * v_ab.beta;
	float v_ll_bc =  sqrt3 * v_ab.beta;
	float v_ll_ca = -1.5f * v_ab.alpha - sqrt3_over_2 * v_ab.beta;

	// 2. Find the maximum absolute line-to-line voltage using standard math functions
	float max_ll = fmaxf(fabsf(v_ll_ab), fmaxf(fabsf(v_ll_bc), fabsf(v_ll_ca)));

	// 3. Scale back if the vector exceeds the hexagon boundary
	if (max_ll > v_bus) {
		float scale = v_bus / max_ll;
		v_ab.alpha *= scale;
		v_ab.beta  *= scale;
	}

	return v_ab;
}

DQ_Components sat_dq_circle(DQ_Components v, float vmax){
	DQ_Components ret;
	float mag = hypotf(v.d,v.q);
	float scale = 1;
	if(mag >vmax && mag != 0){
		scale = vmax/mag;
	}
	ret.d = v.d*scale;
	ret.q = v.q*scale;

	return ret;
}

float normalizeAngle(float theta){
	float ret = fmod(theta,M_2PI);
	if(ret > M_2PI) ret -= M_2PI;
	else if(ret < 0) ret += M_2PI;
	return ret;
}


AlphaBeta_Components clarkeTransform(float a, float b, float c) {
    AlphaBeta_Components ab;
//    ab.alpha = a;  // Simple transform (assumes a + b + c = 0)
//    ab.beta = (b - c) * ONE_OVER_SQRT3;

     ab.alpha = TWO_OVER_3 * (a - 0.5f * b - 0.5f * c);
     ab.beta = 0.57735026919 * (b - c);
    //POWER INVARIANT FORM
//    ab.alpha = SQRT_2_OVER_3 * (a - 0.5f*b - 0.5f*c);
//    ab.beta  = ONE_OVER_SQRT2 * (0 + b - c);

    return ab;
}

// Inverse Clarke transform (alpha-beta to 3-phase)
ThreePhase_Components inverseClarkeTransform(float alpha, float beta) {
    ThreePhase_Components abc;
//    abc.a = alpha;
    abc.b = -0.5f * alpha + SQRT3_OVER_2 * beta;
    abc.c = -0.5f * alpha - SQRT3_OVER_2 * beta;
    //POWER INVARIANT FORM
//    abc.a = SQRT_2_OVER_3*alpha;
//    abc.b = SQRT_2_OVER_3*-0.5f*alpha + ONE_OVER_SQRT2*beta;
//    abc.c = SQRT_2_OVER_3*-0.5f*alpha - ONE_OVER_SQRT2*beta;
    return abc;
}

// Forward Park transform (alpha-beta to d-q)
DQ_Components parkTransform(float alpha, float beta, float theta) {
    DQ_Components dq;
    float cos_theta;
    float sin_theta;
//    cos_theta = cosf(theta);
//    sin_theta = sinf(theta);
    cordic_sin_cos(theta,&sin_theta,&cos_theta);


    dq.d = alpha * cos_theta + beta * sin_theta;
    dq.q = -alpha * sin_theta + beta * cos_theta;
    return dq;
}

// Inverse Park transform (d-q to alpha-beta)
AlphaBeta_Components inverseParkTransform(float d, float q, float theta) {
    AlphaBeta_Components ab;
    float cos_theta;
    float sin_theta;
//    cos_theta = cosf(theta);
//    sin_theta = sinf(theta);
    cordic_sin_cos(theta,&sin_theta,&cos_theta);

    ab.alpha = d * cos_theta - q * sin_theta;
    ab.beta = d * sin_theta + q * cos_theta;
    return ab;
}

// fast absolute value for floats
static inline float fast_fabsf(float a) {
	return (a < 0.0f) ? -a : a;
}

// fast maximum value for floats
static inline float fast_fmaxf(float a, float b) {
	return (a > b) ? a : b;
}
