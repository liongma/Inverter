/*
 * appMath.c
 *
 *  Created on: Aug 13, 2025
 *      Author: LiongMa
 */


#include "appMath.h"
#include "math.h"



float clip(float val, float min, float max){
	if(val > max) return max;
	if(val < min) return min;
	return val;
}

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
	if(ret < 0) ret += M_2PI;
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
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);

    dq.d = alpha * cos_theta + beta * sin_theta;
    dq.q = -alpha * sin_theta + beta * cos_theta;
    return dq;
}

// Inverse Park transform (d-q to alpha-beta)
AlphaBeta_Components inverseParkTransform(float d, float q, float theta) {
    AlphaBeta_Components ab;
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);

    ab.alpha = d * cos_theta - q * sin_theta;
    ab.beta = d * sin_theta + q * cos_theta;
    return ab;
}

