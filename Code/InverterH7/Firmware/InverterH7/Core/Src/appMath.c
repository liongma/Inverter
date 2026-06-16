/*
 * appMath.c
 *
 *  Created on: Aug 13, 2025
 *      Author: LiongMa
 */


#include "appMath.h"
#include "math.h"


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
     ab.beta = TWO_OVER_3 * (SQRT3_OVER_2 * b - SQRT3_OVER_2 * c);
    return ab;
}

// Inverse Clarke transform (alpha-beta to 3-phase)
ThreePhase_Components inverseClarkeTransform(float alpha, float beta) {
    ThreePhase_Components abc;
    abc.a = alpha;
    abc.b = -0.5f * alpha + SQRT3_OVER_2 * beta;
    abc.c = -0.5f * alpha - SQRT3_OVER_2 * beta;
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

// Add third harmonic injection for SVPWM
ThreePhase_Components addThirdHarmonic(ThreePhase_Components abc) {
    // Find the min and max of the three phases
    float min_val = fminf(abc.a, fminf(abc.b, abc.c));
    float max_val = fmaxf(abc.a, fmaxf(abc.b, abc.c));

    // Calculate third harmonic component
    float third_harmonic = 0.5f * (max_val + min_val);

    // Inject third harmonic
    ThreePhase_Components result;
    result.a = abc.a - third_harmonic;
    result.b = abc.b - third_harmonic;
    result.c = abc.c - third_harmonic;

    return result;
}

// Complete inverse transform with third harmonic injection
ThreePhase_Components inverseDQTransformWithSVPWM(DQ_Components dq, float theta) {
    // 1. Inverse Park to get alpha-beta
    AlphaBeta_Components ab = inverseParkTransform(dq.d, dq.q, theta);

    // 2. Inverse Clarke to get 3-phase
    ThreePhase_Components abc = inverseClarkeTransform(ab.alpha, ab.beta);

    // 3. Add third harmonic injection for SVPWM
    return addThirdHarmonic(abc);
}
