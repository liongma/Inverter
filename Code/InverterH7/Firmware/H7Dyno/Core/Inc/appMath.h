/*
 * appMath.h
 *
 *  Created on: Aug 13, 2025
 *      Author: LiongMa
 */

#ifndef INC_APPMATH_H_
#define INC_APPMATH_H_

#include "stdio.h"
#include "main.h"

#include <math.h>
#include <stdio.h>

// Constants
#define SQRT3_OVER_2 0.86602540378f  // sqrt(3)/2
#define TWO_OVER_3   0.66666666666f  // 2/3
#define SQRT_2_OVER_3 0.81649658092
#define ONE_OVER_SQRT3 0.57735026919f // 1/sqrt(3)
#define M_2PI       6.28318530718f   // 2Pi
#define M_PI2		1.57079632679f //PI/2
#define M_PI4		0.78539816339
#define M_PI3		1.0471975512
#define M_PI6		1.0471975512

#define ONE_OVER_SQRT2 0.70710678118
#define M_3_OVER_PI 0.95492965855



// Structure for DQ currents/voltages
typedef struct {
    float d;    // Direct axis component
    float q;    // Quadrature axis component
} DQ_Components;

// Structure for Alpha-Beta components
typedef struct {
    float alpha;
    float beta;
} AlphaBeta_Components;

// Structure for 3-phase components
typedef struct {
    float a;
    float b;
    float c;
} ThreePhase_Components;

// Functions

float clip(float val, float min, float max);
AlphaBeta_Components sat_ab_circle(AlphaBeta_Components v, float vmax);
DQ_Components sat_dq_circle(DQ_Components v, float vmax);

float normalizeAngle(float theta);
float shortenAngle(float theta);

AlphaBeta_Components clarkeTransform(float a, float b, float c);
ThreePhase_Components inverseClarkeTransform(float alpha, float beta);

DQ_Components parkTransform(float alpha, float beta, float theta);
AlphaBeta_Components inverseParkTransform(float d, float q, float theta);


#endif /* INC_APPMATH_H_ */
