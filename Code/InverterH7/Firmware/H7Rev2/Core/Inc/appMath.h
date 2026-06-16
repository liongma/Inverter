/*
 * appMath.h
 *
 *  Created on: Aug 13, 2025
 *      Author: LiongMa
 */

#ifndef INC_APPMATH_H_
#define INC_APPMATH_H_

#include "stdio.h"

#include <math.h>
#include <stdio.h>
#include <stdint.h>
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

//structure for lin alg
typedef struct {
    float m[2][2];
} mat22;

typedef struct {
    float m[2];
} mat21;

typedef struct {
    float m[4];
} mat41;

typedef struct {
    float m[4][4];
} mat44;

// CORDIC FUNCTIONS
float cordic_sin(float theta);
void cordic_sin_cos(float theta, float* s, float* c);
int32_t rad_to_cordic_q31(float theta_rad);
float wrap_angle_cordic(float theta);
void appMath_Init();

// matrix function
mat41 mat41_sca(mat41 * a, float b);

mat21 mat21_add(mat21 * a, mat21 * b);
mat21 mat21_sub(mat21 * a, mat21 * b);
mat21 mat21_mul(mat22 * a, mat21 * b);
mat21 mat21_sca(mat21 * a, float b);

mat22 mat_add(mat22 * a, mat22 * b);
mat22 mat_sub(mat22 * a, mat22 * b);
mat22 mat_mul(mat22 * a, mat22 * b);
float mat_det(mat22 * a);
mat22 mat_inv(mat22 * a);
mat22 mat_tra(mat22 * a);
mat22 mat_sca(mat22 * a, float scalar);


// Functions

float clip(float val, float min, float max);
AlphaBeta_Components sat_ab_circle(AlphaBeta_Components v, float vmax);
AlphaBeta_Components sat_ab_hexagon(AlphaBeta_Components v_ab, float v_bus); //prototype for the hexagonal saturation function
DQ_Components sat_dq_circle(DQ_Components v, float vmax);

float normalizeAngle(float theta);
float shortenAngle(float theta);

AlphaBeta_Components clarkeTransform(float a, float b, float c);
ThreePhase_Components inverseClarkeTransform(float alpha, float beta);

DQ_Components parkTransform(float alpha, float beta, float theta);
AlphaBeta_Components inverseParkTransform(float d, float q, float theta);


#endif /* INC_APPMATH_H_ */
