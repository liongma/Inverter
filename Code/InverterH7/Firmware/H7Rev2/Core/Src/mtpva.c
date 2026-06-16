/*
 * mtpva.c
 *
 *  Created on: Dec 10, 2025
 *      Author: lionm
 */


#include "mtpva.h"
#include "lut.h"
#include "math.h"
float mtpva_q(float i_mag, float omega_e, float v){
	return cosf(iwv[(int)i_mag/5][(int)fabs(omega_e)/500][(int)v/50 + 1]/320.0f);
}

float mtpva_d(float i_mag, float omega_e, float v){
	return -sinf(iwv[(int)i_mag/5][(int)fabs(omega_e)/500][(int)v/50 + 1]/320.0f);
}

void compute_Fd_Fq(float Id, float Iq, float *Fd, float *Fq)
{
    float d = Id / 140.0f;   // 'f' suffix keeps it in float — without it,
    float q = Iq / 160.0f;   // the division promotes to double temporarily

    float d2 = d  * d;
    float d3 = d2 * d;
    float d4 = d3 * d;

    float q2 = q  * q;
    float q3 = q2 * q;
    float q4 = q3 * q;
    float q5 = q4 * q;

    *Fd =  2.754577e-02f
         + q2  * (-1.074559e-02f + d2 *  9.511528e-03f)
         + q4  *  1.610519e-03f
         + d   * ( 3.709562e-02f + q2 * -1.067418e-02f)
         + d2  * -2.697243e-02f
         + d3  * -7.946521e-03f
         + d4  *  6.195936e-03f;

    *Fq = q  * ( 5.393741e-02f + d  * (-2.454568e-02f + d * (-1.793721e-02f + d * ( 1.237522e-02f + d *  6.329816e-03f))))
        + q3 * (-2.765651e-02f + d  * ( 1.559723e-02f  + d *  1.608109e-02f))
        + q5 *  9.237308e-03f;
}
