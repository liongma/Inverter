/*
 * mtpva.h
 *
 *  Created on: Dec 10, 2025
 *      Author: lionm
 */

#ifndef INC_MTPVA_H_
#define INC_MTPVA_H_

float mtpva_q(float i_mag, float omega_e, float v);
float mtpva_d(float i_mag, float omega_e, float v);
void compute_Fd_Fq(float Id, float Iq, float *Fd, float *Fq);
#endif /* INC_MTPVA_H_ */
