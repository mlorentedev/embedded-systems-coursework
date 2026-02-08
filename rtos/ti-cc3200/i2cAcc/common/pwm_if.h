/*
 * pwm_if.h
 *
 *  Created on: 6 de feb. de 2017
 *      Author: cano
 */

#ifndef COMMON_PWM_IF_H_
#define COMMON_PWM_IF_H_

void PWM_IF_Init(unsigned long ulValue);
void PWM_IF_OutputEnable();
void PWM_IF_OutputDisable();
void PWM_IF_SetDutycycle(unsigned long red, unsigned long green, unsigned long orange);



#endif /* COMMON_PWM_IF_H_ */
