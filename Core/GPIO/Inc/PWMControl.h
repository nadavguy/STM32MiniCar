/*
 * PWMControl.h
 *
 *  Created on: Jul 9, 2020
 *      Author: raing
 */

#ifndef GPIO_INC_PWMCONTROL_H_
#define GPIO_INC_PWMCONTROL_H_
extern void start_pwm1(int onTimemSec);
extern void stop_pwm1(void);

extern void start_pwm2(int onTimemSec);
extern void stop_pwm2(void);

extern void start_pwm3(int onTimemSec);
extern void stop_pwm3(void);

extern void start_pwm4(int onTimemSec);
extern void stop_pwm4(void);

#endif /* GPIO_INC_PWMCONTROL_H_ */
