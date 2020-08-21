/*
 * PWMControl.h
 *
 *  Created on: Jul 9, 2020
 *      Author: raing
 */

#ifndef GPIO_INC_PWMCONTROL_H_
#define GPIO_INC_PWMCONTROL_H_

extern uint32_t IC_Value1;
extern uint32_t IC_Value2;
extern uint32_t Difference;
extern uint32_t Frequency;
extern uint8_t Is_First_Captured;  // 0- not captured, 1- captured
extern bool NewDiffAvailable;


extern void start_pwm1(int onTimemSec);
extern void set_pwm1(int onTimemSec);
extern void stop_pwm1(void);

extern void start_pwm2(int onTimemSec);
extern void set_pwm2(int PercentOn);
extern void stop_pwm2(void);

extern void start_pwm3(void);
extern void stop_pwm3(void);

extern void start_pwm4(int onTimemSec);
extern void stop_pwm4(void);

extern void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif /* GPIO_INC_PWMCONTROL_H_ */
