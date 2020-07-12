/*
 * PushButton.h
 *
 *  Created on: Jul 12, 2020
 *      Author: raing
 */

#ifndef GPIO_INC_PUSHBUTTON_H_
#define GPIO_INC_PUSHBUTTON_H_

extern bool ButtonIsHigh;
extern bool ButtonIsLow;
extern GPIO_PinState PA0PinState;
extern uint32_t ButtonPressStart;
extern uint32_t ButtonPressCycleStart;
extern uint32_t ButtonPressDurationmSec[5];
extern uint8_t ButtonCycle;

extern void CheckButton(void);

#endif /* GPIO_INC_PUSHBUTTON_H_ */
