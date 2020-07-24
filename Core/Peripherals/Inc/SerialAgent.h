/*
 * SerialAgent.h
 *
 *  Created on: Jul 22, 2020
 *      Author: raing
 */

#ifndef PERIPHERALS_INC_SERIALAGENT_H_
#define PERIPHERALS_INC_SERIALAGENT_H_

extern uint8_t LocalUART5RXArray[256];

extern uint32_t ReadDataFromUART(void);
extern uint32_t CheckDataFromUART(void);
extern uint8_t ParseRFMessage(uint8_t *Angle, uint8_t *Power);

#endif /* PERIPHERALS_INC_SERIALAGENT_H_ */
