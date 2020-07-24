/*
 * terminal.h
 *
 *  Created on: Jul 12, 2020
 *      Author: raing
 */

#ifndef PERIPHERALS_INC_TERMINAL_H_
#define PERIPHERALS_INC_TERMINAL_H_

extern void SendToScreen(bool AddNewLine);
extern void ParseRCMessage(uint8_t *pData);

#endif /* PERIPHERALS_INC_TERMINAL_H_ */
