/*
 * terminal.h
 *
 *  Created on: Jul 12, 2020
 *      Author: raing
 */

#ifndef PERIPHERALS_INC_TERMINAL_H_
#define PERIPHERALS_INC_TERMINAL_H_

extern void SendToScreen(bool AddNewLine);
extern void getCMD(void);
extern void func_dir(void);
extern void func_imp(void);
extern uint32_t SearchString(uint8_t *pSrc, uint8_t *StringToLookFor);
extern uint8_t myswitch( char* token );

#endif /* PERIPHERALS_INC_TERMINAL_H_ */
