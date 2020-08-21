/*
 * terminal.h
 *
 *  Created on: Jul 12, 2020
 *      Author: raing
 */

#ifndef PERIPHERALS_INC_TERMINAL_H_
#define PERIPHERALS_INC_TERMINAL_H_

extern void Print(bool AddNewLine,bool SendToLog, bool SendToScreen);
extern void getCMD(void);
extern void func_dir(void);
extern void func_imp(void);
extern void func_read(void);
extern void func_fmt(void);
extern void func_flush(void);
extern int32_t SearchString(uint8_t *pSrc, uint8_t *StringToLookFor);
extern uint8_t funcTable( char* token );

#endif /* PERIPHERALS_INC_TERMINAL_H_ */
