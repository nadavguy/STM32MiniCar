/*
 * ms5611.h
 *
 *  Created on: Jul 11, 2020
 *      Author: raing
 */

#ifndef PERIPHERALS_INC_MS5611_H_
#define PERIPHERALS_INC_MS5611_H_

extern int32_t TEMP;
extern int32_t P;
extern bool isNewMS56XXDataAvailable;

extern void MS56XXReset(void);
extern uint16_t MS56XXReadProm(uint8_t address);
extern void MS56XXInit(void);
extern void MS56XXSendCmd(uint8_t Cmd);
extern uint32_t MS56XXRead3Bytes(uint8_t address);
extern void MS56XXCyclicRead(void);
uint16_t GetAltitudeAndTemp(void);
#endif /* PERIPHERALS_INC_MS5611_H_ */
