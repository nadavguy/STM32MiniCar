/*
 * ms5611.c
 *
 *  Created on: Jul 11, 2020
 *      Author: raing
 */
#include "main.h"
#include "ms5611.h"
uint8_t SPITX[1] = {0};
uint8_t SPIRX2Byte[2] = {0};
uint8_t SPIRX3Byte[3] = {0};
uint16_t Coeff1 = 0;
uint16_t Coeff2 = 0;
uint16_t Coeff3 = 0;
uint16_t Coeff4 = 0;
uint16_t Coeff5 = 0;
uint16_t Coeff6 = 0;
uint16_t MSCRC = 0;
bool isCmdSet = false;
bool isPressureLastCmd = false;
bool isNewMS56XXDataAvailable = false;
uint32_t LastTempMeasurement = 0;
uint32_t LastPressureMeasurement = 0;
uint32_t LastCommandSent = 0;
uint32_t RawTemp = 0;
uint32_t RawPressure = 0;

int32_t dT = 0;
int32_t TEMP = 0;
int64_t OFF = 0;
int64_t SNES = 0;
int32_t P = 0;

void MS56XXReset(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	SPITX[0] = 0x1E;
	HAL_SPI_Transmit(&hspi1, &SPITX[0], 1, HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

uint16_t MS56XXReadProm(uint8_t address)
{
	SPITX[0] = address;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &SPITX[0], 1, HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);
	HAL_SPI_Receive(&hspi1, &SPIRX2Byte[0], 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, &SPIRX2Byte[1], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	return (SPIRX2Byte[0]<<8) + SPIRX2Byte[1];
}

void MS56XXInit(void)
{
	MS56XXReset();
	HAL_Delay(50);
	MS56XXReadProm(0xA0);
	Coeff1 = MS56XXReadProm(0xA2);
	Coeff2 = MS56XXReadProm(0xA4);
	Coeff3 = MS56XXReadProm(0xA6);
	Coeff4 = MS56XXReadProm(0xA8);
	Coeff5 = MS56XXReadProm(0xAA);
	Coeff6 = MS56XXReadProm(0xAC);
	MSCRC = MS56XXReadProm(0xAE);

	MS56XXSendCmd(0x58);
	HAL_Delay(9);
	RawTemp = MS56XXRead3Bytes(0);

	MS56XXSendCmd(0x48);
	HAL_Delay(9);
	RawPressure = MS56XXRead3Bytes(0);
	GetAltitudeAndTemp();
}

void MS56XXSendCmd(uint8_t Cmd)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	SPITX[0] = Cmd;
	HAL_SPI_Transmit(&hspi1, &SPITX[0], 1, HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	isCmdSet = true;
}

uint32_t MS56XXRead3Bytes(uint8_t address)
{
	SPITX[0] = 0x00;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &SPITX[0], 1, HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);
	HAL_SPI_Receive(&hspi1, &SPIRX3Byte[0], 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, &SPIRX3Byte[1], 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, &SPIRX3Byte[2], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	isCmdSet = false;
	return (SPIRX3Byte[0] << 16) + (SPIRX3Byte[1] << 8 ) + (SPIRX3Byte[0]);
}

uint16_t GetAltitudeAndTemp(void)
{
	dT = RawTemp - Coeff5 * 256;
	TEMP = 2000 + dT * Coeff6 / 8388608;
	OFF = Coeff2 * 131072 + Coeff4 * dT / 64;
	SNES = Coeff1 * 32768 + (Coeff3 * dT) / 127;
	P = ((RawPressure * SNES) / 2097152 - OFF) / 32768;
	return P;
}

void MS56XXCyclicRead(void)
{
	if ( (HAL_GetTick() - LastTempMeasurement) > 1000 )
	{
		if (!isCmdSet)
		{
			MS56XXSendCmd(0x58);
			isCmdSet = true;
			isNewMS56XXDataAvailable = false;
			isPressureLastCmd = false;
			LastCommandSent = HAL_GetTick();
		}
	}

	if ((HAL_GetTick() - LastPressureMeasurement) > 20)
	{
		if (!isCmdSet)
		{
			MS56XXSendCmd(0x48);
			isCmdSet = true;
			isNewMS56XXDataAvailable = false;
			isPressureLastCmd = true;
			LastCommandSent = HAL_GetTick();
		}
	}

	if ( (HAL_GetTick() - LastCommandSent) > 9 )
	{
		if (isCmdSet)
		{
			if (!isPressureLastCmd)
			{
				RawTemp = MS56XXRead3Bytes(0);
				LastTempMeasurement = HAL_GetTick();
			}
			else
			{
				RawPressure = MS56XXRead3Bytes(0);
				LastPressureMeasurement = HAL_GetTick();
			}
			isCmdSet = false;
			GetAltitudeAndTemp();
			isNewMS56XXDataAvailable = true;
		}
	}
}
