/*
 * terminal.c
 *
 *  Created on: Jul 12, 2020
 *      Author: raing
 */
#include "main.h"
#include "string.h"

void SendToScreen(bool AddNewLine)
{
	HAL_UART_Transmit(&huart2, USBTXArray, 256,10); // HAL_UART_Transmit(&huart2, USBTXArray, 1024,3); TIM2->CCR1
	memset(USBTXArray,0, 256);
	if (AddNewLine)
	{
		HAL_UART_Transmit(&huart2, "\r\n", 2,1);
	}
}

void getCMD()
{

}

void ParseRCMessage(uint8_t *pData)
{
	int a = 1;
}
