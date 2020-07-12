/*
 * terminal.c
 *
 *  Created on: Jul 12, 2020
 *      Author: raing
 */
#include "main.h"

void SendToScreen(void)
{
	HAL_UART_Transmit(&huart2, USBTXArray, 150,4); // HAL_UART_Transmit(&huart2, USBTXArray, 1024,3); TIM2->CCR1
	memset(USBTXArray,0, 150);
}
