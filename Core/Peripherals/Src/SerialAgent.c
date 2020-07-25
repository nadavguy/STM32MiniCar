/*
 * SerialAgent.c
 *
 *  Created on: Jul 22, 2020
 *      Author: raing
 */

#include "main.h"
#include "string.h"

uint8_t GeneralHeader[8] = {'A','B','5','6','F','E','2','1'};
uint8_t LocalUART5RXArray[256] = {0};

uint8_t ParseRFMessage(uint8_t *Angle, uint8_t *Power)
{
//	AB56FE21,7,20580,#0,90!~
	int ret = 0;
	int HeaderStartIndex = 0;
	int HeaderStopIndex = 0;
	int MessageEndIndex = 0;
	int HashTagIndex = 0;
	int CommaIndex = 0;
	int LocalCounter = 0;
	bool HeaderFound = false;
	bool MessageEndFound = false;
	bool HashFound = false;
	bool CommaFound = false;

	*Angle = 90;
	*Power = 0;
	if (strlen(UART5RXArray) > strlen(GeneralHeader))
	{
		for (int i = 0 ; i< strlen(UART5RXArray) - strlen(GeneralHeader); i++ )
		{
			if (HeaderFound)
			{
				break;
			}
			for (int j = 0; j < strlen(GeneralHeader); j++)
			{
				if (GeneralHeader[j] == UART5RXArray[j+i])
				{
					LocalCounter++;
				}
				else
				{
					LocalCounter = 0;
					break;
				}
				if (strlen(GeneralHeader) == LocalCounter)
				{
					HeaderStartIndex = i;
					HeaderStopIndex = i + strlen(GeneralHeader);
					HeaderFound = true;
					continue;
				}
			}
		}
	}


	if (HeaderFound)
	{
		for (int i = 0 ; i< strlen(UART5RXArray); i++ )
		{
			if ( (UART5RXArray[i] == '#') && (!HashFound) )
			{
				HashFound = true;
				HashTagIndex = i;
			}
			if ( (UART5RXArray[i] == ',') && (HashFound) && (!CommaFound))
			{
				CommaFound = true;
				CommaIndex = i;
			}
			if ( (UART5RXArray[i] == '!') && (CommaFound) && (!MessageEndFound) )
			{
				MessageEndFound = true;
				MessageEndIndex = i;
			}
		}
	}
	if ((HashFound) && (HashFound) && (CommaFound) )
	{
		*Angle = 0;
		*Power = 0;
		for (int i = 0; i < CommaIndex - HashTagIndex -1 ; i++)
		{
			*Power = *Power * 10 + UART5RXArray[HashTagIndex + i + 1] - '0';
		}
		for (int i = 0; i < MessageEndIndex - CommaIndex -1; i++)
		{
			*Angle = *Angle * 10 + UART5RXArray[CommaIndex + i + 1] - '0';
		}
//		sprintf(USBTXArray, "%6.3f, ",CurrentTime());
//		SendToScreen(false);
//		memcpy(USBTXArray,UART5RXArray,MessageEndIndex);
//		SendToScreen(true);
		memcpy(&UART5RXArray[0],&UART5RXArray[MessageEndIndex+1],255-MessageEndIndex);
		memset(&UART5RXArray[255-MessageEndIndex],0,MessageEndIndex+1);
		return 0;
	}
//	sprintf(USBTXArray, "%6.3f, Error Code",CurrentTime());
//	SendToScreen(false);
	return 1;
}


uint32_t ReadDataFromUART(void)
{
	int NumberOfBytesRead = 0;
	int LocalCounter = 0;
	HAL_StatusTypeDef Uart_Ret;
	Uart_Ret = HAL_UART_Receive(&huart5, UART5RXArray, 256,5);
	while (UART5RXArray[LocalCounter] != 0)
	{
		LocalCounter++;
	}
	NumberOfBytesRead = LocalCounter;
	return NumberOfBytesRead;
}

uint32_t CheckDataFromUART(void)
{
	int NumberOfBytesRead = 0;
	int LocalCounter = 0;
	int StepInc = 0;
	int ret = 0;
	HAL_StatusTypeDef Uart_Ret;
	HAL_UART_Receive_DMA(&huart5, LocalUART5RXArray, 256);
	HAL_UART_DMAPause(&huart5);
	memcpy(&UART5RXArray, &LocalUART5RXArray,256);
	memset(LocalUART5RXArray,0,256);
	HAL_UART_DMAResume(&huart5);
	for (int i = 0; i<256;i++)
	{
		if (UART5RXArray[0] == 0)
		{
			memcpy(&UART5RXArray[StepInc],&UART5RXArray[StepInc+1],255-i);
			UART5RXArray[255 - i] = 0;
		}
		else
		{
			StepInc++;
		}
	}
	while (UART5RXArray[LocalCounter] != 0)
	{
		LocalCounter++;
	}
	NumberOfBytesRead = LocalCounter;
	return NumberOfBytesRead;
}

