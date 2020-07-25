/*
 * terminal.c
 *
 *  Created on: Jul 12, 2020
 *      Author: raing
 */
#include "main.h"
#include "string.h"

void func_dir(void)
{
	sprintf(USBTXArray, "%6.3f, dir command received\r\n",CurrentTime());
	SendToScreen(false);
	int a= 1;
}
void func_imp(void)
{
	sprintf(USBTXArray, "%6.3f, import command received\r\n",CurrentTime());
	SendToScreen(false);
	int b =1;
}

uint32_t SearchString(char *pSrc, char *StringToLookFor)
{
	uint32_t LocalCounter = 0;
	bool StringFound = false;

	if (strlen(pSrc) > strlen(StringToLookFor))
	{
		for (int i = 0 ; i <= strlen(pSrc) - strlen(StringToLookFor); i++ )
		{
			if (StringFound)
			{
				break;
			}
			for (int j = 0; j < strlen(StringToLookFor); j++)
			{
				if (StringToLookFor[j] == pSrc[j+i])
				{
					LocalCounter++;
				}
				else
				{
					LocalCounter = 0;
					break;
				}
				if (strlen(StringToLookFor) == LocalCounter)
				{
					StringFound = true;
					return LocalCounter+1;
					continue;
				}
			}
		}
	}
	return 0;
}

typedef struct  { char* string; void (*func)(void); } stringcase;
//typedef struct { char *key; int val; } t_symstruct;
stringcase cases [] =
{
		{ "dir", func_dir },
		{ "imp", func_imp }
};


void SendToScreen(bool AddNewLine)
{
	HAL_UART_Transmit(&huart2, USBTXArray, 256,10); // HAL_UART_Transmit(&huart2, USBTXArray, 1024,3); TIM2->CCR1
	memset(USBTXArray,0, 256);
	if (AddNewLine)
	{
		HAL_UART_Transmit(&huart2, "\r\n", 2,1);
	}
}

uint8_t myswitch( char* token )
{
	for( stringcase* pCase = cases; pCase != cases + sizeof( cases ) / sizeof( cases[0] ); pCase++ )
	{
		if( 0 == strcmp( pCase->string, token ) )
		{
			(*pCase->func)();
			return 1;
			break;
		}
	}
	return 0;
}

void getCMD(void)
{
	HAL_UART_Receive_DMA(&huart2, USBRXArray, 64);

	int LocalCounter = 0;
	uint8_t LocalRet = 2;
	if (USBRXArray[0] == 0)
	{
		HAL_UART_DMAPause(&huart2);
		while ( (LocalCounter < sizeof(USBRXArray)) && (USBRXArray[0] == 0) )
		{
			memcpy(&USBRXArray[0], &USBRXArray[1],sizeof(USBRXArray)-1);
			memset(&USBRXArray[sizeof(USBRXArray)-1],0,1);
			LocalCounter++;
		}
		HAL_UART_DMAResume(&huart2);
	}
	if ((strlen(USBRXArray) > 0) && (strstr(USBRXArray, "\r\n")) )
	{
		HAL_UART_DMAPause(&huart2);
		uint32_t NewLineIndex = 0;
		NewLineIndex = SearchString(USBRXArray,"\r\n");
		char LocalCMD[10] = {0};
		strncpy(&LocalCMD,&USBRXArray,NewLineIndex);


		LocalRet = myswitch(LocalCMD);
		if (!LocalRet)
		{
			sprintf(USBTXArray, "%6.3f, failed to execute command\r\n",CurrentTime());
			SendToScreen(false);
		}
		memset(&USBRXArray,0,sizeof(USBRXArray));
		HAL_UART_DMAResume(&huart2);
	}
//
}




