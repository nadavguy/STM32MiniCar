/*
 * terminal.c
 *
 *  Created on: Jul 12, 2020
 *      Author: raing
 */
#include "main.h"
#include "string.h"
#include "ff.h"
#include "fatfs.h"
#include <stdio.h>

FRESULT FS_ret2;
FILINFO fno;
DIR dp;
uint16_t DisplayedCharIndex = 0;

void func_dir(void)
{
	int br = 0;
	FS_ret2 = f_opendir (&dp, "\\");
	do
	{

//		FS_ret2 = f_findnext(&dp,&fno);
		FS_ret2 = f_readdir(&dp, &fno);
		if (fno.fname[0] != 0)
		{
			memcpy(USBTXArray,fno.fname, sizeof(fno.fname));
			SendToScreen(true);
		}
		else
		{
			sprintf(USBTXArray,"%6.3f, EOD\r\n",CurrentTime());
			SendToScreen(false);
		}
//		memset(FileReadBuffer,0,sizeof(FileReadBuffer));

	} while (fno.fname[0] != 0);
}
void func_imp(void)
{
	sprintf(USBTXArray, "%6.3f, import command received\r\n",CurrentTime());
	SendToScreen(false);
	int b =1;
}

void func_read(void)
{
	do
	{
		HAL_Delay(1);
		FS_ret2 = f_open(&USERFile, "Index.txt", FA_READ);
	} while ( (FS_ret2 != FR_OK) );

	unsigned int br = 0;
	do
	{
		FS_ret2 = f_read(&USERFile, &FileReadBuffer, sizeof(FileReadBuffer), &br);
		memcpy(USBTXArray,FileReadBuffer, sizeof(FileReadBuffer));
		SendToScreen(false);
		memset(FileReadBuffer,0,sizeof(FileReadBuffer));
	} while (br != 0);
	FS_ret2 = f_close(&USERFile);
}

void func_fmt(void)
{
	uint8_t buffer[_MAX_SS];
	FS_ret2 = f_mkfs("\\", FM_FAT, 0, buffer, sizeof(buffer));
	sprintf(USBTXArray, "%6.3f, Disk Formated\r\n",CurrentTime());
	SendToScreen(false);
	FS_ret2 = f_open(&USERFile, "Index.txt", FA_CREATE_ALWAYS);
	sprintf(USBTXArray, "%6.3f, Created Index file\r\n",CurrentTime());
	SendToScreen(false);
	FS_ret2 = f_close(&USERFile);
	sprintf(USBTXArray, "%6.3f, Closed file\r\n",CurrentTime());
	SendToScreen(false);
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
					return i;
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
		{ "imp", func_imp },
		{ "read", func_read},
		{"fmt", func_fmt}
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

uint8_t funcTable( char* token )
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
	HAL_UART_DMAPause(&huart2);

	int LocalCounter = 0;
	uint8_t LocalRet = 2;

	for (int i = 0; i<64;i++)
	{
		if (USBRXArray[i] == 0)
		{
			memcpy(&USBRXArray[i],&USBRXArray[i + 1],63-i);
			UART5RXArray[63 - i] = 0;
		}
		else
		{
			LocalCounter++;
		}
	}
	int LocalIndex =  DisplayedCharIndex;
	for (int i = LocalIndex; i<strlen(USBRXArray);i++)
	{
		if (USBRXArray[i] != 0)
		{
			memcpy(&USBTXArray, &USBRXArray[i],1);
			SendToScreen(false);
			DisplayedCharIndex++;
		}
	}



	uint32_t NewLineIndex = 0;
	NewLineIndex = SearchString(USBRXArray,"\r\n");
	if ((strlen(USBRXArray) > 0) && (NewLineIndex != 0) )
	{

		char LocalCMD[10] = {0};
		strncpy(&LocalCMD,&USBRXArray,NewLineIndex);

		SendToScreen(true);
		LocalRet = funcTable(LocalCMD);
		if (!LocalRet)
		{
			sprintf(USBTXArray, "%6.3f, failed to execute command\r\n",CurrentTime());
			SendToScreen(false);
		}
		memset(&USBRXArray,0,sizeof(USBRXArray));
		DisplayedCharIndex = 0;

	}
	HAL_UART_DMAResume(&huart2);
//
}




