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
//			memcpy(USBTXArray,fno.fname, sizeof(fno.fname));
			sprintf(USBTXArray, "file: %s, size: %d\r\n",fno.fname, fno.fsize);
			Print(false, false, true);
		}
		else
		{
			sprintf(USBTXArray,"%6.3f, EOD\r\n",CurrentTime());
			Print(false, false, true);
		}
		//		memset(FileReadBuffer,0,sizeof(FileReadBuffer));

	} while (fno.fname[0] != 0);
}
void func_imp(void)
{
	sprintf(USBTXArray, "%6.3f, import command received\r\n",CurrentTime());
	Print(false, true, true);
}

void func_read(void)
{
	FS_ret2 = f_close(&USERFile);
	do
	{
		HAL_Delay(1);
		FS_ret2 = f_open(&USERFile, "Index.txt", FA_READ);
	} while ( (FS_ret2 != FR_OK) );

	unsigned int br = 0;
	do
	{
		FS_ret2 = f_read(&USERFile, &FileReadBuffer, sizeof(FileReadBuffer), &br);
		memcpy(&USBTXArray,FileReadBuffer, br);
		Print(false, false, true);
		memset(&FileReadBuffer,0,sizeof(FileReadBuffer));
	} while (br != 0);
	FS_ret2 = f_close(&USERFile);
}

void func_write(void)
{
	do
	{
		HAL_Delay(1);
		FS_ret2 = f_open(&USERFile, "Index.txt", FA_OPEN_APPEND | FA_WRITE);
	} while ( (FS_ret2 != FR_OK) );
	ActiveLog = true;
}

void func_close(void)
{
	ActiveLog = false;
//	FS_ret2 = f_sync(&USERFile);
	FS_ret2 = f_close(&USERFile);
}

void func_flush(void)
{
	FS_ret2 = f_sync(&USERFile);
}

void func_fmt(void)
{
	uint8_t buffer[_MAX_SS];
	FS_ret2 = f_mkfs("\\", FM_FAT, 0, buffer, sizeof(buffer));
	sprintf(USBTXArray, "%6.3f, Disk Formated\r\n",CurrentTime());
	Print(false, false, true);
	FS_ret2 = f_open(&USERFile, "Index.txt", FA_CREATE_ALWAYS);
	sprintf(USBTXArray, "%6.3f, Created Index file\r\n",CurrentTime());
	Print(false, false, true);
	FS_ret2 = f_close(&USERFile);
	sprintf(USBTXArray, "%6.3f, Closed file\r\n",CurrentTime());
	Print(false, false, true);
}

void func_debug(void)
{
	isDebugMode = !isDebugMode;
	if (isDebugMode)
	{
		sprintf(USBTXArray, "%6.3f, Debug mode active\r\n",CurrentTime());
	}
	else
	{
		sprintf(USBTXArray, "%6.3f, Debug mode deactivated\r\n",CurrentTime());
	}
	Print(false, false, true);
}

int32_t SearchString(char *pSrc, char *StringToLookFor)
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
	return -1;
}

typedef struct  { char* string; void (*func)(void); } stringcase;
//typedef struct { char *key; int val; } t_symstruct;
stringcase cases [] =
{
		{ "dir", func_dir },
		{ "imp", func_imp },
		{ "read", func_read },
		{ "write", func_write },
		{ "close", func_close },
		{ "debug", func_debug },
		{ "flush", func_flush },
		{ "fmt", func_fmt }
};


void Print(bool AddNewLine, bool SendToLog, bool SendToScreen)
{
	if (AddNewLine)
	{
		strcat(&USBTXArray,"\r\n");
	}
	if (SendToScreen)
	{
		HAL_UART_Transmit(&huart2, USBTXArray, strlen(USBTXArray),HAL_MAX_DELAY); // HAL_UART_Transmit(&huart2, USBTXArray, 1024,3); TIM2->CCR1
	}
	if (ActiveLog)
	{
		BytesWritten = 0;
		FS_ret2 = f_write(&USERFile, USBTXArray, strlen(USBTXArray), &BytesWritten);
//		f_puts(USBTXArray, &USERFile);
	}
	memset(&USBTXArray,0, strlen(USBTXArray));

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
//	int LocalIndex =  DisplayedCharIndex;
//	for (int i = LocalIndex; i<strlen(USBRXArray);i++)
//	{
//		if (USBRXArray[i] != 0)
//		{
//			memcpy(&USBTXArray, &USBRXArray[i],1);
//			Print(false, false, true);
//			DisplayedCharIndex++;
//		}
//	}



	int32_t NewLineIndex = 0;
	NewLineIndex = SearchString(USBRXArray,"\r\n");
	if ((strlen(USBRXArray) > 0) && (NewLineIndex > 0) )
	{

		char LocalCMD[10] = {0};
		strncpy(&LocalCMD,&USBRXArray,NewLineIndex);

//		Print(true, false, true);
		LocalRet = funcTable(LocalCMD);
		if (!LocalRet)
		{
			sprintf(USBTXArray, "%6.3f, failed to execute command\r\n",CurrentTime());
			Print(false, true, true);
		}
		memset(&USBRXArray,0,sizeof(USBRXArray));
		DisplayedCharIndex = 0;

	}
	else if ((strlen(USBRXArray) > 0) && (NewLineIndex == 0))
	{
//		memcpy(&USBRXArray[0],&USBRXArray[2],2);
		memset(&USBRXArray,0,2);
	}
	HAL_UART_DMAResume(&huart2);
	//
}




