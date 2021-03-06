/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/* 
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future. 
 * Kept to ensure backward compatibility with previous CubeMx versions when 
 * migrating projects. 
 * User code previously added there should be copied in the new user sections before 
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <includes.h>   
#include "ff_gen_drv.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Disk status */
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
//#include "QSPI_Flash.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SECTOR_SIZE                4096
/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;
uint32_t SecAdd ;
uint32_t Size;
uint32_t DSector;

uint8_t QSPI_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
uint8_t QSPI_Erase_Sector4K(uint32_t SectorAddress);
uint8_t QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size);

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);  
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read, 
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */  
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
    Stat = 0; //STA_NOINIT;
//	if(	QSPIFLASH_Init() == HAL_OK)
//	{       Stat &= ~STA_NOINIT;
//	}
//	else
//	{      // printf("DISK IO INIT ERROR \n");
//	}
    return Stat;
  /* USER CODE END INIT */
}
 
/**
  * @brief  Gets Disk Status 
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
//    Stat = 0 ; //STA_NOINIT;
//    return Stat;

//  QSPI_CommandTypeDef s_command;
//  uint8_t R1 = 1;
//  while(R1 & 0x01)
//  {
//      /* Initialize the reading of status register */
//      s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE; //QSPI_INSTRUCTION_4_LINES;
//      s_command.Instruction       = 0x05;
//      s_command.AddressMode       = QSPI_ADDRESS_NONE;
//      s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
//      s_command.DataMode          = QSPI_DATA_1_LINE; // QSPI_DATA_4_LINES;
//      s_command.DummyCycles       = 0;
//      s_command.NbData            = 1;
//      s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//      s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
//      s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
//      /* Configure the command */
//      if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
//      {
//        return HAL_ERROR;
//      }
//      /* Reception of the data */
//      if (HAL_QSPI_Receive(&hqspi, &R1, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
//      {
//        return HAL_ERROR;
//      }
//  }

  Stat = STA_NOINIT;
  Stat &= ~STA_NOINIT;
  	return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s) 
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
    	uint32_t SecAdd = sector * SECTOR_SIZE;
	uint32_t Size = count * SECTOR_SIZE;
//uint8_t QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)        
	if(QSPI_Read((uint8_t*)buff, (uint32_t)SecAdd, (uint32_t) Size ) ==  HAL_OK)
	{       return RES_OK;
	}
	else
	{       //printf("DISK Read Error \n");
		return RES_ERROR;
	}
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)  
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{ 
  /* USER CODE BEGIN WRITE */
	DSector = sector;
	SecAdd = sector * SECTOR_SIZE;
	Size = count * SECTOR_SIZE;
	for(uint16_t i = 0; i< count; i++)
	{       if(QSPI_Erase_Sector4K(SecAdd + (i * SECTOR_SIZE)) != HAL_OK)
		{	// printf("DISK IO ERASE FAIL \n");
			return RES_ERROR;
		}
	}
     
	if(QSPI_Write((uint8_t *)buff, SecAdd, Size) != HAL_OK)
	{       // printf("DISK IO WRITE FAIL \n");
		return RES_ERROR;
	}
  /* USER CODE HERE */
    return RES_OK;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation  
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
    DRESULT res = RES_ERROR;
  
  if (Stat & STA_NOINIT) return RES_NOTRDY;
  
  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = RES_OK;
    break;
  
  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
    *(DWORD*)buff = 4096; //SDRAM_DEVICE_SIZE / BLOCK_SIZE;
    res = RES_OK;
    break;
  
  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :
    *(WORD*)buff = SECTOR_SIZE;
    res = RES_OK;
    break;
  
  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
    *(DWORD*)buff = SECTOR_SIZE;
    break;
  
  default:
    res = RES_PARERR;
  }
	return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
