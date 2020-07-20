#ifndef __FS_DISPLAY_H__
#define __FS_DISPLAY_H__
//-----------------------------------------------------------------------------

#include "diskio.h"
#include "ff.h"
#include "fs_display.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
//#include "stm32f7xx_ll_rcc.h"
#include <string.h>
#include <stdio.h>
#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"

//-----------------------------------------------------------------------------

#define SD_BLOCK_SIZE (512)
#define SD_BUFFER_BLOCKS (16)
#define SD_BUFFER_SIZE (SD_BLOCK_SIZE * SD_BUFFER_BLOCKS)
//-----------------------------------------------------------------------------

extern DIR dir; // Directory object
extern FILINFO fi; // File information object
//-----------------------------------------------------------------------------

void fs_disp_result(FRESULT);
void fs_disp_dir(pU08);

#endif
