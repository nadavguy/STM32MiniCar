// Module..: FS_DISPLAY.C
// Chip....: STM32F4xx
//-----------------------------------------------------------------------------

#include "fs_display.h"
//-----------------------------------------------------------------------------

DIR dir;    // Directory object
FILINFO fi; // File information object
//-----------------------------------------------------------------------------

static void fs_disp_file_info(FILINFO*);
//-----------------------------------------------------------------------------

void 
fs_disp_result(FRESULT result)
{
	int a =1;
  switch (result)
  {
  case FR_OK:
//    g_term.putstr("FS_OK");
    break;
  default:
	  a =1;
//    g_term.putstr("FS_UNKNOWN");
  }
}
//-----------------------------------------------------------------------------

// Display Files and Directories
void fs_disp_dir(uint8_t* path)
{
  FRESULT res;
//  DIR dj;         // Directory search object
//            f_findfirst(&dj, &fi, "", "*.*");  // Start to search first files
//            while (fr == FR_OK && fi.fname[0]) 
//            {   fr = f_findnext(&dj, &fi);               /* Search for next item */
//                fs_disp_file_info(&fi);
//                g_term.putstr("\r\n");
//            }    
  
//  g_term.putstr("dir EOD\r\n");
  
  res = f_opendir(&dir, (const char*)path);
  if (res != FR_OK)
  {
    fs_disp_result(res);
    return;
  }
//  g_term.putstr("\r\n");
  while (((res = f_readdir(&dir, &fi)) == FR_OK) && fi.fname[0])
  { HAL_Delay(5);
 //   g_term.putstr("dir ");
    fs_disp_file_info(&fi);

//    g_term.putstr("\r\n");
  }
//  g_term.putstr("dir EOD\r\n");
}
//-----------------------------------------------------------------------------

// Display file name
static void fs_disp_file_info(FILINFO* fi)
{
  // Display the file name
//  g_term.putstr((char*)fi->fname);

  // Display file size
//  g_term.putchr('\t');
//  g_nc.U32s(term_putc, fi->fsize, 10, false);
  
  // Display dot.dot if it is a DIR entry
  if (fi->fattrib & AM_DIR && strlen((char *)fi->fname) == 0)
  {
//    g_term.putchr('.');
//    g_term.putchr('.');
  }
}
