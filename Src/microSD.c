#include "microSD.h"

/******* FATFS *********/
 FATFS fileSystem;
  FIL testFile;
  uint8_t testBuffer[16] = "SD write success";
  UINT testBytes;
  FRESULT res;
/***********************/

void testSDCard(){
	res = f_mount(&fileSystem, SD_Path, 1); 
	
	if( res == FR_OK)
  {
		USB_Print("f_mount - OK!\n");
		
    uint8_t path[13] = "testfile.txt";
    path[12] = '\0';
 
    res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
		if( res != FR_OK){
			USB_Print("ERROR - ");
			USB_SendNumber(res);
			USB_Print(" can't f_open! \n");
		}
		else
			USB_Print("f_open - OK!\n");
		
    res = f_write(&testFile, testBuffer, 16, &testBytes);
		if( res != FR_OK){
			USB_Print("ERROR - ");
			USB_SendNumber(res);
			USB_Print(" can't f_open! \n");
		}
		else
			USB_Print("f_write - OK!\n");
		
    res = f_close(&testFile);
		if( res != FR_OK){
			USB_Print("ERROR - ");
			USB_SendNumber(res);
			USB_Print(" can't f_close! \n");
		}
		else
			USB_Print("f_close - OK!\n");
  }
	else{
		USB_Print("ERROR - ");
		USB_SendNumber(res);
		USB_Print(" can't mount FS! \n");
	}
}



