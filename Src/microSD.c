#include "microSD.h"
#include "ADS1298.h"
#include "MPU9250.h"
#include "BME280.h"

#include "stdbool.h"

#define SD_BUFF 2048

bool existSD = false;

bool changeBufFlag = false;
bool writeFlag = false;

uint8_t buff_1[SD_BUFF];
uint32_t ptr = 0;
uint8_t buff_2[SD_BUFF];

/******* FATFS *********/
 FATFS fileSystem;
 FIL testFile;
 FILINFO fno;
 uint8_t testBuffer[16] = "SD write success";
 UINT testBytes;
 FRESULT res;
 DIR dir;
 
 	uint8_t path[12]; //Name of file
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

void SD_Init(){
	int numFiles = 0;

	
	if(verbose)
		HAL_Delay(5000);
	printDebug("\nStart init SD Card!\n");
	
	res = f_mount(&fileSystem, SD_Path, 1); 
	if(res == FR_OK){
		printDebug("\nSD card mounting - OK\n");
		
		// --------------- Open Dir ---------------
		res = f_opendir(&dir, "");
		if(res != FR_OK){
			printDebug("F_OpenDir - ERROR!\n");
			existSD = false;
			return;
		}else{
			printDebug("F_OpenDir - OK\n");
		}
		// --------- Counting files in folder -----
		for(;;){
				res = f_readdir(&dir,&fno);
				if (res != FR_OK || fno.fname[0] == 0) 
					break;
				numFiles++;				
		}
		
		if(verbose){
			USB_Print("Number files in folder= ");
			USB_SendNumber(numFiles);
			USB_Print("\n");
		}
		
		
		// -------------- Open File --------------
		if(++numFiles<10)
			sprintf((char*)&path,"Data_0%d.txt",numFiles);
		else
			sprintf((char*)&path,"Data_%d.txt",numFiles);
		
		if(verbose){
			USB_Print("Name of file: ");
			USB_Print(path);
			USB_Print("\n");
		}

    path[11] = '\0';
		
		res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
		if( res != FR_OK){
			printDebug("F_Open file on SD - ERROR!\n");
			existSD = false;
			return;
		}else{
			printDebug("F_Open file on SD - OK\n");
		}

		existSD = true;
		
		writeStringSD("Ch1\tCh2\tCh3\tCh4\tCh5\tCh6\tCh7\tCh8\tAx\tAy\tAz\tTemp\tPres\tHumid\n");
	}
	else{
		printDebug("SD card mounting - ERROR!\n");
		existSD = false;
		return;
	}
	//f_close(&testFile);
}


void printDebug(unsigned char* string){
	if(verbose){
		USB_Print(string);
	}
}


void writeSD(){
	int _i = 0;
	int32_t* dataADS;
	int32_t* Acc;
	int32_t* Meteo;
	
	if(existSD){
		// Write 8 channel ECG
		dataADS = getChannelData();
		for(_i = 0; _i < 8; _i++){
			writeSD4Byte(dataADS[_i]);
			writeStringSD("\t");
		}
	
		// Write accelerations
		Acc = getAccel();
		for(_i = 0; _i < 3; _i++){
			//if(Acc[_i] & 0x00008000 == 0x00008000){ 
			//	writeStringSD("-");
			//	writeSD4Byte(0x0000FFFF-Acc[_i]);
			//}else
				writeSD4Byte(Acc[_i]);
			writeStringSD("\t");
		}
		
		// Write temperature, preassure, humidity
		Meteo = getMeteo();
		for(_i = 0; _i < 3; _i++){
			writeSD4Byte(Meteo[_i]);
			writeStringSD("\t");
		}
		
		writeSD4Byte( HAL_GetTick());
		writeStringSD("\n");
	
		if(writeFlag){
			writeFlag = false;
			f_writeRun();
		}
	}
	else{
		SD_Init();
	}
}

void writeSD4Byte(int32_t d){
	uint8_t value[10]; //a temp array to hold results of conversion
  int i = 0, k = 0; //loop index
  
  do
  {
    value[i++] = (char)(d % 10) + '0'; //convert integer to character
		d /= 10;
  } while(d);
  
	for(k = i - 1; k >= 0; k--){
		//while( (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9) == GPIO_PIN_SET) && (CDC_Transmit_FS(&value[k],1) == USBD_BUSY));
		//writeNumberSD(value[k]);
		if(changeBufFlag)
			buff_1[ptr] = value[k];
		else
			buff_2[ptr] = value[k];
		
		ptr++;
		if(ptr == SD_BUFF){
			changeBufFlag = !changeBufFlag; // Change buffer
			writeFlag = true;
			ptr = 0;
		}
	}
}

void write4NumberSD(uint32_t val){
	char value[5]={'0','0','0','0','0'}; //a temp array to hold results of conversion
  int i = 4; //loop index
  
  do{
    value[i--] = (char)(val % 10) + '0'; //convert integer to character
    val /= 10;
  } while(val);
	
	//i++;
	i = 1;
	
	while(i<5){
		if(changeBufFlag)
			buff_1[ptr] = value[i];
		else
			buff_2[ptr] = value[i];
		
		ptr++; i++;
		if(ptr == SD_BUFF){
			changeBufFlag = !changeBufFlag; // Change buffer
			writeFlag = true;
			ptr = 0;
		}
	}
	//f_write(&fil,value+i+1,4-i,&bw);  // was here!!!
}

void writeNumberSD(uint32_t val){
	char value[5]={0,0,0,0,0}; //a temp array to hold results of conversion
  int i = 4; //loop index
  
  do{
    value[i--] = (char)(val % 10) + '0'; //convert integer to character
    val /= 10;
  } while(val);
	
	i++;
	while(i<5){
		if(changeBufFlag)
			buff_1[ptr] = value[i];
		else
			buff_2[ptr] = value[i];
		
		ptr++; i++;
		if(ptr == SD_BUFF){
			changeBufFlag = !changeBufFlag; // Change buffer
			writeFlag = true;
			ptr = 0;
		}
	}
	//f_write(&fil,value+i+1,4-i,&bw);  // was here!!!
}

/*-------------------------------------------*/
// Write array of chars to SD card
/*-------------------------------------------*/
void writeStringSD(char* str){ // should be added '\0' to the end of string (string means array of chars here)
	//int i=0;                     // writeStringSD("fsdfdsfsdf") will be working OK!
	//while(str[i++]!='\0'){
	while( *str ){
		if(changeBufFlag)
			buff_1[ptr] = *str++;
		else
			buff_2[ptr] = *str++;
	
		ptr++; 
		if(ptr == SD_BUFF){
			changeBufFlag = !changeBufFlag; // Change buffer
			writeFlag = true;
			ptr = 0;
		}
	}
	//if (i>1)
		//f_write(&fil,str,i-1,&bw);
}

void f_writeRun(){	
	if(!changeBufFlag){
		res = f_write(&testFile,buff_1,SD_BUFF,&testBytes);	
	}else{
		res = f_write(&testFile,buff_2,SD_BUFF,&testBytes);
	}
	if(res !=  FR_OK){
		reOpenFile();
	}
		
	res = f_sync(&testFile);
	if(res != FR_OK){
		reOpenFile();
	}
}

void reOpenFile(){
	res = f_close(&testFile);
	if(res != FR_OK){
		existSD = false;
		return;
	}
	
	res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
	if( res != FR_OK){
		printDebug("F_Open file on SD - ERROR!\n");
		existSD = false;
		return;
	}else{
		printDebug("F_Open file on SD - OK\n");
	}
	
	res = f_lseek(&testFile,  f_size(&testFile));
	if( res != FR_OK){
		printDebug("F_Lseek of file - ERROR!\n");
		existSD = false;
		return;
	}else{
		printDebug("F_Lseek file on SD - OK\n");
	}
}


