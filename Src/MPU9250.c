#include "MPU9250.h"

int16_t Ax;
int16_t Ay;
int16_t Az;

uint8_t acc[6];

void MPU_Init(){
	
	if(verbose){
		USB_Print("\n**************************************\n");
		USB_Print("Start MPU9250\n");
	}
	
	MPU_RREG(WHOAMI);
	
	MPU_WREG(ACCEL_CONFIG,ACC_FULL_SCALE_2_G);
	MPU_RREG(ACCEL_CONFIG);
	
	MPU_WREG(GYRO_CONFIG,GYRO_FULL_SCALE_1000_DPS);
	MPU_RREG(GYRO_CONFIG);
	
	if(verbose){
		USB_Print("\n MPU9250 configure ..... DONE!\n");
		USB_Print("\n**************************************\n");
		HAL_Delay(3000);
	}
	
}

void MPU_SendData(){
	if(!verbose){
		USB_Print("DTX");
		USB_Send2Byte((uint8_t*)&Ax);
		USB_Print("DTY");
		USB_Send2Byte((uint8_t*)&Ay);
		USB_Print("DTZ");
		USB_Send2Byte((uint8_t*)&Az);
	}
	else 
		HAL_Delay(500);
}

uint8_t MPU_RREG(uint8_t _address){
	uint8_t opcode = 0x80 | _address;
	uint8_t rx = 0x00;
	
	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET);
	
	transferMPU(opcode);
	rx = transferMPU(0x00); 
	
	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);
	
	if(verbose){
		USB_Print("\nRegister: ");
		MPU_printRegisterName(_address);
		USB_Print(" has value: ");
		USB_SendBits(rx);
		USB_Print("\n");
	}
	
	return rx;
}


void MPU_WREG(uint8_t _address, uint8_t _value) {		//  Write ONE register at _address
	uint8_t opcode1 = _address; 								//  WREG expects 010rrrrr where rrrrr = _address
	
	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET); 					//  open SPI
	
	transferMPU( opcode1);												//  Send WREG command & address
	transferMPU( _value);													//  Write the value to the register
	
	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);						//  close SPI
	
	if(verbose){												//  verbose output
		USB_Print("\nRegister: ");
		MPU_printRegisterName(_address);
		USB_Print(" - modified\n");
	}
}

// Read acceleration registers Ax Ay Az
void MPU_ReadAcc(){
	int i = 0;
	
	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET); 					//  open SPI
	
	transferMPU(0x80 | ACCEL_XOUT_H);
	for(i = 0; i < 6; i++){
			acc[i] = transferMPU(0x00);
	}
	
	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);						//  close SPI

	Ax =  acc[0] << 8 | (uint8_t)acc[1];
	Ay =  acc[2] << 8 | (uint8_t)acc[3];
	Az =  acc[4] << 8 | (uint8_t)acc[5];
	
	if(verbose){
		USB_Print("\n------------");
		USB_Print("\nAx: ");
		USB_SendNumber(Ax);
		USB_Print("\nAy: ");
		USB_SendNumber(Ay);
		USB_Print("\nAz: ");
		USB_SendNumber(Az);
		USB_Print("\n------------\n");
	}
}

void MPU_printRegisterName(uint8_t _address){
    if(_address == WHOAMI){
        USB_Print("ID "); 
    }
		else if(_address == CONFIG){
			USB_Print("CONFIG ");
		}
		else if(_address == GYRO_CONFIG){
			USB_Print("GYRO_CONFIG ");
		}
		else if(_address == ACCEL_CONFIG){
			USB_Print("ACCEL_CONFIG ");
		}
}


