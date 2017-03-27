#include "BME280.h"

// ------ Private variables -----------

uint32_t	dig_T1 = 0x00000000;
int32_t 	dig_T2 = 0x00000000;
int32_t 	dig_T3 = 0x00000000;

uint16_t 	dig_P1 = 0x0000;
int16_t 	dig_P2 = 0x0000;
int16_t 	dig_P3 = 0x0000;
int16_t 	dig_P4 = 0x0000;
int16_t 	dig_P5 = 0x0000;
int16_t 	dig_P6 = 0x0000;
int16_t 	dig_P7 = 0x0000;
int16_t 	dig_P8 = 0x0000;
int16_t 	dig_P9 = 0x0000;

uint8_t 	dig_H1 = 0x00;
int16_t 	dig_H2 = 0x0000;
uint8_t 	dig_H3 = 0x00;
int16_t 	dig_H4 = 0x0000;
int16_t 	dig_H5 = 0x0000;
uint8_t 	dig_H6 = 0x00;

uint8_t temp[3];
uint8_t pres[3];
uint8_t humid[2];

int32_t t_fine;

int32_t Temperature;
int32_t Pressure;
int32_t Humidity;

// -------- Functions -----------------

void BME_SendData(){
	if(!verbose){
		USB_Print("DTT");
		USB_Send4Byte((uint8_t*)&Temperature);
		USB_Print("DTP");
		USB_Send4Byte((uint8_t*)&Pressure);
		USB_Print("DTH");
		USB_Send4Byte((uint8_t*)&Humidity);
	}else
			HAL_Delay(500);
}

void BME_readData(){
	if(verbose){
		USB_Print("\n-----------------\n");
		USB_Print("Read data from BME280\n");
	}
	
	BME_ReadPreassure();
	BME_ReadTemp();
	BME_ReadHumid();
	
	if(verbose){
		USB_Print("\n-----------------\n");
	}
}

void BME_Init(){
	BME_RREG(BME280_REGISTER_CHIPID);
	
	BME_WREG(BME280_REGISTER_CONTROLHUMID, 0x01);
	//HAL_I2C_Mem_Write(&hi2c1,BME280_ADDR<<1,BME280_REGISTER_CONTROLHUMID,1,(uint8_t*)2,1,0x1000);
	HAL_Delay(100);
	//HAL_I2C_Mem_Write(&hi2c1,BME280_ADDR<<1,BME280_REGISTER_CONTROL,1,(uint8_t*)0x4B,1,0x1000);
	BME_WREG(BME280_REGISTER_CONTROL, 0x6B);
	
	HAL_Delay(100);
	
	BME_WREG(BME280_REGISTER_CONFIG, 0xA8);
	
	if(verbose)
		USB_Print("\nValidation...\n");
	
	BME_RREG(BME280_REGISTER_CONTROLHUMID);
	BME_RREG(BME280_REGISTER_CONTROL);
	BME_RREG(BME280_REGISTER_CONFIG);
	
	BME_getDIG_T();
	BME_getDIG_P();
	BME_getDIG_H();
}

void BME_WREG(uint8_t addr, uint8_t data){
	HAL_I2C_Mem_Write(&hi2c1, BME280_ADDR<<1 ,addr ,1 ,&data ,1 ,0x1000);
	
	if(verbose){
		USB_Print("\nRegister ");
		BME_printRegisterName(addr);
		USB_Print(" modified on ");
		USB_SendBits(data);
		USB_Print("\n");
	}
}

void BME_ReadPreassure(){
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR<<1, BME280_REGISTER_PRESSUREDATA    , 1, &pres[0], 1, 0x1000);
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR<<1, BME280_REGISTER_PRESSUREDATA + 1, 1, &pres[1], 1, 0x1000);
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR<<1, BME280_REGISTER_PRESSUREDATA + 2, 1, &pres[2], 1, 0x1000);
		
	Pressure = BME280_compensate_P_int32((int32_t)((pres[0] << 12) | (pres[1] << 4) | pres[2] >> 4 ));
	
}

// Read temperature ADC
void BME_ReadTemp(){
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR<<1, BME280_REGISTER_TEMPDATA    , 1, &temp[0], 1, 0x1000);
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR<<1, BME280_REGISTER_TEMPDATA + 1, 1, &temp[1], 1, 0x1000);
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR<<1, BME280_REGISTER_TEMPDATA + 2, 1, &temp[2], 1, 0x1000);
		
	Temperature = BME280_compensate_T_int32( (int32_t)((temp[0] << 12) | (temp[1] << 4) | temp[2] >> 4));
	
}

void BME_ReadHumid(){
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR<<1, BME280_REGISTER_HUMIDDATA     , 1, &humid[0], 1, 0x1000);
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR<<1, BME280_REGISTER_HUMIDDATA +1  , 1, &humid[1], 1, 0x1000);
	
	Humidity = BME280_CalcH( (int32_t) ((humid[0] << 8) | humid[1]) );
}

uint8_t BME_RREG(uint8_t addr){
	uint8_t rx = 0x00;
	
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR<<1,addr,1,&rx,1,0x1000);
	
	if(verbose){
		USB_Print("\nRead register ");
		BME_printRegisterName(addr);
		USB_Print(" ");
		USB_SendBits(rx);
		USB_Print("\n");
	}
	
	return rx;
}

void BME_getDIG_H(void){
	uint8_t b1, b2;
	
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR << 1, BME280_REGISTER_DIG_H1, 1, &dig_H1, 1, 0x1000);
	
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR << 1, BME280_REGISTER_DIG_H2, 1, &b1, 1, 0x1000);
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR << 1, BME280_REGISTER_DIG_H2 + 1, 1, &b2, 1, 0x1000);
	
	dig_H2 = (int16_t)((b2 << 8 ) | b1);
	
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR << 1, BME280_REGISTER_DIG_H3, 1, &dig_H3, 1, 0x1000);
	
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR << 1, BME280_REGISTER_DIG_H4, 1, &b1, 1, 0x1000);
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR << 1, BME280_REGISTER_DIG_H5, 1, &b2, 1, 0x1000);
	
	dig_H4 = (int16_t) ((b1 << 4) | (b2 & 0x0F) );
	
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR << 1, BME280_REGISTER_DIG_H5, 1, &b1, 1, 0x1000);
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR << 1, BME280_REGISTER_DIG_H5+1, 1, &b2, 1, 0x1000);
	
	dig_H5 = (int16_t)( (b2 << 4) | (b1 >> 4) );
	
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR << 1, BME280_REGISTER_DIG_H6, 1, &dig_H6, 1, 0x1000);
	
}

void BME_getDIG_P(void){
	int i = 0;
	uint8_t dig[18];
	
	for ( i = 0; i < 18; i++ ){
			dig[i] = BME_RREG(BME280_REGISTER_DIG_P1 + i);
			HAL_Delay(200);
	}
	
	dig_P1 = ((dig[1] << 8) | dig[0]);
	dig_P2 = ((dig[3] << 8) | dig[2]);
	dig_P3 = ((dig[5] << 8) | dig[4]);
	dig_P4 = ((dig[7] << 8) | dig[6]);
	dig_P5 = ((dig[9] << 8) | dig[8]);
	dig_P6 = ((dig[11] << 8) | dig[10]);
	dig_P7 = ((dig[13] << 8) | dig[12]);
	dig_P8 = ((dig[15] << 8) | dig[14]);
	dig_P9 = ((dig[17] << 8) | dig[16]);
	
}

void BME_getDIG_T(void){
	int i = 0;
	uint8_t dig[6];
	
	for ( i = 0; i < 6; i++ ){
		dig[i] = BME_RREG(BME280_REGISTER_DIG_T1 + i);
		HAL_Delay(200);
	}

	dig_T1 = ((dig[1] << 8) | dig[0]);
	dig_T2 = ((dig[3] << 8) | dig[2]);
	dig_T3 = ((dig[4] << 8) | dig[4]);

	}

void BME_printRegisterName(uint8_t _address) {
	if(_address == BME280_REGISTER_DIG_T1){
		USB_Print("BME280_REGISTER_DIG_T1 ");
	}
	else if(_address == BME280_REGISTER_DIG_T2){
		USB_Print("BME280_REGISTER_DIG_T2 ");
	}
	else if(_address == BME280_REGISTER_DIG_T3){
		USB_Print("BME280_REGISTER_DIG_T3 ");
	}
	else if(_address == BME280_REGISTER_CONTROLHUMID){
		USB_Print("BME280_REGISTER_CONTROLHUMID ");
	}
	else if(_address == BME280_REGISTER_CONTROL){
		USB_Print("BME280_REGISTER_CONTROL ");
	}
	else if(_address == BME280_REGISTER_CONFIG){
		USB_Print("BME280_REGISTER_CONFIG ");
	}
	else if(_address == BME280_REGISTER_PRESSUREDATA){
		USB_Print("BME280_REGISTER_PRESSUREDATA ");
	}
	else if(_address == BME280_REGISTER_TEMPDATA){
		USB_Print("BME280_REGISTER_TEMPDATA ");
	}
	else if(_address == BME280_REGISTER_DIG_P1){
		USB_Print("BME280_REGISTER_DIG_P1 ");
	}
	else if(_address == BME280_REGISTER_DIG_P2){
		USB_Print("BME280_REGISTER_DIG_P2 ");
	}
	else if(_address == BME280_REGISTER_DIG_P3){
		USB_Print("BME280_REGISTER_DIG_P3 ");
	}
	else if(_address == BME280_REGISTER_DIG_P4){
		USB_Print("BME280_REGISTER_DIG_P4 ");
	}
	else if(_address == BME280_REGISTER_DIG_P5){
		USB_Print("BME280_REGISTER_DIG_P5 ");
	}
	else if(_address == BME280_REGISTER_DIG_P6){
		USB_Print("BME280_REGISTER_DIG_P6 ");
	}
	else if(_address == BME280_REGISTER_DIG_P7){
		USB_Print("BME280_REGISTER_DIG_P7 ");
	}
	else if(_address == BME280_REGISTER_DIG_P8){
		USB_Print("BME280_REGISTER_DIG_P8 ");
	}
	else if(_address == BME280_REGISTER_DIG_P9){
		USB_Print("BME280_REGISTER_DIG_P9 ");
	}
}


int32_t BME280_compensate_T_int32(int32_t adc_T){
	int32_t var1 = 0, var2 = 0, T = 0;
	
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1)))*((int32_t)dig_T2))>>11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	
	t_fine = var1 + var2;
	
	T = (t_fine * 5 + 128) >> 8;
	
	if(verbose){
		USB_Print("Temp: "); 
		USB_SendNumber(T/100);
		USB_Print("."); 
		USB_SendNumber(T%100);
		USB_Print(" deg C.\n"); 
		
		
	}
	
	
	return T;
}





// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
uint32_t BME280_compensate_P_int32(int32_t adc_P){
	int32_t  var1, var2;
	uint32_t  p;
	
	var1 = ( t_fine >> 1) - 64000;
	var2 = (((var1 >> 2) * ( var1 >> 2)) >> 11 ) * dig_P6;
	var2 = var2 + ( (var1*dig_P5) << 1 );
	var2 = (var2 >> 2) + ( dig_P4 << 16 );
	var1 = (((dig_P3 * ((( var1 >> 2) * (var1>>2)) >> 13 )) >> 3) + ((dig_P2 * var1) >> 1) ) >> 18;
	var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
	
	if (var1 == 0){
		USB_Print("\nERROR! - Division by ZERO!\n");
		return 0; // avoid exception caused by division by zero
	}
	
	p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
	
	if (p < 0x80000000){
		p = (p << 1) / ((uint32_t)var1);
	}
	else{
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
	
	p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
	
	if(verbose){
		USB_Print("Pressure: ");
		USB_SendNumber(p);
		USB_Print(" Pa\n");
	}
	
	return p;
}

uint32_t BME280_CalcH(int32_t UH) {
	int32_t vx1;

	vx1  = t_fine - (int32_t)76800;
	vx1  = ((((UH << 14) - ((int32_t)dig_H4 << 20) - ((int32_t)dig_H5 * vx1)) +	(int32_t)16384) >> 15) *
			(((((((vx1 * (int32_t)dig_H6) >> 10) * (((vx1 * (int32_t)dig_H3) >> 11) +
			(int32_t)32768)) >> 10) + (int32_t)2097152) * ((int32_t)dig_H2) + 8192) >> 14);
	vx1 -= ((((vx1 >> 15) * (vx1 >> 15)) >> 7) * (int32_t)dig_H1) >> 4;
	vx1  = (vx1 < 0) ? 0 : vx1;
	vx1  = (vx1 > 419430400) ? 419430400 : vx1;

	if(verbose){
		USB_Print("Humidity: ");
		USB_SendNumber( (vx1 >> 12)/1024 );
		USB_Print(".");
		USB_SendNumber( (vx1 >> 12)%1024 );
		USB_Print("%\n");
	}
	
	return (uint32_t)(vx1 >> 12);
}

