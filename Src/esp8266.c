#include "esp8266.h"
#include "stdbool.h"

/* Ring buffer */
uint8_t ringBuffer[512];
int head = 0;
int tail = 0;
int ringCount = 0;

bool readyAns = false;

uint8_t ansESP;

 
void ESP8266_Init(){
	HAL_UART_Receive_IT(&huart3, &ansESP,1);
	flushRxBuff();
	
	if(verbose){
		HAL_Delay(6000);		
		USB_Print("Start init ESP8266 .... \n");
		HAL_Delay(1000);		
	}
		
	
	ESP8266_SendCMD(AT);
	ESP8266_ReceiveAns();
	
	HAL_Delay(500);
	
	USART_SendCMD("AT+CIOBAUD=115200\r\n");
	ESP8266_ReceiveAns();
	HAL_Delay(2000);
	
	ESP8266_SendCMD(AT_GMR);
	ESP8266_ReceiveAns();
	
	HAL_Delay(1000);
	
	ESP8266_SendCMD(AT_CWLAP);
	ESP8266_ReceiveAns();
	HAL_Delay(1000);

	USART_SendCMD("AT+CWJAP=\"mipt\",\"\"\r\n");
	ESP8266_ReceiveAns();
	HAL_Delay(180000);
	
	USART_SendCMD("AT+CWJAP?\r\n");
	ESP8266_ReceiveAns();
	HAL_Delay(1000);
	
	USART_SendCMD("AT+RST\r\n");
	ESP8266_ReceiveAns();
	HAL_Delay(1000);
	
/*
	USART_SendCMD("AT+CWMODE=1\r\n");
	ESP8266_ReceiveAns();
	HAL_Delay(1000);
	
	USART_SendCMD("AT+CWMODE?\r\n");
	ESP8266_ReceiveAns();
	HAL_Delay(1000);

	USART_SendCMD("AT+CIPMUX=1\r\n");
	ESP8266_ReceiveAns();
	HAL_Delay(1000);
	
	USART_SendCMD("AT+CIPSERVER=1,80\r\n");
	ESP8266_ReceiveAns();
	HAL_Delay(1000);
	
	USART_SendCMD("AT+CIFSR\r\n");
	ESP8266_ReceiveAns();
	HAL_Delay(1000);
	*/
}

void ESP8266_SendCMD(enum AT_CMD cmd){
	switch(cmd){
		case AT:
			USART_SendCMD("AT\r\n");
			break;
		case AT_GMR:
			USART_SendCMD("AT+GMR\r\n");
			break;
		case AT_RST:
			USART_SendCMD("AT+RST\r\n");
			break;
		case AT_CWLAP:
			USART_SendCMD("AT+CWLAP\r\n");
			HAL_Delay(5000);
			break;
		default:
			USB_Print("Comand not found!\n");
			break;
	}
}

void USART_SendCMD(uint8_t* cmd){
	uint8_t* ptr = cmd;
	int size = 0;
	
	while(*cmd++)
		size++;
	
	while( HAL_UART_Transmit(&huart3, ptr, size, 0x1000) != HAL_OK);
	
	if(verbose){
		USB_Print("Cmd: ");
		USB_Print(ptr);
	}
}

void ESP8266_ReceiveAns(){
	
	HAL_UART_Receive_IT(&huart3, &ansESP,1);
	while(!readyAns);
	readyAns = false;
	
	printRingBuffer();
	
}

void flushRxBuff(){
	tail = head;
	ringCount = 0;
}


void addRingBuffer(uint8_t d){
	ringBuffer[head] = d;
	head++;
	ringCount++;
	
	if(head == tail){
		head++;
		tail++;
		ringCount--;
		
		if(head == 512)
			head = 0;
		
		if(tail == 512)
			tail = 0;
	}
	
	if(head == 512)
		head = 0;
	
	if(ringCount > 1)
		readyAns = true;
	
}

void printRingBuffer(){
	if(verbose){
		if(ringCount == 0){
			USB_Print("\nBuffer is empty!\n");
		}
		else{
			USB_Print("\nESP8266 answer: \n");
		}
	}
	
	while(ringCount > 0){
		//while( CDC_Transmit_FS(&ringBuffer[tail],1) == USBD_BUSY);
		USB_SendByte(&ringBuffer[tail]);
		tail++;
		ringCount--;
		
		if(tail == 512)
			tail = 0;
	}
	
	if(verbose){
		USB_Print("\n==============\n");
	}
}

