// serial.c
// 
// This file is part of the CX10_fnrf project, released under the 
// GNU General Public License, see LICENSE.md for further details.
//
// Copyright © 	2015 Samuel Powell
//							2014 Felix Niessen

#include "config.h"

#if defined(SERIAL_ACTIVE)
static uint8_t rxBuffer[256];
static uint8_t rxBufTail = 0;
static uint8_t rxBufHead = 0;
static uint8_t txBufTail = 0;
static uint8_t txBufHead = 0;
static uint8_t txOn = 0;


static uint8_t txBuf[256];


void init_UART(uint32_t speed){
	// USART1_TX    PA9
	// USART1_RX    PA10
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
	
	GPIO_InitTypeDef gpioinitUART;
	gpioinitUART.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	gpioinitUART.GPIO_Mode = GPIO_Mode_AF;
	gpioinitUART.GPIO_Speed = GPIO_Speed_50MHz;
	gpioinitUART.GPIO_OType = GPIO_OType_PP; 
	gpioinitUART.GPIO_PuPd   = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpioinitUART);

	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = speed;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);


	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_TC, ENABLE);

	USART_Cmd(USART1, ENABLE);
	
}


void USART1_IRQHandler(void){
	if (USART_GetITStatus( USART1, USART_IT_RXNE ) != RESET){
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		rxBuffer[rxBufHead++] = USART_ReceiveData(USART1);
	}  
	if (USART_GetITStatus(USART1, USART_IT_TC) != RESET){
		USART_ClearITPendingBit(USART1, USART_IT_TC);
		if(txBufTail != txBufHead && txOn)
			USART_SendData(USART1, txBuf[txBufTail++]);
		else{
			txOn = 0;
			USART_ITConfig(USART1, USART_IT_TC, DISABLE);
			StartUSART();
		}
	}
}



void StartUSART(void){
	if(!txOn && txBufHead != txBufTail){
		txOn = 1;
		USART_ITConfig(USART1, USART_IT_TC, ENABLE);
		USART_SendData(USART1, txBuf[txBufTail++]);
	}
}




uint8_t serial_available(void){
	return rxBufHead-rxBufTail;
}

uint8_t serial_read(void){
	if(rxBufTail != rxBufHead)
		return rxBuffer[(uint8_t)rxBufTail++];
	else return 0;
}

void serial_send_bytes(uint8_t *s, int n){
	uint8_t  i = 0;
	while(i < n) txBuf[txBufHead++] = s[i++];
	StartUSART();
}


void print_int16(int16_t p_int){
	uint16_t useVal = p_int;
	uint8_t pre = ' ';
	if(p_int < 0){
		useVal = p_int*-1;
		pre = '-';
	}
	uint8_t aciidig[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9' };
	uint8_t i = 0;
        uint8_t digits[6] = {0,0,0,0,0,0};
	while(useVal >= 10000){digits[0]++; useVal-=10000;}
	while(useVal >= 1000){digits[1]++; useVal-=1000;}
	while(useVal >= 100){digits[2]++; useVal-=100;}
	while(useVal >= 10){digits[3]++; useVal-=10;}
	digits[4] = useVal;
	uint8_t result[6] = {' ',' ',' ',' ',' ','0'};
	uint8_t signdone = 0;
	for(i = 0; i < 6;i++){
		if(i == 5 && signdone == 0) continue;
		else if(aciidig[digits[i]] != '0' && signdone == 0){
			result[i] = pre;
			signdone = 1;
		}else if(signdone) result[i] = aciidig[digits[i-1]];
	}
	serial_send_bytes(result, 6);
}

#endif









