/*
	This file is part of STM32F05x brushed Copter FW
	Copyright © 2014 Felix Niessen ( felix.niessen@googlemail.com )

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "config.h"


//PA14(SWCLK) as PPM input
int16_t RXcommands[6] = {0,500,500,500,500,500};// Throttle,Roll,Pitch,Yaw,Aux1,Aux2
uint8_t failsave;
static uint8_t chanOrder[6] = {RC_CHAN_ORDER};
static uint16_t RawChannels[6] = {1500,1500,1500,1500,1500,1500};
static uint8_t chanNewValue[6] = {1,1,1,1,1,1};

void init_PPMRX(){
	GPIO_InitTypeDef RXGPIOinit;
	RXGPIOinit.GPIO_Pin = GPIO_Pin_14;
	RXGPIOinit.GPIO_Mode = GPIO_Mode_IN;
	RXGPIOinit.GPIO_Speed = GPIO_Speed_50MHz;
	RXGPIOinit.GPIO_OType = GPIO_OType_OD; 
	RXGPIOinit.GPIO_PuPd   = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &RXGPIOinit);		
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource14);
	
	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line = EXTI_Line14;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void EXTI4_15_IRQHandler(void){
	static uint16_t lastTime = 0;
	static uint8_t actChannel = 0;
	uint16_t ThisTime = TIM3->CNT;
	if((EXTI->PR & EXTI_Line14) != (uint32_t)RESET){
		EXTI->PR = EXTI_Line14;
		if((GPIOA->IDR & GPIO_Pin_14) != (uint32_t)Bit_RESET){// spike filter
			uint16_t PPMVal = ThisTime-lastTime;
			if(PPMVal > 5000) actChannel = 0;
			else if(PPMVal > 500 && PPMVal <2500){
				failsave = 0;
				if(actChannel < 6){
					chanNewValue[actChannel] = 1;
					RawChannels[actChannel++] = PPMVal;
				}
			}
			lastTime = ThisTime;
		}
	}	
}


void getRXDatas(){
	static uint16_t channelBuffer[6] = {1500,1500,1500,1500,1500,1500};
	uint8_t i;
	for(i=0;i<6;i++){
		if(chanNewValue[chanOrder[i]] && ((channelBuffer[i]-RawChannels[chanOrder[i]] > 1) || (channelBuffer[i]-RawChannels[chanOrder[i]] < 1))){
			if(i == 0) RXcommands[i] = constrain(RawChannels[chanOrder[i]]-1000,0,1000); // throttle
			else RXcommands[i] = constrain(RawChannels[chanOrder[i]]-1500,-500,500);
			channelBuffer[i] = RawChannels[chanOrder[i]];
			chanNewValue[chanOrder[i]] = 0;
		}
	}
}




