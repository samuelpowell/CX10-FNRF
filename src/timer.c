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



void init_Timer(){
	
	TIM_DeInit(TIM3);
	
	// 16bit tim 3 for time measurement
	TIM_TimeBaseInitTypeDef timerbaseinit;
	timerbaseinit.TIM_Prescaler = 47; // ticks with 1µs
	timerbaseinit.TIM_Period = 0xFFFF;
	timerbaseinit.TIM_ClockDivision = TIM_CKD_DIV1;
	timerbaseinit.TIM_RepetitionCounter = 0;
	timerbaseinit.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &timerbaseinit);
	TIM_Cmd(TIM3, ENABLE); 
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	
	#if defined(CX_10_RED_BOARD)
	GPIO_InitTypeDef gpioinitTIM;
	gpioinitTIM.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_8 | GPIO_Pin_11;
	gpioinitTIM.GPIO_Mode = GPIO_Mode_AF;
	gpioinitTIM.GPIO_Speed = GPIO_Speed_50MHz;
	gpioinitTIM.GPIO_OType = GPIO_OType_PP; 
	gpioinitTIM.GPIO_PuPd   = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpioinitTIM);
	
	gpioinitTIM.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOB, &gpioinitTIM);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_2);
	
	timerbaseinit.TIM_Prescaler = 1; // 24khz
	timerbaseinit.TIM_Period = 1000;
	timerbaseinit.TIM_ClockDivision = TIM_CKD_DIV1;
	timerbaseinit.TIM_RepetitionCounter = 0;
	timerbaseinit.TIM_CounterMode = TIM_CounterMode_Up;
	
	
	TIM_DeInit(TIM1);
	TIM_TimeBaseInit(TIM1, &timerbaseinit);
	
	TIM_DeInit(TIM2);
	TIM_TimeBaseInit(TIM2, &timerbaseinit);
	
	TIM_DeInit(TIM16);
	TIM_TimeBaseInit(TIM16, &timerbaseinit);
	
	TIM_OCInitTypeDef channelbaseconf;
	channelbaseconf.TIM_OCMode = TIM_OCMode_PWM2;
	channelbaseconf.TIM_OutputState = TIM_OutputState_Enable; 
	channelbaseconf.TIM_Pulse = 0; 
	channelbaseconf.TIM_OCPolarity = TIM_OCPolarity_Low;
	channelbaseconf.TIM_OCIdleState = TIM_OCIdleState_Reset;
	
	TIM_OC1Init(TIM1, &channelbaseconf); 
	TIM_OC4Init(TIM1, &channelbaseconf); 
	TIM_OC1Init(TIM16, &channelbaseconf);
	TIM_OC4Init(TIM2, &channelbaseconf); 
	
	TIM_Cmd(TIM1, ENABLE); 
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
	TIM_CtrlPWMOutputs(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE); 
	TIM_CtrlPWMOutputs(TIM16, ENABLE);
	TIM_Cmd(TIM16, ENABLE); 
	#endif 
	
	#if defined(CX_10_BLUE_BOARD)
	GPIO_InitTypeDef gpioinitTIM;
	#if defined(FORCE_SERIAL)
	gpioinitTIM.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
	#else
	gpioinitTIM.GPIO_Pin = GPIO_Pin_8 |GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	#endif
	gpioinitTIM.GPIO_Mode = GPIO_Mode_AF;
	gpioinitTIM.GPIO_Speed = GPIO_Speed_50MHz;
	gpioinitTIM.GPIO_OType = GPIO_OType_PP; 
	gpioinitTIM.GPIO_PuPd   = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpioinitTIM);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
	#if !defined(FORCE_SERIAL)
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_2);
	#endif
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_2);
	timerbaseinit.TIM_Prescaler = 1; // 24khz
	timerbaseinit.TIM_Period = 1000;
	timerbaseinit.TIM_ClockDivision = TIM_CKD_DIV1;
	timerbaseinit.TIM_RepetitionCounter = 0;
	timerbaseinit.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_DeInit(TIM1);
	TIM_TimeBaseInit(TIM1, &timerbaseinit);
	
	TIM_OCInitTypeDef channelbaseconf;
	channelbaseconf.TIM_OCMode = TIM_OCMode_PWM2;
	channelbaseconf.TIM_OutputState = TIM_OutputState_Enable; 
	channelbaseconf.TIM_Pulse = 0; 
	channelbaseconf.TIM_OCPolarity = TIM_OCPolarity_Low;
	channelbaseconf.TIM_OCIdleState = TIM_OCIdleState_Reset;
	
	TIM_OC1Init(TIM1, &channelbaseconf); 
	TIM_OC2Init(TIM1, &channelbaseconf); 
	TIM_OC3Init(TIM1, &channelbaseconf);
	TIM_OC4Init(TIM1, &channelbaseconf); 
	
	TIM_Cmd(TIM1, ENABLE); 
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	#endif
}












