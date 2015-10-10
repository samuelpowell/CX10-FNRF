// adc.c
// 
// This file is part of the CX10_fnrf project, released under the 
// GNU General Public License, see LICENSE.md for further details.
//
// Copyright © 	2015 Samuel Powell
//							2014 Felix Niessen

#include "config.h"

extern int16_t LiPoVolt;

void init_ADC(){
	
	GPIO_InitTypeDef gpioinitADC;
	#if defined(CX_10_RED_BOARD)
	gpioinitADC.GPIO_Pin = GPIO_Pin_2;
	#endif 
	#if defined(CX_10_BLUE_BOARD)
	gpioinitADC.GPIO_Pin = GPIO_Pin_7; //-5
	#endif
	gpioinitADC.GPIO_Mode = GPIO_Mode_AN;
	gpioinitADC.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &gpioinitADC);
	
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
	ADC_Init (ADC1, &ADC_InitStructure);
	
	#if defined(CX_10_RED_BOARD)
	ADC_ChannelConfig (ADC1, ADC_Channel_2, ADC_SampleTime_239_5Cycles);
	#endif 
	#if defined(CX_10_BLUE_BOARD)
	ADC_ChannelConfig (ADC1, ADC_Channel_7, ADC_SampleTime_239_5Cycles);
	#endif
	
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_COMP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	ADC_GetCalibrationFactor(ADC1);
	ADC_Cmd (ADC1, ENABLE);
	while (!ADC_GetFlagStatus (ADC1, ADC_FLAG_ADEN));
	
}



void ADC1_COMP_IRQHandler(void){
	if((uint32_t)(ADC1->ISR & ADC_IT_EOC) != (uint32_t)RESET){
		ADC1->ISR = (uint32_t)ADC_IT_EOC; 
		#if defined(CX_10_RED_BOARD)
		LiPoVolt      = ((ADC1->DR)<<7)/954;
		#endif
		#if defined(CX_10_BLUE_BOARD)
		LiPoVolt      = ((ADC1->DR)<<7)/153;
		#endif
	}
}





