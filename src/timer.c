// timer.c
//
// Provide a µs resolution application clock for timing/PID/IMU.
//
// This file is part of the CX10_fnrf project, released under the 
// GNU General Public License, see LICENSE.md for further details.
//
// Copyright ©  2015 Samuel Powell
//				2014 Felix Niessen

// Overview
//
// TIM3 is configured to increment at 1us by setting a 48Mhz
// system clock prescaler of (47+1). An interrupt is generated when
// the timer overflows, this will occurr every 0xFFFF us. A global
// variable is incremented each time this occurrs.

#include "config.h"

// Count how many overflows (0xFFFF)us since initialisation
static uint16_t T3OV = 0;

// Return system time in microseconds, and milliseconds
uint32_t micros() {return (T3OV<<16)+(TIM3->CNT);}
uint32_t millis() {return (micros()/1000);}

// Pause execution for a specified number of microseconds
void delay_micros(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us);
}

// Overflow indicates 0xFFFFus have passed, increment counter
void TIM3_IRQHandler(void)
{
    if(TIM3->SR & TIM_IT_Update)
    {
        TIM3->SR = (uint16_t)~TIM_IT_Update;
        T3OV++;
    }
}

// Initialise TIM3 and configure NVIC
void init_timer()
{
    
    TIM_TimeBaseInitTypeDef timerbaseinit;
    NVIC_InitTypeDef NVIC_InitStructure;
	
    // Prescaler of 47 -> 48Mhz/(47+1) = 1us update
	TIM_DeInit(TIM3);
	timerbaseinit.TIM_Prescaler = 47;
	timerbaseinit.TIM_Period = 0xFFFF;
	timerbaseinit.TIM_ClockDivision = TIM_CKD_DIV1;
	timerbaseinit.TIM_RepetitionCounter = 0;
	timerbaseinit.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &timerbaseinit);
	TIM_Cmd(TIM3, ENABLE); 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    
}










