// blinker.c
// 
// This file is part of the CX10_fnrf project, released under the 
// GNU General Public License, see LICENSE.md for further details.
//
// Copyright © 	2015 Samuel Powell

// Overview
//
// Configure a timer to provide a low-frequency interrupt, used to
// drive selectable illumination patterns on the LEDs.

#include "config.h"

// Configure interrupt handler and globals
static uint8_t blink_style = BLINKER_OFF;

void TIM17_IRQHandler(void) {
    
    static uint8_t phase = 0;
	
    if(TIM17->SR & TIM_IT_Update) {
		TIM17->SR = (uint16_t)~TIM_IT_Update;
        
        switch(blink_style)
        {
            // All on
            case BLINKER_ON:
                GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDon);
                GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDon);
                break;
            
            // Flash front
            case BLINKER_BIND:
                if(phase) {
                    GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDon);
                    GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDoff);
                }
                else {
                    GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDoff);
                    GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDoff);
                }
                break;
            
            // Front on
            case BLINKER_DISARM:
                GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDon);
                GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDoff);
                break;

            // Flash front and back alternately
            case BLINKER_LOWBAT:
                if(phase) {
                    GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDon);
                    GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDoff);
                }
                else {
                    GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDoff);
                    GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDon);
                }        
                break;
            
            case BLINKER_OFF:
            default:
                GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDoff);
                GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDoff);
                break;    

        }
        // Toggle phase
        phase = !phase;
	}
}

// Set the current blinker style and force an interrupt
void set_blink_style(uint8_t style)
{
    blink_style = style;
    TIM17->SR |= (uint16_t)~TIM_IT_Update;
    TIM17_IRQHandler();
}
    

// Initialise timers and inteerupt
void init_blinker(void) {
    
    TIM_TimeBaseInitTypeDef timerbaseinit;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // Configure timer 17 for 1ms tick
	TIM_DeInit(TIM17);
	timerbaseinit.TIM_Prescaler = 47999;                
	timerbaseinit.TIM_Period = 0x00FF; 
	timerbaseinit.TIM_ClockDivision = TIM_CKD_DIV1;
	timerbaseinit.TIM_RepetitionCounter = 0;
	timerbaseinit.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM17, &timerbaseinit);
	TIM_Cmd(TIM17, ENABLE); 
	
	// Enable interrupts on timer overflow
	NVIC_InitStructure.NVIC_IRQChannel = TIM17_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM17, TIM_IT_Update, ENABLE);
    
    // Configure GPIO
    GPIO_InitTypeDef LEDGPIOinit;
    LEDGPIOinit.GPIO_Pin = LED1_BIT;
    LEDGPIOinit.GPIO_Mode = GPIO_Mode_OUT;
    LEDGPIOinit.GPIO_Speed = GPIO_Speed_50MHz;
    LEDGPIOinit.GPIO_OType = GPIO_OType_PP;
    LEDGPIOinit.GPIO_PuPd   = GPIO_PuPd_NOPULL;
    GPIO_Init(LED1_PORT, &LEDGPIOinit);
    
    LEDGPIOinit.GPIO_Pin = LED2_BIT;
    GPIO_Init(LED2_PORT, &LEDGPIOinit);
    
}




