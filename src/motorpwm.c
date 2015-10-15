// motorpwm.c
//
// Provide PWM driven motor outputs.
//
// This file is part of the CX10_fnrf project, released under the
// GNU General Public License, see LICENSE.md for further details.
//
// Copyright �  2015 Samuel Powell
//				2014 Felix Niessen

// Overview
//
// Timers and GPIO are configures according to target PCB layout to
// provide 24kHz PWM with 1000 steps. Current duty cycle is set by
// writing to CCR registers.

#include "config.h"

#define MOTORPWM_DUTY_MIN 50
#define MOTORPWM_DUTY_MAX 1000

// Initialise timers, GPIO, and output capture for PWM
void init_motorpwm()
{
    GPIO_InitTypeDef gpioinitTIM;
    TIM_OCInitTypeDef channelbaseconf;
     TIM_TimeBaseInitTypeDef timerbaseinit;
    
    #if defined(CX10_REDV1)
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
    timerbaseinit.TIM_Period = MOTORPWM_DUTY_MAX;
    timerbaseinit.TIM_ClockDivision = TIM_CKD_DIV1;
    timerbaseinit.TIM_RepetitionCounter = 0;
    timerbaseinit.TIM_CounterMode = TIM_CounterMode_Up;
    
    
    TIM_DeInit(TIM1);
    TIM_TimeBaseInit(TIM1, &timerbaseinit);
    
    TIM_DeInit(TIM2);
    TIM_TimeBaseInit(TIM2, &timerbaseinit);
    
    TIM_DeInit(TIM16);
    TIM_TimeBaseInit(TIM16, &timerbaseinit);
    
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
    
    #if defined(CX10_BLUE)
    
    gpioinitTIM.GPIO_Pin = GPIO_Pin_8 |GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    gpioinitTIM.GPIO_Mode = GPIO_Mode_AF;
    gpioinitTIM.GPIO_Speed = GPIO_Speed_50MHz;
    gpioinitTIM.GPIO_OType = GPIO_OType_PP;
    gpioinitTIM.GPIO_PuPd   = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpioinitTIM);
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_2);
    
    timerbaseinit.TIM_Prescaler = 1; // 24khz
    timerbaseinit.TIM_Period = MOTORPWM_DUTY_MAX;
    timerbaseinit.TIM_ClockDivision = TIM_CKD_DIV1;
    timerbaseinit.TIM_RepetitionCounter = 0;
    timerbaseinit.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_DeInit(TIM1);
    TIM_TimeBaseInit(TIM1, &timerbaseinit);
    
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

// Set duty cycle of motor outputs (if ENabled)
void set_motorpwm(int16_t throttle, int16_t *PIDdata, bool EN)
{
    int16_t motorpwm[4] = {0,0,0,0}; 
    if(EN)  mixer(throttle, PIDdata, motorpwm, MOTORPWM_DUTY_MIN, MOTORPWM_DUTY_MAX);

    #if defined(CX10_REDV1)
    TIM1->CCR1  = motorpwm[0]; // FL
    TIM1->CCR4  = motorpwm[1]; // FR
    TIM16->CCR1 = motorpwm[2]; // RR
    TIM2->CCR4  = motorpwm[3]; // RL
    #endif
    
    #if defined(CX10_BLUE)
    TIM1->CCR4 = motorpwm[0]; // FL
    TIM1->CCR3 = motorpwm[1]; // FR
    TIM1->CCR2 = motorpwm[2]; // RR
    TIM1->CCR1 = motorpwm[3]; // RL
    #endif
    
}

void mixer(int16_t throttle, int16_t *rpy, int16_t *motorpwm, uint16_t minpwm, uint16_t maxpwm)
{

    // Mix commanded roll/pitch/yaw into FL, FR, RR, RlL
    motorpwm[0] = throttle + rpy[0] + rpy[1] - rpy[2]; // Front left:  +roll, +pitch, -yaw
    motorpwm[1] = throttle - rpy[0] + rpy[1] + rpy[2]; // Front right: -roll, +pitch, +yaw
    motorpwm[2] = throttle - rpy[0] - rpy[1] - rpy[2]; // Rear right:  -roll, -pitch, -yaw
    motorpwm[3] = throttle + rpy[0] - rpy[1] + rpy[2]; // Rear left:   +roll, -pitch, +yaw
    
    // Contrain to minimum and maximum
    for(int i=0; i<4; i++) motorpwm[i] = constrain(motorpwm[i], minpwm, maxpwm);

}    











