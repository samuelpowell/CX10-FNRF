// main.c
//
// This file is part of the CX10_fnrf project, released under the
// GNU General Public License, see LICENSE.md for further details.
//
// Copyright � 	2015 Samuel Powell
//							2015 Bart Slinger
//							2014 Felix Niessen

#include "config.h"

int16_t LiPoVolt = 0;
uint8_t failsafe = 100;
uint8_t mode = 0;

enum states { INIT, BIND, CALIBRATING, DISARMED, ARMED, ARMED_LOWBAT };

int main(void)
{
    uint32_t last_Time = 0;
    uint8_t CalibDelay = 20;
    int32_t lastError[3] = {0,0,0};
    int32_t Isum[3] = {0,0,0};
    int16_t RPY_useRates[3] = {0,0,0};
    int16_t setpoint[3] = {0,0,0};
    int16_t PIDdata[3] = {0,0,0};
    int16_t LastDt[3];
    uint16_t LiPoEmptyWaring = 0;
    
    int16_t RXcommands[6] = {0,500,500,500,-500,500};
    int8_t Armed = 0;
    

    int16_t GyroXYZ[3] = {0,0,0};
    int16_t ACCXYZ[3] = {0,0,0};
    int16_t angle[3] = {0,0,0};
    int16_t I2C_Errors = 0;
    uint16_t calibGyroDone = 500;
    
    static const uint16_t RC_Rate = RC_RATE;
    static const uint8_t RPY_Rate[3] = {RC_ROLL_RATE,RC_PITCH_RATE,RC_YAW_RATE};
    static const int16_t Imax[3] = {18000,18000,5000};
    static const uint8_t G_P[3] = {GYRO_P_ROLL,GYRO_P_PITCH,GYRO_P_YAW};
    static const uint8_t G_I[3] = {GYRO_I_ROLL,GYRO_I_PITCH,GYRO_I_YAW};
    static const uint8_t G_D[3] = {GYRO_D_ROLL,GYRO_D_PITCH,GYRO_D_YAW};
    
    static const uint16_t minCycleTime = 2000;
    
    static enum states state = INIT;
    static enum states state_next = INIT;
    
    // Startup code calls SystemInit: system clock is configured 
    
    // Enable peripheral clocks    
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA
                         | RCC_AHBPeriph_GPIOB, ENABLE);
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1
                         | RCC_APB1Periph_TIM2
                         | RCC_APB1Periph_TIM3, ENABLE);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 
                         | RCC_APB2Periph_USART1
                         | RCC_APB2Periph_TIM17 
                         | RCC_APB2Periph_ADC1 
                         | RCC_APB2Periph_TIM16 
                         | RCC_APB2Periph_TIM1 
                         | RCC_APB2Periph_SYSCFG, ENABLE);
    
    // Initialise peripherals
    init_timer();
    init_ADC();
    init_blinker();
    init_motorpwm();
    init_MPU6050(&I2C_Errors);
    init_rf();
    
    
    #if defined(CX10_BLUE)
    // Enable 3.3v LDO
    GPIO_InitTypeDef LEDGPIOinit;
    LEDGPIOinit.GPIO_Pin = LED1_BIT;
    LEDGPIOinit.GPIO_Mode = GPIO_Mode_OUT;
    LEDGPIOinit.GPIO_Speed = GPIO_Speed_50MHz;
    LEDGPIOinit.GPIO_OType = GPIO_OType_PP;
    LEDGPIOinit.GPIO_PuPd   = GPIO_PuPd_NOPULL;
    GPIO_Init(LED1_PORT, &LEDGPIOinit);
    
    LEDGPIOinit.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOA, &LEDGPIOinit);
    GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
    #endif
    
    // Bind to TX
    
  
    
    state_next = BIND;
    
    // Start main FC loop
    while(1) {
        
        // Set to next state
        state = state_next;
        
        // Record start of cycle
        uint32_t CycleStart = micros();
        
        switch(state)
        {
            case BIND:
                
                set_blink_style(BLINKER_BIND);
                bind_rf();
                state_next = CALIBRATING;            
                break;
            
            case CALIBRATING:
                
                set_blink_style(BLINKER_BIND);
                    
                // TODO: This was originally called at 10Hz, so there will be no delay now
                // and rapidly flashing LEDs, fix with timer.
                if(CalibDelay > 0)
                {
                    if(CalibDelay%2)GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDon);
                    else GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDoff);
                    CalibDelay--;
                }
                
                
                if(calibGyroDone > 0 && CalibDelay == 0) 
                {
                    ReadMPU(GyroXYZ, ACCXYZ, angle, &I2C_Errors, &calibGyroDone);
                }
                else
                {
                    state_next = DISARMED;
                }
                break;
                           
            case DISARMED:
                
                set_blink_style(BLINKER_DISARM);
                
                rx_rf(RXcommands);
                // Device may arm when AUX1 is high, throttle is low, and RF is good
                if(RXcommands[4] > 150)
                {
                    if(Armed == 0 && failsafe < 10 && RXcommands[0] <= 150)
                    {
                        Armed = 1;
                        state_next = ARMED;    
                    }
                }      
                break;
                        
            case ARMED:
            case ARMED_LOWBAT:    
                
                set_blink_style(BLINKER_ON);
            
                ReadMPU(GyroXYZ, ACCXYZ, angle, &I2C_Errors, &calibGyroDone);
                rx_rf(RXcommands);
                
                // Disarm on RF failsafe or user command
                if(failsafe > 10 || RXcommands[4] <= 150)
                {
                    RXcommands[0] = 0;
                    Armed = false;
                    state_next = DISARMED;
                }
                
                // Get setpooints
                for(int i=0; i<3; i++)
                {
                    RPY_useRates[i] = 100-(uint32_t)((abs(RXcommands[i+1])*2)*RPY_Rate[i])/1000;
                    if(mode == 0)
                    { // HH mode
                        setpoint[i] = ((RXcommands[i+1])*RC_Rate/100);
                    }
                }
                
                // PID controller (time is not implemented because of a fix loop time)
                for(int i=0; i<3; i++)
                {
                    // Error
                    int32_t error = setpoint[i]-((GyroXYZ[i])*RPY_useRates[i]/100);
                    
                    // Proportional
                    int32_t PT = (error*G_P[i])/100;
                    
                    // Integral
                    int32_t IT = constrain(error+Isum[i],(Imax[i]*-1),Imax[i]);
                    if(abs(error) > 700)  IT = 0;
                    if(RXcommands[0] < MIN_COMMAND) IT = 0;
                    Isum[i] = IT;
                    IT = (IT*G_I[i])/3000;
                    
                    // Derivative
                    int16_t tmpDT = error-lastError[i];
                    int32_t DT= ((tmpDT+LastDt[i])*G_D[i])/300;
                    LastDt[i] = tmpDT;
                    lastError[i] = error;
                    
                    // Combine
                    if(i == 2)
                    {
                        PIDdata[i] = constrain(PT+IT+DT,-500,+500);
                    }
                    else
                    {
                        PIDdata[i] = PT+IT+DT;
                    }
                }
                
                // Set motor duty cycle
                set_motorpwm(PIDdata, RXcommands, Armed && (RXcommands[0] > MIN_COMMAND));
                
                
                // Sample the battery voltage
                ADC_StartOfConversion(ADC1);
                
                if(millis()-last_Time > 100) // 10Hz
                {
                    failsafe++; // RX should send with ~50Hz so it should not be higher then 10 as long as there is a good signal
                    last_Time = millis();
                   
                    if(LiPoEmptyWaring == 350)
                    {
                        set_blink_style(BLINKER_LOWBAT);
                        
                        #if defined(CX10_BLUE) // turn off to save the lipo
                        if(LiPoVolt < 250) GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
                        #endif
                        
                    }
                    else if(LiPoVolt < 300) LiPoEmptyWaring++;
                    else if(LiPoEmptyWaring > 10) LiPoEmptyWaring -= 10;
                }
        
        
                break;
                
        }
                
        while(micros()-CycleStart<minCycleTime);
           
    }
}










