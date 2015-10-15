// main.c
//
// This file is part of the CX10_fnrf project, released under the
// GNU General Public License, see LICENSE.md for further details.
//
// Copyright © 	2015 Samuel Powell
//              2015 Bart Slinger
//              2014 Felix Niessen

#include "config.h"
#include "math.h"

// Globals
int16_t LiPoVolt = 0;

// State enumerations
enum states { INIT, BIND, CALIBRATING, DISARMED, ARMED, ARMED_LOWBAT };
enum fmodes { RATE, ATTITUDE };

int main(void)
{
    // Power
    uint16_t LiPoEmptyWaring = 0;
    
    // Radio & command
    static const float cmd_rps_scale = RC_CMD_RPS_SCALE;
    static const float cmd_rad_scale = RC_CMD_RAD_SCALE;
 
    uint16_t failsafe = 1000;
    int16_t RXcommands[6] = {0,500,500,500,-500,-500};
    
    // IMU
    float gyr[3];   // Calibrated gyro readings [rad/s]
    float acc[3];   // Calibrated accelerometer readings [ms-^2]
    static float imu_pitch, imu_roll, imu_yaw, req_pitch, req_roll;
    static const uint8_t RPY_Rate[3] = {RC_ROLL_RATE,RC_PITCH_RATE,RC_YAW_RATE};
    static const int16_t Imax[3] = {18000,18000,5000};
    static const uint8_t G_P[3] = {GYRO_P_ROLL,GYRO_P_PITCH,GYRO_P_YAW};
    static const uint8_t G_I[3] = {GYRO_I_ROLL,GYRO_I_PITCH,GYRO_I_YAW};
    static const uint8_t G_D[3] = {GYRO_D_ROLL,GYRO_D_PITCH,GYRO_D_YAW};
    static const float pid_scale = PID_PWM_SCALE;
    static const float pid_attd_p = PID_ATTD_P;
    int16_t I2C_Errors = 0;
    uint16_t calibGyroDone = IMU_CALIB_CYCLES;
    
    // PID
    int32_t lastError[3] = {0,0,0};
    int32_t Isum[3] = {0,0,0};
    int16_t RPY_useRates[3] = {0,0,0};
    static float setpoint[3] = {0,0,0}; // Static for debug purposes
    int16_t PIDdata[3] = {0,0,0};
    int16_t LastDt[3];
    
    // Time
    static int32_t timer_imu_start = 0;
    static int32_t timer_imu =0;
    static const uint16_t minCycleTime = 2000;
    static uint32_t looptime;
    uint32_t last_Time = 0;
    
    // Device state and flight mode
    static enum states state = INIT;
    static enum states state_next = INIT;
    static enum fmodes fmode = RATE;
    
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
    
    // Enable 3.3v LDO on blue board (held high by cap charge?)
    #if defined(CX10_BLUE)
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
    
    // Initialise peripherals
    init_timer();
    init_ADC();
    init_blinker();
    init_motorpwm();
    init_MPU6050(&I2C_Errors);
    init_rf();
    
    // Set first state transition
    state_next = BIND;
    
    // Start main FC loop
    while(1) {
        
        state = state_next;                 // Set to next state
        uint32_t CycleStart = micros();     // Record start of cycle
        
        switch(state)
        {
            case INIT:
                // It should never come to this
                break;
            
            case BIND:
                // Bind RF, when finished, move to calibration state
                set_blink_style(BLINKER_BIND);
                bind_rf();
                state_next = CALIBRATING;            
                break;
            
            case CALIBRATING:
                // Run calibration cycle, when finished, move to disarmed state
                set_blink_style(BLINKER_BIND);
                if(calibGyroDone > 0) ReadMPU(gyr, acc, &I2C_Errors, &calibGyroDone);
                else state_next = DISARMED;
                break;
                           
            case DISARMED:
                // Await a valid arming request, move to armed state when received
                set_blink_style(BLINKER_DISARM);
                failsafe = rx_rf(RXcommands) ? 0 : constrain(failsafe+1, 0, 500);
                if(RXcommands[4] > 150 && failsafe < 500 && RXcommands[0] <= 150) state_next = ARMED;  
                break;
                        
            case ARMED:
            case ARMED_LOWBAT:    
                // Device is armed, process flight control and write motors, move to
                // disarm if there is an RF failure or if it commanded.
                set_blink_style(BLINKER_ON);
                ReadMPU(gyr, acc, &I2C_Errors, &calibGyroDone);
                failsafe = rx_rf(RXcommands) ? 0 : constrain(failsafe+1, 0, 500);
                
                // Disarm on RF failsafe or user command
                if(failsafe > 500 || RXcommands[4] <= 150)
                {
                    RXcommands[0] = 0;
                    state_next = DISARMED;
                }
                
                // Update IMU
                timer_imu_start = micros();
                update_imu(gyr[0], gyr[1], gyr[2], acc[0], acc[1], acc[2]);
                timer_imu = micros()-timer_imu_start;
                
                // Choose flight mode based upon AUX2
                if(RXcommands[5] < 0) fmode = RATE;
                if(RXcommands[5] >= 0) fmode = ATTITUDE;
                
                switch(fmode)
                {
                    case RATE:
                        for(int i=0; i<3; i++)
                        {
                            // Configure rate controller set-point according to command [rad/s], 
                            // and set feedforward scaling for open loop at maximum commanded rate.
                            
                            setpoint[i] = RXcommands[i+1]*cmd_rps_scale;
                            
                            // Disable feedforward
                            for(int i=0; i<3; i++) RPY_useRates[i] = 100;
                            RPY_useRates[i] = 100-(uint32_t)((abs(RXcommands[i+1])*2)*RPY_Rate[i])/1000;   
                        }   
                        break;
                        
                    case ATTITUDE:
                        
                        // Drive rate controller setpoint [rad/s] with outer controller to minimise
                        // attitude error [rad].
                    
                        // Calculate IMU RPY (quaternian to Tait-Bryan) [rad]
                        imu_roll  = atan2f(q2*q3 + q0*q1, 0.5f - (q1*q1 + q2*q2));
                        imu_pitch = asinf(-2.0f*(q1*q3 - q0*q2));
                        //imu_yaw =   atan2f(q1*q2 + q0*q3, 0.5f - (q2*q2 + q3*q3));
                    
                        // Calculate commanded attitude [rad]
                        req_roll =  RXcommands[1]*cmd_rad_scale;
                        req_pitch = RXcommands[2]*cmd_rad_scale;
                        
                        // Implement Q&D proprtional controller
                        setpoint[0] = pid_attd_p*(req_roll-imu_roll);
                        setpoint[1] = pid_attd_p*(req_pitch-imu_pitch);

                        
                        // Disable feedforward
                        for(int i=0; i<3; i++) RPY_useRates[i] = 100;
                        break;
                }
                
                // Yaw is always rate mode
                setpoint[2] = RXcommands[3]*cmd_rps_scale;
                
                // PID controller (time is not implemented because of a fix loop time)
                for(int i=0; i<3; i++)
                {
                    // Error (with feedforward correction)
                    // Note the correction PID_PWM_SCALE, which allows the old PID gains to be
                    // employed with the calibrated data.
                    int32_t error = (setpoint[i]-(gyr[i]*(RPY_useRates[i]/100)))*pid_scale;
                    
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
                
               
                
                // Sample the battery voltage
                ADC_StartOfConversion(ADC1);
                
                if(millis()-last_Time > 100) // 10Hz
                {
                    
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
        
         // Always set motor duty cycle
        bool EN =  (state_next == ARMED) && (state == ARMED) && (RXcommands[0] > 10);
        set_motorpwm(RXcommands[0], PIDdata, EN);
        
        looptime = micros()-CycleStart;
        while(micros()-CycleStart<minCycleTime);
           
    }
}

