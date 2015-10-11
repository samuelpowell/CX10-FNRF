// main.c
//
// This file is part of the CX10_fnrf project, released under the
// GNU General Public License, see LICENSE.md for further details.
//
// Copyright � 	2015 Samuel Powell
//							2015 Bart Slinger
//							2014 Felix Niessen

#include "config.h"

static uint8_t TelMtoSend = 0;
static uint16_t minCycleTime = 2000;

static int8_t answerStayTime = 0;
static uint16_t LiPoEmptyWaring = 0;
uint8_t nx[2] = {'\n','\r'};
uint8_t TelRXThrottle[10] = {'T','h','r','o','t','t','l','e',' ',' '};
uint8_t TelRXRoll[10] = {'R','o','l','l',' ',' ',' ',' ',' ',' '};
uint8_t TelRXPitch[10] = {'P','i','t','c','h',' ',' ',' ',' ',' '};
uint8_t TelRXYaw[10] = {'Y','a','w',' ',' ',' ',' ',' ',' ',' '};
uint8_t TelRXAux1[10] = {'A','u','x','1',' ',' ',' ',' ',' ',' '};
uint8_t TelRXAux2[10] = {'A','u','x','2',' ',' ',' ',' ',' ',' '};
uint8_t TelLiPoVolt[10] = {'L','i','P','o',' ','V','o','l','t','.'};
uint8_t TelGX[10] = {'G','y','r','o',' ','X',' ',' ',' ',' '};
uint8_t TelGY[10] = {'G','y','r','o',' ','Y',' ',' ',' ',' '};
uint8_t TelGZ[10] = {'G','y','r','o',' ','Z',' ',' ',' ',' '};
uint8_t TelAX[10] = {'A','C','C',' ','X',' ',' ',' ',' ',' '};
uint8_t TelAY[10] = {'A','C','C',' ','Y',' ',' ',' ',' ',' '};
uint8_t TelAZ[10] = {'A','C','C',' ','Z',' ',' ',' ',' ',' '};
uint8_t TelDefaultAnswer[10] = {'H','o','d','o','r','!',' ',' ',' ',' '};

int16_t RXcommands[6] = {0,500,500,500,-500,500};
int8_t Armed = 0;
int16_t LiPoVolt = 0;
int16_t GyroXYZ[3] = {0,0,0};
int16_t ACCXYZ[3] = {0,0,0};
int16_t angle[3] = {0,0,0};
int16_t I2C_Errors = 0;
uint16_t calibGyroDone = 500;
uint8_t failsafe = 100;


uint8_t mode = 0;


uint8_t G_P[3] = {GYRO_P_ROLL,GYRO_P_PITCH,GYRO_P_YAW};
uint8_t G_I[3] = {GYRO_I_ROLL,GYRO_I_PITCH,GYRO_I_YAW};
uint8_t G_D[3] = {GYRO_D_ROLL,GYRO_D_PITCH,GYRO_D_YAW};

uint16_t RC_Rate = RC_RATE;
uint8_t RPY_Rate[3] = {RC_ROLL_RATE,RC_PITCH_RATE,RC_YAW_RATE};
int16_t Imax[3] = {18000,18000,5000};



int main(void)
{
    SystemInit();
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_USART1 | RCC_APB2Periph_TIM17 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM1 | RCC_APB2Periph_SYSCFG, ENABLE);
    
    // Initialise peripherals
    init_timer();
    init_ADC();
    init_blinker();
    init_motorpwm();
    init_MPU6050();
    init_rf();
    
#if defined(SERIAL_ACTIVE)
    init_UART(115200);
#endif
    
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
    set_blink_style(BLINKER_BIND);
    bind_rf();
    set_blink_style(BLINKER_DISARM);
    
    
    // Start main FC loop
    while(1) {
        
        static uint32_t last_Time = 0;
        static uint8_t CalibDelay = 20;
        static int32_t lastError[3] = {0,0,0};
        static int32_t Isum[3] = {0,0,0};
        static int16_t RPY_useRates[3] = {0,0,0};
        static int16_t setpoint[3] = {0,0,0};
        static int16_t PIDdata[3] = {0,0,0};
        static int16_t LastDt[3];
        uint8_t i = 0;
        
        // Record start of cycle
        uint32_t CycleStart = micros();
        
        if(calibGyroDone > 0 && CalibDelay == 0) ReadMPU();
        
        
        if(CalibDelay == 0 && calibGyroDone == 0)
        {
            ReadMPU();
            rx_rf();
            
            // Check RF failsafe, null throttle and disarm on failure
            if(failsafe > 10)
            {
                RXcommands[0] = 0;      // Kill throttle
                Armed = false;
            }
            
            // Device may arm when AUX1 is high, throttle is low, and RF is good
            if(RXcommands[4] > 150)
            {
                if(Armed == 0 && failsafe < 10 && RXcommands[0] <= 150) {
                    Armed = 1;
                    set_blink_style(BLINKER_ON);
                }
            }
            else
            {
                Armed = 0;
                set_blink_style(BLINKER_DISARM);
            }
            
            // Get setpooints
            for(i=0;i<3;i++){
                RPY_useRates[i] = 100-(uint32_t)((abs(RXcommands[i+1])*2)*RPY_Rate[i])/1000;
                if(mode == 0){ // HH mode
                    setpoint[i] = ((RXcommands[i+1])*RC_Rate/100);
                }
            }
            
            // PID controller (time is not implemented because of a fix loop time)
            for(i=0;i<3;i++){
                //error
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
                
                //combine
                if(i == 2){
                    PIDdata[i] = constrain(PT+IT+DT,-500,+500);
                }else{
                    PIDdata[i] = PT+IT+DT;
                }
            }
            
            // Set motor duty cycle
            set_motorpwm(PIDdata, Armed && (RXcommands[0] > MIN_COMMAND));
            
        }
        else
        {
            failsafe = 100;
        }
        
        // Sample the battery voltage
        ADC_StartOfConversion(ADC1);
        
        if(millis()-last_Time > 100) // 10Hz
        {
            failsafe++; // RX should send with ~50Hz so it should not be higher then 10 as long as there is a good signal
            last_Time = millis();
            
            TelMtoSend = 15;
            if(answerStayTime > 0) answerStayTime--;
            if(CalibDelay > 0){
                if(CalibDelay%2)GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDon);
                else GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDoff);
                CalibDelay--;
            }
            
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
        #if defined(SERIAL_ACTIVE)
        while (serial_available()){
            serial_read();
            answerStayTime = 20;
        }
        #endif
        
        while(micros()-CycleStart<minCycleTime){
            #if defined(SERIAL_ACTIVE)
            if(TelMtoSend > 1 || (answerStayTime > 0 && TelMtoSend > 0)){
                TelMtoSend--;
                switch(TelMtoSend){
                    case 14:
                        serial_send_bytes(TelRXThrottle,10);
                        print_int16(RXcommands[0]);
                        serial_send_bytes(nx,2);
                        break;
                    case 13:
                        serial_send_bytes(TelRXRoll,10);
                        print_int16(RXcommands[1]);
                        serial_send_bytes(nx,2);
                        break;
                    case 12:
                        serial_send_bytes(TelRXPitch,10);
                        print_int16(RXcommands[2]);
                        serial_send_bytes(nx,2);
                        break;
                    case 11:
                        serial_send_bytes(TelRXYaw,10);
                        print_int16(RXcommands[3]);
                        serial_send_bytes(nx,2);
                        break;
                    case 10:
                        serial_send_bytes(TelRXAux1,10);
                        print_int16(RXcommands[4]);
                        serial_send_bytes(nx,2);
                        break;
                    case 9:
                        serial_send_bytes(TelRXAux2,10);
                        print_int16(RXcommands[5]);
                        serial_send_bytes(nx,2);			
                        break;		
                    case 8:
                        serial_send_bytes(TelLiPoVolt,10);
                        print_int16(LiPoVolt);
                        serial_send_bytes(nx,2);			
                        break;
                    case 7:
                        serial_send_bytes(TelGX,10);
                        print_int16(GyroXYZ[0]);
                        serial_send_bytes(nx,2);			
                        break;	
                    case 6:
                        serial_send_bytes(TelGY,10);
                        print_int16(GyroXYZ[1]);
                        serial_send_bytes(nx,2);			
                        break;
                    case 5:
                        serial_send_bytes(TelGZ,10);
                        print_int16(GyroXYZ[2]);
                        serial_send_bytes(nx,2);			
                        break;	
                    case 4:
                        serial_send_bytes(TelAX,10);
                        print_int16(ACCXYZ[0]);
                        serial_send_bytes(nx,2);			
                        break;
                    case 3:
                        serial_send_bytes(TelAY,10);
                        print_int16(ACCXYZ[1]);
                        serial_send_bytes(nx,2);			
                        break;
                    case 2:
                        serial_send_bytes(TelAZ,10);
                        print_int16(ACCXYZ[2]);
                        serial_send_bytes(nx,2);			
                        break;					
                    case 1:
                        serial_send_bytes(nx,2);
                        serial_send_bytes(nx,2);
                        serial_send_bytes(nx,2);
                        break;
                    case 0:
                        serial_send_bytes(TelDefaultAnswer,10);
                        serial_send_bytes(nx,2);
                        serial_send_bytes(nx,2);
                        serial_send_bytes(nx,2);
                        break;
                }
            }
            #endif
        }
    }
}










