/*
	STM32F05x brushed Copter FW v1.0
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
	
	
	inspired by MultiWiiCopter by Alexandre Dubus
*/
#include "config.h"

static uint8_t TelMtoSend = 0;
static uint16_t minCycleTime = 2000;
static uint16_t T3OV = 0;
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

int16_t RXcommands[6];
int8_t Armed = 0;
int16_t LiPoVolt = 0;
int16_t GyroXYZ[3];
int16_t ACCXYZ[3];
int16_t angle[3];
int16_t I2C_Errors;
uint16_t calibGyroDone;
uint8_t failsave = 100;


uint8_t mode = 0;
uint8_t OkToArm = 0;

uint8_t G_P[3] = {GYRO_P_ROLL,GYRO_P_PITCH,GYRO_P_YAW};
uint8_t G_I[3] = {GYRO_I_ROLL,GYRO_I_PITCH,GYRO_I_YAW};
uint8_t G_D[3] = {GYRO_D_ROLL,GYRO_D_PITCH,GYRO_D_YAW};

uint16_t RC_Rate = RC_RATE;
uint8_t RPY_Rate[3] = {RC_ROLL_RATE,RC_PITCH_RATE,RC_YAW_RATE};
int16_t Imax[3] = {18000,18000,5000};


void TIM3_IRQHandler(void){
	if(TIM3->SR & TIM_IT_Update){
		TIM3->SR = (uint16_t)~TIM_IT_Update;
		T3OV++;
	}
}


uint32_t micros(){
	return (T3OV<<16)+(TIM3->CNT);
}

uint32_t millis(){
	return (micros()/1000);
}

void delayMicroseconds(uint32_t us){
	uint32_t now = micros();
	while (micros() - now < us);
}


int main(void){
	SystemInit();

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM1 | RCC_APB2Periph_SYSCFG, ENABLE);
	
	//init
	init_Timer();
	init_ADC();
	#if defined(SERIAL_ACTIVE)
	init_UART(115200);
	#endif
	init_PPMRX();
	init_MPU6050();
	
	GPIO_InitTypeDef LEDGPIOinit;
	LEDGPIOinit.GPIO_Pin = LED1_BIT;
	LEDGPIOinit.GPIO_Mode = GPIO_Mode_OUT;
	LEDGPIOinit.GPIO_Speed = GPIO_Speed_50MHz;
	LEDGPIOinit.GPIO_OType = GPIO_OType_PP; 
	LEDGPIOinit.GPIO_PuPd   = GPIO_PuPd_NOPULL;
	GPIO_Init(LED1_PORT, &LEDGPIOinit);	
	
	LEDGPIOinit.GPIO_Pin = LED2_BIT;
	GPIO_Init(LED2_PORT, &LEDGPIOinit);	
	
	GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDoff);
	GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDoff);
	
	#if defined(CX_10_BLUE_BOARD)
		LEDGPIOinit.GPIO_Pin = GPIO_Pin_5; // 3,3V LDO enable
		GPIO_Init(GPIOA, &LEDGPIOinit);	
		
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
	#endif
	
	
	
	while(1){
		static uint32_t last_Time = 0;
		static uint8_t CalibDelay = 20;
		static int32_t lastError[3] = {0,0,0};
		static int32_t Isum[3] = {0,0,0};
		static int16_t RPY_useRates[3] = {0,0,0};
		static int16_t setpoint[3] = {0,0,0}; 
		static int16_t PIDdata[3] = {0,0,0};   
		static int16_t LastDt[3];
		uint8_t i = 0;
		
		uint32_t CycleStart = micros();
		
		if(calibGyroDone > 0 && CalibDelay == 0) ReadMPU();
		if(CalibDelay == 0 && calibGyroDone == 0){
			
			//collect datas
			ReadMPU();
			getRXDatas();
			
			// get setpoint
			for(i=0;i<3;i++){
				RPY_useRates[i] = 100-(uint32_t)((abs(RXcommands[i+1])*2)*RPY_Rate[i])/1000;
				if(mode == 0){ // HH mode
					setpoint[i] = ((RXcommands[i+1])*RC_Rate/100);
				}
			}
			
			// PID controller... time is not implemented because of a fix loop time
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
				
			// Arm with Aux 1
			if(RXcommands[4] > 150){
				if(Armed == 0 && OkToArm == 250 &&  failsave < 10 && RXcommands[0] <= 150){
					Armed = 1;
					GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDon);
				}
			}else{
				if(Armed == 1){ 
					Armed = 0; 
					GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDoff);
				}
				if(OkToArm < 250 &&  failsave < 10) OkToArm++;
			}
			
			uint16_t motorMax = 0;
			uint16_t motorMin = 0;
			if(Armed){
				if(RXcommands[0] < MIN_COMMAND) motorMax = 0;
				else{
					motorMax = 1000; 
					motorMin = MIN_THROTTLE;
				}
			}
			
			#define MIX(X,Y,Z) constrain(RXcommands[0],0,1000) + PIDdata[0]*X + PIDdata[1]*Y + PIDdata[2]*Z
			
			// write Motors
			
			if(failsave > 10) RXcommands[0] = 0; // fall down
			
			#if defined(CX_10_RED_BOARD)
			TIM1->CCR1 = constrain(MIX(+1,-1,-1),motorMin,motorMax); // front left
			TIM1->CCR4 = constrain(MIX(-1,-1,+1),motorMin,motorMax); // front right
			TIM16->CCR1 = constrain(MIX(-1,+1,-1),motorMin,motorMax); // rear right
			TIM2->CCR4 = constrain(MIX(+1,+1,+1),motorMin,motorMax); // rear left
			#endif
			
			#if defined(CX_10_BLUE_BOARD)
			TIM1->CCR4 = constrain(MIX(+1,-1,-1),motorMin,motorMax); // front left
			TIM1->CCR3 = constrain(MIX(-1,-1,+1),motorMin,motorMax); // front right
			TIM1->CCR2 = constrain(MIX(-1,+1,-1),motorMin,motorMax); // rear right
			TIM1->CCR1 = constrain(MIX(+1,+1,+1),motorMin,motorMax); // rear left
			#endif
		}else failsave = 100; 

		ADC_StartOfConversion(ADC1);
		if(millis()-last_Time > 100){ // 10Hz
			failsave++; // RX should send with ~50Hz so it should not be higher then 10 as long as there is a good signal
			last_Time = millis(); 
			
			TelMtoSend = 15;
			if(answerStayTime > 0) answerStayTime--;
			if(CalibDelay > 0){
				if(CalibDelay%2)GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDon);	
				else GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDoff);	
				CalibDelay--;
			}
			static uint8_t blinker = 0;
			if(LiPoEmptyWaring == 350){
				blinker++;
				if(blinker%2){
					GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDoff);
					GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDon);
				}else{
					GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDon);
					GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDoff);
				}
				#if defined(CX_10_BLUE_BOARD) // turn off to save the lipo
				if(LiPoVolt < 250) GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
				#endif
			}else if(LiPoVolt < 300) LiPoEmptyWaring++;
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










