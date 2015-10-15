// MPU6050.c
// 
// This file is part of the CX10_fnrf project, released under the 
// GNU General Public License, see LICENSE.md for further details.
//
// Copyright © 	2015 Samuel Powell
//				2014 Felix Niessen


#include "config.h"
#include <math.h>   

#define MPU_address                   (0x68<<1) 


void I2C_WrReg(uint8_t Reg, uint8_t Val, int16_t *I2C_Errors){
	
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);

	I2C_TransferHandling(I2C1, MPU_address, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
	
	uint16_t test = 0;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET){
		test++;
		if(test > 50000){ (*I2C_Errors)++; return;}
	}

	I2C_SendData(I2C1, Reg);
	
	test = 0;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TCR) == RESET){
		test++;
		if(test > 50000){ (*I2C_Errors)++; return;}
	}

	I2C_TransferHandling(I2C1, MPU_address, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);
	
	test = 0;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET){
		test++;
		if(test > 50000){ (*I2C_Errors)++; return;}
	}

	I2C_SendData(I2C1, Val);
	
	test = 0;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET){
		test++;
		if(test > 50000){ (*I2C_Errors)++; return;}
	}

	I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);

}


// Returns baseline corrected Gyro and Acc readings
void ReadMPU(float *gyr, float *acc, int16_t *I2C_Errors, uint16_t *calibGyroDone)
 {
	static uint8_t i = 0;
	static int32_t calibGyro[3] = {0,0,0};
    static int32_t calibAcc[3] = {0,0,0};
    static int16_t GyroXYZ[3] = {0,0,0};
    static int16_t ACCXYZ[3] = {0,0,0};
     
	uint8_t I2C_rec_Buffer[14];
	
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);

	I2C_TransferHandling(I2C1, MPU_address, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
	
	uint16_t test = 0;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET){
		test++;
		if(test > 50000){ (*I2C_Errors)++; return;}
	}

	I2C_SendData(I2C1, (uint8_t)0x3B);
	
	test = 0;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TC) == RESET){
		test++;
		if(test > 50000){ (*I2C_Errors)++; return;}
	}

	I2C_TransferHandling(I2C1, MPU_address, 14, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

	for(i = 0; i<14; i++){
		
		test = 0;
		while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET){
			test++;
			if(test > 50000){ (*I2C_Errors)++; return;}
		}

		I2C_rec_Buffer[i] = I2C_ReceiveData(I2C1);
	}
	
	test = 0;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET);

	I2C_ClearFlag(I2C1, I2C_FLAG_STOPF); 

	ACC_ORIENTATION( (int16_t)((I2C_rec_Buffer[0]<<8) | I2C_rec_Buffer[1]),
		(int16_t)((I2C_rec_Buffer[2]<<8) | I2C_rec_Buffer[3]),
		(int16_t)((I2C_rec_Buffer[4]<<8) | I2C_rec_Buffer[5]));	
	
	GYRO_ORIENTATION( (int16_t)((I2C_rec_Buffer[8]<<8) | I2C_rec_Buffer[9]),
		(int16_t)((I2C_rec_Buffer[10]<<8) | I2C_rec_Buffer[11]),
		(int16_t)((I2C_rec_Buffer[12]<<8) | I2C_rec_Buffer[13]));
	
	if(*calibGyroDone > 0){
		for(i = 0; i<3;i++){
			calibGyro[i]+= GyroXYZ[i];
            calibAcc[i] += ACCXYZ[i];
		}
		
		if(*calibGyroDone%2)GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDoff);	
		else GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDon);	
			
		
		if(*calibGyroDone == 1){
			for(i = 0; i<3;i++){
				calibGyro[i]= calibGyro[i]/IMU_CALIB_CYCLES;
                calibAcc[i] = calibAcc[i]/IMU_CALIB_CYCLES;
			}
            // Add back in gravity (1g) in +ve Z-axis. Calibration is subtracted
            // so this is entered as a negative value.
            calibAcc[2] -= 4096;
            
			GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDon);			
		}
		(*calibGyroDone)--;
	}else{
		for(i = 0; i<3;i++){
			GyroXYZ[i] -= calibGyro[i];
            ACCXYZ[i] -= calibAcc[i];
            gyr[i] = (float)GyroXYZ[i]*0.0010642252f;      // Range = +/- 2000 dps (16.4 LSBs/DPS)
            acc[i] = (float)ACCXYZ[i]*0.00239502f;         // Range = +/- 8 g (4096 lsb/g)
            
		}
	}
}
 



void init_MPU6050(int16_t *I2C_Errors){ 
	GPIO_InitTypeDef gpioinitI2C1;
	gpioinitI2C1.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	gpioinitI2C1.GPIO_Mode = GPIO_Mode_AF;
	gpioinitI2C1.GPIO_OType = GPIO_OType_OD;
	gpioinitI2C1.GPIO_PuPd = GPIO_PuPd_UP;//NOPULL;
	GPIO_Init(GPIOB, &gpioinitI2C1);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_1);
	
	I2C_InitTypeDef initI2C1;
	initI2C1.I2C_Timing = 0x0010020A;
	initI2C1.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	initI2C1.I2C_DigitalFilter = 0;
	initI2C1.I2C_Mode = I2C_Mode_I2C;
	initI2C1.I2C_OwnAddress1 = 0xAB;
	initI2C1.I2C_Ack = I2C_Ack_Enable;
	initI2C1.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1, &initI2C1);
	I2C_Cmd(I2C1, ENABLE);	
	
	delay_micros(5000); 
	I2C_WrReg(0x6B, 0x80, I2C_Errors);
	delay_micros(5000); 
	I2C_WrReg(0x6B, 0x03, I2C_Errors); 
	I2C_WrReg(0x1A, 0, I2C_Errors);     // Set LPF to minimum on Gyro and Acc
	I2C_WrReg(0x1B, 0x18, I2C_Errors);  // Configure Gyro 2000deg/s full scale
	I2C_WrReg(0x1C, 0x10, I2C_Errors);	// Configure Acc +- 8g full scale
	
}







