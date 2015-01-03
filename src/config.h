// ===== CONFIG ===== //

// Cheerson Board Define
#define CX_10_RED_BOARD
//#define CX_10_GREEN_BOARD // ATM. not implemented
//#define CX_10_BLUE_BOARD



// Throttle settings
#define MIN_COMMAND 80 // controll starts if throttle is higher then that 
#define MIN_THROTTLE 50 // minimum speed for the motors

// P term 40 == 4.0
#define GYRO_P_ROLL  45
#define GYRO_P_PITCH 45
#define GYRO_P_YAW   70

// I term 40 == 0.040
#define GYRO_I_ROLL  65
#define GYRO_I_PITCH 65
#define GYRO_I_YAW   85

// D term 40 == 40
#define GYRO_D_ROLL  120
#define GYRO_D_PITCH 120
#define GYRO_D_YAW   0

// RC Settings
#define RC_RATE 460 // 100-990
#define RC_ROLL_RATE 88 // 0-100
#define RC_PITCH_RATE 88 // 0-100
#define RC_YAW_RATE 88 // 0-100

// order is Throttle,Roll,Pitch,Yaw,Aux1,Aux2
#define RC_CHAN_ORDER 0,1,2,3,4,5 // deltang ppm
//#define RC_CHAN_ORDER 2,0,1,3,4,5 // orangerx ppm



// just for Setting things up

// force Serial1 (pin A9 & A10).  unflyable (not needed for the red board)
//#define FORCE_SERIAL



// ===== CONFIG END ===== //


#if defined(CX_10_RED_BOARD) || defined(FORCE_SERIAL)
	#define SERIAL_ACTIVE
#endif

#if defined(CX_10_RED_BOARD)
	#define LEDon Bit_SET
	#define LEDoff Bit_RESET
	
	#define LED1_PORT GPIOB
	#define LED1_BIT GPIO_Pin_2
	
	#define LED2_PORT GPIOA
	#define LED2_BIT GPIO_Pin_15

	#define GYRO_ORIENTATION(X, Y, Z) {GyroXYZ[0] = -X; GyroXYZ[1] = -Y; GyroXYZ[2] = -Z;}
	#define ACC_ORIENTATION(X, Y, Z)  {ACCXYZ[0]  = -Y; ACCXYZ[1]  =  -X; ACCXYZ[2]  =  -Z;}
#endif

#if defined(CX_10_BLUE_BOARD)
	#define LEDon Bit_RESET
	#define LEDoff Bit_SET

	#define LED1_PORT GPIOB
	#define LED1_BIT GPIO_Pin_2
	
	#define LED2_PORT GPIOB
	#define LED2_BIT GPIO_Pin_1

	#define GYRO_ORIENTATION(X, Y, Z) {GyroXYZ[0] = X; GyroXYZ[1] = Y; GyroXYZ[2] = -Z;}
	#define ACC_ORIENTATION(X, Y, Z)  {ACCXYZ[0]  = Y; ACCXYZ[1]  =  -X; ACCXYZ[2]  =  Z;}
#endif





//includes
#include "stm32f0xx_conf.h"
#include "adc.h"
#include "main.h"
#include "MPU6050.h"
#include "RX.h"
#include "timer.h"
#include "serial.h"

//defines
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

//globals
extern int16_t RXcommands[6];
extern int8_t Armed;
extern int16_t LiPoVolt;
extern int16_t GyroXYZ[3];
extern int16_t ACCXYZ[3];
extern int16_t I2C_Errors;
extern uint16_t calibGyroDone;
extern uint8_t failsave;
extern int16_t angle[3];