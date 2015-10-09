// Target board
#define CX10_BLUE
//#define CX10_REDV1
//#define CX10_REDV2    // Not implemented
//#define CX10_GREEN    // Not implemented

// Throttle settings
#define MIN_COMMAND 80      // controll starts if throttle is higher then that 
#define MIN_THROTTLE 50     // minimum speed for the motors

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
#define RC_RATE 460         // 100-990
#define RC_ROLL_RATE 88     // 0-100
#define RC_PITCH_RATE 88    // 0-100
#define RC_YAW_RATE 88      // 0-100

// Channel order (TAER12)
#define RC_CHAN_ORDER 0,1,2,3,4,5

// Force serial output (pin A9 & A10) 
// Serial is active by default on REDV1, and can be anabled for other
// boards, though they cannot be flown whilst in this mode.
//#define FORCE_SERIAL

// Configure radio chipset and protocol according to device
#ifdef CX10_REDV1
    #define RF_CHSET_NRF24
    #define RF_PROTO_REDV1
#endif

#ifdef CX10_REDV2
    #define RF_CHSET_XN297
    #define RF_PROTO_BLUE
#endif

#ifdef CX10_BLUE
    #define RF_CHSET_XN297
    #define RF_PROTO_BLUE
#endif

#ifdef CX10_BLUE
    #define RF_CHSET_XN297
    #define RF_PROTO_GREEN
#endif

#if defined(CX10_REDV1) || defined(FORCE_SERIAL)
    #define SERIAL_ACTIVE
#endif

#if defined(CX10_REDV1)
	#define LEDon Bit_SET
	#define LEDoff Bit_RESET
	
	#define LED1_PORT GPIOB
	#define LED1_BIT GPIO_Pin_2
	
	#define LED2_PORT GPIOA
	#define LED2_BIT GPIO_Pin_15
	
	#define GYRO_ORIENTATION(X, Y, Z) {GyroXYZ[0] = -X; GyroXYZ[1] = -Y; GyroXYZ[2] = -Z;}
	#define ACC_ORIENTATION(X, Y, Z)  {ACCXYZ[0]  = -Y; ACCXYZ[1]  =  -X; ACCXYZ[2]  =  -Z;}

    #define RADIO_SPI_PORT            GPIOA
    #define RADIO_SPI_CS_PORT         GPIOA
    #define RADIO_SPI                 SPI1
	#define RADIO_GPIO_SPI_CS         GPIO_Pin_4
	#define RADIO_GPIO_SPI_SCK        GPIO_Pin_5
	#define RADIO_GPIO_SPI_MISO       GPIO_Pin_6
	#define RADIO_GPIO_SPI_MOSI       GPIO_Pin_7
#endif
    

#if defined(CX10_BLUE)
	#define LEDon Bit_RESET
	#define LEDoff Bit_SET

	#define LED1_PORT GPIOB
	#define LED1_BIT GPIO_Pin_2
	
	#define LED2_PORT GPIOB
	#define LED2_BIT GPIO_Pin_1

	#define GYRO_ORIENTATION(X, Y, Z) {GyroXYZ[0] = X; GyroXYZ[1] = Y; GyroXYZ[2] = -Z;}
	#define ACC_ORIENTATION(X, Y, Z)  {ACCXYZ[0]  = Y; ACCXYZ[1]  =  -X; ACCXYZ[2]  =  Z;}

    #define RADIO_SPI_PORT            GPIOB
    #define RADIO_SPI_CS_PORT         GPIOA
    #define RADIO_SPI                 SPI1
    #define RADIO_GPIO_SPI_CS         GPIO_Pin_15
    #define RADIO_GPIO_SPI_SCK        GPIO_Pin_3
    #define RADIO_GPIO_SPI_MISO       GPIO_Pin_4
    #define RADIO_GPIO_SPI_MOSI       GPIO_Pin_5
    #define RADIO_GPIO_CE_PORT        GPIOB
    #define RADIO_GPIO_CE             GPIO_Pin_8
#endif

// System headers
#include <stdbool.h>
#include <string.h>
    
// Library headers
#include "stm32f0xx_conf.h"    
    
// Application headers
#include "adc.h"
#include "main.h"
#include "MPU6050.h"
#include "timer.h"
#include "serial.h"
#include "nrf24.h"

#if defined(RF_PROTO_BLUE)
    #include "proto_blue.h"
#endif

#if defined(RF_PROTO_REDV1)
    #include "proto_redv1.h"
#endif

// Preprocessor functions
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// Global variables
extern int16_t RXcommands[6];
extern int8_t Armed;
extern int16_t LiPoVolt;
extern int16_t GyroXYZ[3];
extern int16_t ACCXYZ[3];
extern int16_t I2C_Errors;
extern uint16_t calibGyroDone;
extern uint8_t failsave;
extern int16_t angle[3];
