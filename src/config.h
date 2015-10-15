// Target board
#define CX10_BLUE
//#define CX10_REDV1
//#define CX10_REDV2    // Not implemented
//#define CX10_GREEN    // Not implemented

// Throttle settings
#define MIN_COMMAND  80      // controll starts if throttle is higher then that 
#define MIN_THROTTLE 20     // minimum speed for the motors

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

// Rate mode feedforward scaling [%]
#define RC_ROLL_RATE 88     // 0-100
#define RC_PITCH_RATE 88    // 0-100
#define RC_YAW_RATE 88      // 0-100

// Math floating point constants
#define PI_F                3.14159265358979323f
#define PI_2_F              1.57079632679489661f
#define PI_4_F              0.78539816339744830f

// RC calibrated settings
#define RC_MAX_RPS          PI_F            // Maximum rate commanded [rad/s]
#define RC_MAX_RAD          PI_4_F          // Maximum attitude command [rad]

#define RC_CMD_RPS_SCALE    RC_MAX_RPS/500  // Convert stick to rate in rad/s
#define RC_CMD_RAD_SCALE    RC_MAX_RAD/500  // Convert stick to attitude in rad

// PWM scaling
#define PID_PWM_SCALE       100             // Convert rate PID to duty cycle

// Attitude PID (scale [rad] error to [rad/s] setpoint on rate controller)
#define PID_ATTD_P          10.0f

// Channel order (TAER12)
#define RC_CHAN_ORDER 0,1,2,3,4,5

// Array elements
#define R    0
#define P    1
#define Y    2

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

    // +ve x is the forward longitudinal
    // +ve y is starboard
    // +ve z is downwards
	#define GYRO_ORIENTATION(X, Y, Z) {GyroXYZ[0] = X; GyroXYZ[1] = -Y; GyroXYZ[2] = -Z;}
	#define ACC_ORIENTATION(X, Y, Z)  {ACCXYZ[0]  = -X; ACCXYZ[1]  = +Y; ACCXYZ[2]  = Z;}

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
    
#define IMU_CALIB_CYCLES    500

// System headers
#include <stdbool.h>
#include <string.h>
    
// Library headers
#include "stm32f0xx_conf.h"    
    
// Application headers
#include "adc.h"
#include "MPU6050.h"
#include "timer.h"
#include "nrf24.h"
#include "blinker.h"
#include "motorpwm.h"
#include "timer.h"
#include "rfproto.h"
#include "imu.h"

// Preprocessor functions
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// Global variables
extern int16_t LiPoVolt;



