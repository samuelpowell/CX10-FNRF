/*	Cheerson CX-10 integrated RF rate mode firmware.
	 - NRF24 Radio driver header.

	This source is mostly dervived from the Crazyflie control firmware
	whose original GPLv3 license is repeated below.

	For more details see the Bitcraze website: http://www.bitcraze.se.

	All original components are provided under the GPLv3:

    Copyright (C) 2015, Samuel Powell.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * nrf24l01.h: nRF24L01(-p) PRX mode low level driver
 */

#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#include "config.h"

// Init and test of the connection to the chip
void nrfInit(void);
bool nrfTest(void);

// Interrupt routine
void nrfIsr( void);

// Defines
#define RADIO_RATE_250K 0
#define RADIO_RATE_1M 1
#define RADIO_RATE_2M 2

#define ACTIVATE_DATA   				0x73
#define ACTIVATE_BK2423_DATA   	0x53

#define DUMMY_BYTE    					0xA5

/* Registers address definition */
#define REG_CONFIG 0x00
#define REG_EN_AA 0x01
#define REG_EN_RXADDR 0x02
#define REG_SETUP_AW 0x03
#define REG_SETUP_RETR 0x04
#define REG_RF_CH 0x05
#define REG_RF_SETUP 0x06
#define REG_STATUS 0x07
#define REG_OBSERVE_TX 0x08
#define REG_RPD 0x09
#define REG_RX_ADDR_P0 0x0A
#define REG_RX_ADDR_P1 0x0B
#define REG_RX_ADDR_P2 0x0C
#define REG_RX_ADDR_P3 0x0D
#define REG_RX_ADDR_P4 0x0E
#define REG_RX_ADDR_P5 0x0F
#define REG_TX_ADDR 0x10
#define REG_RX_PW_P0 0x11
#define REG_RX_PW_P1 0x12
#define REG_RX_PW_P2 0x13
#define REG_RX_PW_P3 0x14
#define REG_RX_PW_P4 0x15
#define REG_RX_PW_P5 0x16
#define REG_FIFO_STATUS 0x17
#define REG_DYNPD 0x1C
#define REG_FEATURE 0x1D

#define VAL_RF_SETUP_250K 0x26
#define VAL_RF_SETUP_1M   0x06
#define VAL_RF_SETUP_2M   0x0E

#define VAL_SETUP_AW_3B 1
#define VAL_SETUP_AW_4B 2
#define VAL_SETUP_AW_5B 3

// Additional bits and pieces from, e.g., the NRF24 arduino library
#define ACTIVATE        0x50  // Activate command

// #define NRF24_REG_00_CONFIG                             0x00
#define NRF24_MASK_RX_DR                                0x40
#define NRF24_MASK_TX_DS                                0x20
#define NRF24_MASK_MAX_RT                               0x10
#define NRF24_EN_CRC                                    0x08
#define NRF24_CRCO                                      0x04
#define NRF24_PWR_UP                                    0x02
#define NRF24_PRIM_RX                                   0x01

// #define NRF24_REG_01_EN_AA                              0x01
#define NRF24_ENAA_P5                                   0x20
#define NRF24_ENAA_P4                                   0x10
#define NRF24_ENAA_P3                                   0x08
#define NRF24_ENAA_P2                                   0x04
#define NRF24_ENAA_P1                                   0x02
#define NRF24_ENAA_P0                                   0x01

// #define NRF24_REG_02_EN_RXADDR                          0x02
#define NRF24_ERX_P5                                    0x20
#define NRF24_ERX_P4                                    0x10
#define NRF24_ERX_P3                                    0x08
#define NRF24_ERX_P2                                    0x04
#define NRF24_ERX_P1                                    0x02
#define NRF24_ERX_P0                                    0x01

// #define NRF24_REG_03_SETUP_AW                           0x03
#define NRF24_AW_3_BYTES                                0x01
#define NRF24_AW_4_BYTES                                0x02
#define NRF24_AW_5_BYTES                                0x03

// #define NRF24_REG_04_SETUP_RETR                         0x04
#define NRF24_ARD                                       0xf0
#define NRF24_ARC                                       0x0f

// #define NRF24_REG_05_RF_CH                              0x05
#define NRF24_RF_CH                                     0x7f

// #define NRF24_REG_06_RF_SETUP                           0x06
#define NRF24_CONT_WAVE                                 0x80
#define NRF24_RF_DR_LOW                                 0x20
#define NRF24_PLL_LOCK                                  0x10
#define NRF24_RF_DR_HIGH                                0x08
#define NRF24_PWR                                       0x06
#define NRF24_PWR_m18dBm                                0x00
#define NRF24_PWR_m12dBm                                0x02
#define NRF24_PWR_m6dBm                                 0x04
#define NRF24_PWR_0dBm                                  0x06

#define NRF24_ENAA_PA (NRF24_ENAA_P0 | NRF24_ENAA_P1 | NRF24_ENAA_P2 | NRF24_ENAA_P3 | NRF24_ENAA_P4 | NRF24_ENAA_P5)
#define NRF24_ERX_PA (NRF24_ERX_P0 | NRF24_ERX_P1 | NRF24_ERX_P2 | NRF24_ERX_P3 | NRF24_ERX_P4 | NRF24_ERX_P5)
#define NRF_STATUS_CLEAR 0x70

// SPI commands
#define CMD_R_REG              	0x00
#define CMD_W_REG              	0x20
#define CMD_R_RX_PAYLOAD       	0x61
#define CMD_W_TX_PAYLOAD       	0xA0
#define CMD_FLUSH_TX           	0xE1
#define CMD_FLUSH_RX           	0xE2
#define CMD_REUSE_TX_PL        	0xE3
#define CMD_ACTIVATE           	0x50
#define CMD_ACTIVATE_BK2423    	0x53
#define CMD_RX_PL_WID          	0x60
#define CMD_W_ACK_PAYLOAD(P)  	(0xA8|(P&0x0F))
#define CMD_W_PAYLOAD_NO_ACK   	0xD0
#define CMD_NOP                	0xFF



/* Low level reg access
 * FIXME: the user should not need to access raw registers...
 */
unsigned char nrfReadReg(unsigned char address, char *buffer, int len);
unsigned char nrfRead1Reg(unsigned char address);
unsigned char nrfWriteReg(unsigned char address, char *buffer, int len);
unsigned char nrfWrite1Reg(unsigned char address, char byte);

//Interrupt access
void nrfSetInterruptCallback(void (*cb)(void));

// Low level functionality of the nrf chip
unsigned char nrfNop( void );
unsigned char nrfFlushRx( void );
unsigned char nrfFlushTx( void );
unsigned char nrfRxLength(unsigned int pipe);
unsigned char nrfActivate( void );
unsigned char nrfActivateBK2423( void );
unsigned char nrfWriteAck(unsigned int pipe, char *buffer, int len);
unsigned char nrfReadRX(char *buffer, int len);
void nrfSetChannel(unsigned int channel);
void nrfSetDatarate(int datarate);
void nrfSetAddress(unsigned int pipe, char* address);
void nrfSetEnable(bool enable);
unsigned char nrfGetStatus( void );




#endif
