// blinker.h
// 
// This file is part of the CX10_fnrf project, released under the 
// GNU General Public License, see LICENSE.md for further details.
//
// Copyright © 	2015 Samuel Powell
#define BLINKER_OFF     0x00
#define BLINKER_ON      0x01
#define BLINKER_BIND    0X02
#define BLINKER_DISARM  0X03
#define BLINKER_LOWBAT  0x04

void init_blinker(void);
void set_blink_style(uint8_t style);
void TIM17_IRQHandler(void);
