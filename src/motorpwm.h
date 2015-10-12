// motorpwm.h
//
// This file is part of the CX10_fnrf project, released under the
// GNU General Public License, see LICENSE.md for further details.
//
// Copyright © 	2015 Samuel Powell
//				2014 Felix Niessen

void init_motorpwm(void);
void set_motorpwm(int16_t *PIDdata, int16_t *RXcommands, bool EN);
