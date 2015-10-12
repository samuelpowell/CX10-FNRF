// rfproto.h
//
// This file is part of the CX10_fnrf project, released under the
// GNU General Public License, see LICENSE.md for further details.
//
// Copyright ? 	2015 Samuel Powell
//              2015 Goebish
//              2015 Bart Slinger

void init_rf(void);
void bind_rf(void);
bool rx_rf(int16_t *RXcommands);

