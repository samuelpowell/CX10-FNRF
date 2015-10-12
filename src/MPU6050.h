/*
	This file is part of STM32F05x brushed Copter FW
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
*/
void ReadMPU(int16_t *GyroXYZ, int16_t *ACCXYZ, int16_t *angle, int16_t *I2C_Errors, uint16_t *calibGyroDone);
void I2C_WrReg(uint8_t Reg, uint8_t Val, int16_t *I2C_Errors);
void init_MPU6050(int16_t *I2C_Errors);
long atan2_c(long x , long y);


