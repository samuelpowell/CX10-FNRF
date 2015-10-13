// imu.h
//
// This file is part of the CX10_fnrf project, released under the
// GNU General Public License, see LICENSE.md for further details.
//
// Copyright ? 	2015 Samuel Powell
//              2011 Sebastian Madgwick

// Madwick's C implementation of Mahobey's IMU alogirthm, released
// under the GPL. Code available in its original form:
// http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

#ifndef MahonyAHRS_h
#define MahonyAHRS_h

// Externals (defined in imu.c)
extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

// Prototypes
void update_imu(float gx, float gy, float gz, float ax, float ay, float az);

#endif
