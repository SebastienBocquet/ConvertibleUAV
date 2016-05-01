// This file is part of MatrixPilot.
//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009-2011 MatrixPilot Team
// See the AUTHORS.TXT file for a list of authors of MatrixPilot.
//
// MatrixPilot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MatrixPilot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MatrixPilot.  If not, see <http://www.gnu.org/licenses/>.

#include "defines.h" 

// servo_ratios are used to convert degrees of rotation into servo pulse code lengths
// This code is configured for the full throw of the servo to be achieved by a range of
// 2000 units being sent to udb_pwOut. (i.e. min deflection 2000, centered 3000, max deflection 4000)
#define SONAR_PITCH_SERVO_HIGH_RATIO  ((2000.0 / ((SONAR_PITCH_SERVO_THROW / 360.0) * 65536.0)) * 65536.0)
#define SONAR_PITCH_SERVO_RATIO        (2000.0 / ((SONAR_PITCH_SERVO_THROW / 360.0) * 65536.0)) 

int16_t sonar_pitch_servo_pwm_delta = 0;  // Change in PWM pulse value from centred value (3000) to send to camera pitch servo

const int16_t sonar_pitch_servo_high_ratio = SONAR_PITCH_SERVO_HIGH_RATIO;

// Note that most angles in cameraCntrl.c are 16 bit quantities
// For example, 90 degrees is represented as 16384 (65536 / 4)
const int16_t sonar_tan_pitch_in_stabilized_mode = SONAR_TAN_PITCH_IN_STABILIZED_MODE;

const int16_t sonar_pitch_offset_centred_pwm = (SONAR_PITCH_OFFSET_CENTRED * 65536.0 / 360.0) * SONAR_PITCH_SERVO_RATIO;
const int16_t sonar_pitch_offset_pwm = (SONAR_PITCH_OFFSET * 65536.0 / 360.0) * SONAR_PITCH_SERVO_RATIO;

const int16_t sonar_pitch_servo_pwm_max = ((SONAR_PITCH_SERVO_MAX - SONAR_PITCH_OFFSET_CENTRED) * 65536.0 / 360.0) * SONAR_PITCH_SERVO_RATIO;					;
const int16_t sonar_pitch_servo_pwm_min = ((SONAR_PITCH_SERVO_MIN - SONAR_PITCH_OFFSET_CENTRED) * 65536.0 / 360.0) * SONAR_PITCH_SERVO_RATIO;

int32_t sonar_pitchServoLimit(int32_t pwm_pulse)
{
	if (pwm_pulse > sonar_pitch_servo_pwm_max) pwm_pulse = sonar_pitch_servo_pwm_max;
	if (pwm_pulse < sonar_pitch_servo_pwm_min) pwm_pulse = sonar_pitch_servo_pwm_min;
	return(pwm_pulse);
}

void sonarInit(void)
{
    sonar_pitch_servo_pwm_delta = sonar_pitch_offset_centred_pwm;
}

void sonarCntrl(void)
{
#if (USE_SONAR == 1)
	union longbbbb cam;
	int16_t sonar_pitch16 = 0;    // pitch accumalator in 16 bit byte circular

	if (flags._.pitch_feedback == 1 && canStabilizeHover() && current_orientation == F_HOVER)
	{
        //stabilize sonar pointing toward bottom vertical direction
		sonar_pitch16 = rmat[8];
		cam.WW = __builtin_mulss(sonar_pitch16, sonar_pitch_servo_high_ratio) + 0x8000;
		sonar_pitch_servo_pwm_delta = cam._.W1 - sonar_pitch_offset_centred_pwm;		
	}
	else
	{
		// set sonar to default position
		// Pitch Servo
		sonar_pitch_servo_pwm_delta = sonar_pitch_offset_pwm;

    }   
#endif
}
