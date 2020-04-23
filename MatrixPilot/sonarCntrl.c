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
#include "../libSTM/dsp.h"

#define TILT_PWM_EQ BETA_EQ_DEG * 2000.0 * TILT_THROW_RATIO / (TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG)
#define K_TILT (-2 * KQ * K1 / (T_EQ_A * R_A * SIN_ALPHA)) * (180.0 / PI) * 2000.0 * TILT_THROW_RATIO / (TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG)

#define TILT_PWM_MAX 1000 * TILT_THROW_RATIO
#define TILT_PWM_MIN -1000 * TILT_THROW_RATIO
#define TILT_PWM_TRANSITON (1000 * TILT_THROW_RATIO) / (TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG) * (2 * TRANSITION_ANGLE_DEG - TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG)

int16_t motorTiltServoLimit(int16_t pwm_pulse)
{
    pwm_pulse = limit_value(pwm_pulse, TILT_PWM_MIN, TILT_PWM_MAX);
    return(pwm_pulse);
}

void motorTiltInit(void)
{
    motor_tilt_servo_pwm_delta = 0;
}

/*
 * front motor tilt angle used for yaw control.
 */
int16_t yawCntrlByTilt(void)
{
    int32_t temp;
    int16_t motor_tilt_pwm;
    /* temp = __builtin_mulsu(yaw_quad_control, K_TILT); */
    /* motor_tilt_pwm = (int16_t)(temp / RMAX); */
    return K_TILT * yaw_quad_control + (int16_t)(TILT_PWM_EQ);
}

void motorTiltCntrl(void)
{
    int32_t temp;
    int16_t servo_pwm;
    int16_t pwManual[NUM_INPUTS+1];

    // If radio is off, use udb_pwTrim values instead of the udb_pwIn values
    for (temp = 0; temp <= NUM_INPUTS; temp++)
    {
	if (udb_flags._.radio_on)
	    pwManual[temp] = udb_pwIn[temp];
	else
	    pwManual[temp] = udb_pwTrim[temp];
    }

    temp = __builtin_mulsu((pwManual[INPUT_CHANNEL_AUX1] - 3000), TILT_THROW_RATIO*RMAX);
    motor_tilt_servo_pwm_delta = (int16_t)(temp / RMAX) ;
}

boolean motorsInHoveringPos()
{
    return (REVERSE_IF_NEEDED(MOTOR_TILT_CHANNEL_REVERSED,
		motor_tilt_servo_pwm_delta) > TILT_PWM_TRANSITON);
}
