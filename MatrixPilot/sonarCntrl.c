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
#include "../libUDB/heartbeat.h"
#include "../libSTM/dsp.h"

#define TILT_THROW_RATIO 0.5 * (TILT_THROW_RATIO1 + TILT_THROW_RATIO2)
#define TILT_PWM_EQ BETA_EQ_DEG * 2000.0 * TILT_THROW_RATIO / (TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG)
#define K_TILT (-2 * KQ * K1 / (T_EQ_A * R_A * SIN_ALPHA)) * (180.0 / PI) * 2000.0 * TILT_THROW_RATIO / (TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG)

#define TILT_PWM_MAX 1000 * TILT_THROW_RATIO
#define TILT_PWM_MIN -1000 * TILT_THROW_RATIO
#define TILT_PWM_TRANSITION_H (1000 * TILT_THROW_RATIO) / (TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG) * (2 * TRANSITION_ANGLE_DEG - TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG)
#define TILT_PWM_TRANSITION_L (1000 * TILT_THROW_RATIO) / (TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG) * (2 * (TRANSITION_ANGLE_DEG - TRANSITION_HYSTERESIS) - TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG)
#define TILT_PWM_FORWARD_FLIGHT (1000 * TILT_THROW_RATIO) / (TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG) * (2 * FORWARD_FLIGHT_ANGLE_DEG - TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG)
#define TILT_PWM_INCR ((TILT_ANG_VEL * 2000.0 * TILT_THROW_RATIO / (TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG)) * TILT_UPDATE_HZ) / HEARTBEAT_HZ


bool is_forward_flight = false;
int16_t motor_tilt_servo_pwm_target1 = 0;
int16_t motor_tilt_servo_pwm_target2 = 0;


int16_t updateTiltPwm(int16_t tilt_pwm, int16_t target_pwm, int16_t incr_pwm) {

    int16_t temp = tilt_pwm;

    if (udb_heartbeat_counter % (HEARTBEAT_HZ / TILT_UPDATE_HZ) == 0) {
	int16_t incr = (int16_t)(TILT_PWM_INCR);
	if (tilt_pwm >= (target_pwm + incr)) {
	    temp -= incr;
	}
	else {
	    if (tilt_pwm < target_pwm) {
		temp += incr;
	    }
	}
    }

    return temp;
}

int16_t motorTiltServoLimit(int16_t pwm_pulse)
{
    pwm_pulse = limit_value(pwm_pulse, TILT_PWM_MIN, TILT_PWM_MAX);
    return(pwm_pulse);
}

void motorTiltInit(void)
{
    motor_tilt_servo_pwm_delta1 = 0;
    motor_tilt_servo_pwm_delta2 = 0;
    motor_tilt_servo_pwm_target1 = 0;
    motor_tilt_servo_pwm_target2 = 0;
}

/*
 * front motor tilt angle used for yaw control.
 */
int16_t yawCntrlByTilt(void)
{
    if (motorsInHoveringPos()) {        
        int16_t yaw_corr_tilt_pwm = (int16_t)(TILT_PWM_EQ);
        return K_TILT * yaw_quad_control + yaw_corr_tilt_pwm;
    } 
    else {
        return 0.;
    }
}

void motorTiltCntrl(void)
{
    int32_t temp;
    int16_t pwManual[NUM_INPUTS+1];

    // If radio is off, use udb_pwTrim values instead of the udb_pwIn values
    for (temp = 0; temp <= NUM_INPUTS; temp++)
    {
	if (udb_flags._.radio_on)
	    pwManual[temp] = udb_pwIn[temp];
	else
	    pwManual[temp] = udb_pwTrim[temp];
    }

    temp = __builtin_mulsu((pwManual[INPUT_CHANNEL_AUX1] - 3000), TILT_THROW_RATIO1 * RMAX);
    motor_tilt_servo_pwm_target1 = (int16_t)(temp >> 14);
    temp = __builtin_mulsu((pwManual[INPUT_CHANNEL_AUX1] - 3000), TILT_THROW_RATIO2 * RMAX);
    motor_tilt_servo_pwm_target2 = (int16_t)(temp >> 14);
    
    if (!motorsInHoveringPos()) {
	motor_tilt_servo_pwm_target1 = (int16_t)(TILT_PWM_FORWARD_FLIGHT);
	motor_tilt_servo_pwm_target2 = (int16_t)(TILT_PWM_FORWARD_FLIGHT);
    }

    motor_tilt_servo_pwm_delta1 = updateTiltPwm(motor_tilt_servo_pwm_delta1, motor_tilt_servo_pwm_target1, (int16_t)(TILT_PWM_INCR));
    motor_tilt_servo_pwm_delta2 = updateTiltPwm(motor_tilt_servo_pwm_delta2, motor_tilt_servo_pwm_target2, (int16_t)(TILT_PWM_INCR));

}

boolean motorsInHoveringPos() {
    int16_t motor_tilt_servo_pwm_target = 0.5 * (motor_tilt_servo_pwm_target1 + motor_tilt_servo_pwm_target2);
    int16_t pwm_tilt_transition_l = REVERSE_IF_NEEDED(MOTOR_TILT_CHANNEL_REVERSED, TILT_PWM_TRANSITION_L);
    int16_t pwm_tilt_transition_h = REVERSE_IF_NEEDED(MOTOR_TILT_CHANNEL_REVERSED, TILT_PWM_TRANSITION_H);
    int16_t servo_pwm_delta_output = REVERSE_IF_NEEDED(MOTOR_TILT_CHANNEL_REVERSED, motor_tilt_servo_pwm_target);

    if (servo_pwm_delta_output >= pwm_tilt_transition_h) {
	is_forward_flight = true;
    }

    if (servo_pwm_delta_output < pwm_tilt_transition_l) {
	is_forward_flight = false;
    }
   
    return !is_forward_flight;
}
