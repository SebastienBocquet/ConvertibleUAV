// This file is part of the MatrixPilotQuad firmware.
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

#include "../libDCM/libDCM_internal.h"
#include "defines.h"
#include "../libUDB/heartbeat.h"

#define MANUAL_DEADBAND 200 // amount of throttle before fly-by-wire controls engage
#define MAXIMUM_ERROR_INTEGRAL ((int32_t) 32768000 )
#define YAW_DEADBAND 5 // prevent Tx pulse variation from causing yaw drift
#define TIME_MANUAL_TO_AUTO 3
#define INCREMENT_MANUAL_TO_AUTO RMAX / (HEARTBEAT_HZ * TIME_MANUAL_TO_AUTO)

extern int16_t theta[3] ;
extern void matrix_normalize ( int16_t[] ) ;
extern void MatrixRotate( int16_t[] , int16_t[] ) ;
extern int commanded_tilt_gain ;

int16_t throttlemin = (int16_t)(2.0*SERVORANGE*(1+HOVER_THROTTLE_MIN));
int16_t throttlemax = (int16_t)(2.0*SERVORANGE*(1+HOVER_THROTTLE_MAX));

int16_t desired_roll;
int16_t desired_pitch;
int16_t desired_yaw;
int16_t roll_quad_control ;
int16_t pitch_quad_control ;
int16_t yaw_quad_control ;
int16_t accel_feedback ;

int16_t pwManual[NUM_INPUTS+1] ;
int16_t commanded_roll ;
int16_t commanded_pitch ;
int16_t commanded_yaw ;

int16_t pitch_body_frame_control;
int16_t roll_body_frame_control;

int16_t earth_yaw;
int16_t earth_roll;
int16_t earth_pitch;

int16_t roll_error ;
int16_t pitch_error ;
int16_t yaw_error ;

int16_t roll_intgrl;
int16_t pitch_intgrl;
int16_t	yaw_intgrl;

//int16_t roll_rate_intgrl;
//int16_t pitch_rate_intgrl;
//int16_t	yaw_rate_intgrl;

int16_t roll_rate_error_delta_filt = 0;
int16_t pitch_rate_error_delta_filt = 0;
int16_t roll_rate_error_previous = 0 ;
int16_t pitch_rate_error_previous = 0 ;
float roll_rate_error_delta_filt_flt = 0.;
float pitch_rate_error_delta_filt_flt = 0.;

int16_t manual_to_auto_ramp = 0;

union longww roll_quad_error_integral = { 0 } ;
union longww pitch_quad_error_integral = { 0 } ;
union longww yaw_quad_error_integral = { 0 } ;
//union longww roll_rate_quad_error_integral = { 0 } ;
//union longww pitch_rate_quad_error_integral = { 0 } ;
//union longww yaw_rate_quad_error_integral = { 0 } ;

int16_t target_orientation[9] = { RMAX , 0 , 0 , 0 , RMAX , 0 , 0 , 0 , RMAX } ;

const int16_t yaw_command_gain = ((long) MAX_YAW_RATE )*(0.03) ;

int16_t throttle1 = 0;
int16_t throttle2 = 0;
int16_t throttle3 = 0;
int16_t throttle4 = 0;
int16_t mean_throttle = 0;

void reset_target_orientation(void)
{
    target_orientation[0] = RMAX;
    target_orientation[1] = 0;
    target_orientation[2] = 0;
    target_orientation[3] = 0;
    target_orientation[4] = RMAX;
    target_orientation[5] = 0;
    target_orientation[6] = 0;
    target_orientation[7] = 0;
    target_orientation[8] = RMAX;
}

void reset_derivative_terms(void)
{
    roll_rate_error_previous = 0;
    roll_rate_error_delta_filt_flt = 0;
    pitch_rate_error_previous = 0;
    pitch_rate_error_delta_filt_flt = 0;
}

void reset_integral_terms(void)
{
    roll_quad_error_integral.WW  = 0;
    pitch_quad_error_integral.WW  = 0;
    yaw_quad_error_integral.WW  = 0;
}

void motorCntrl(const uint16_t tilt_kp,const uint16_t tilt_ki, const uint16_t tilt_rate_kp, const uint16_t tilt_rate_kd, const uint16_t yaw_ki, const uint16_t yaw_kp, const uint16_t yaw_rate_kp)
{
    int16_t temp ;

    int16_t motor_A ;
    int16_t motor_B ;
    int16_t motor_C ;
    int16_t motor_D ;

    int16_t commanded_roll_body_frame ;
    int16_t commanded_pitch_body_frame ;

    int16_t commanded_tilt[3] ;

    int16_t roll_rate;
    int16_t pitch_rate;
    int16_t yaw_rate;

    int16_t roll_rate_error;
    int16_t pitch_rate_error;
    int16_t yaw_rate_error;

    int16_t roll_rate_error_delta = 0;
    int16_t pitch_rate_error_delta = 0;

    union longww long_accum ;

    int16_t yaw_step ;
    int16_t yaw_vector[3] ;
    struct relative2D matrix_accum  = { 0, 0 };     // Temporary variable to keep intermediate results of functions.
    int16_t target_orientation_transposed[9] ;
    int16_t orientation_error_matrix[9] ;

    // If radio is off, use udb_pwTrim values instead of the udb_pwIn values
    for (temp = 0; temp <= NUM_INPUTS; temp++)
	if (udb_flags._.radio_on)
	    pwManual[temp] = udb_pwIn[temp];
	else
	    pwManual[temp] = udb_pwTrim[temp];

    int16_t yaw_control_weight_reverse = 0;
    int16_t yaw_control_weight = compute_tx_linear_control(pwManual[INPUT_CHANNEL_AUX2], 0, RMAX, yaw_control_weight_reverse);

    if (!dcm_flags._.calib_finished)
    {
	// Leave at 0 (no PWM pulses) until calibrated.
	udb_pwOut[MOTOR_A_OUTPUT_CHANNEL] = 0 ;
	udb_pwOut[MOTOR_B_OUTPUT_CHANNEL] = 0 ;
	udb_pwOut[MOTOR_C_OUTPUT_CHANNEL] = 0 ;
	udb_pwOut[MOTOR_D_OUTPUT_CHANNEL] = 0 ;
    }
    else if (!dcm_flags._.yaw_init_finished)
    {
	target_orientation[0] = rmat[0] ;
	target_orientation[1] = rmat[1] ;
	target_orientation[2] = rmat[2] ;
	target_orientation[3] = rmat[3] ;
	target_orientation[4] = rmat[4] ;
	target_orientation[5] = rmat[5] ;
    }
    else
    {
	//insert yawCorr, pitchCorr and roll_nav_corr to control gps navigation in quad mode
	commanded_roll =  ( pwManual[AILERON_INPUT_CHANNEL] 
		- udb_pwTrim[AILERON_INPUT_CHANNEL])*commanded_tilt_gain ;
	commanded_pitch = ( pwManual[ELEVATOR_INPUT_CHANNEL] 
		- udb_pwTrim[ELEVATOR_INPUT_CHANNEL] )*commanded_tilt_gain  ;
	commanded_yaw = ( pwManual[RUDDER_INPUT_CHANNEL] 
		- udb_pwTrim[RUDDER_INPUT_CHANNEL] )  ;

	if ( commanded_yaw >= YAW_DEADBAND )
	{
	    commanded_yaw -= YAW_DEADBAND ;
	}
	else if ( commanded_yaw <= - YAW_DEADBAND )
	{
	    commanded_yaw += YAW_DEADBAND ;
	}
	else
	{
	    commanded_yaw = 0 ;
	}

	//		adjust roll and pitch commands to prevent combined tilt from exceeding 90 degrees
	commanded_tilt[0] = commanded_roll ;
	commanded_tilt[1] = commanded_pitch ;
	commanded_tilt[2] = RMAX ;
	vector3_normalize( commanded_tilt , commanded_tilt ) ;
	commanded_roll = commanded_tilt[0] ;
	commanded_pitch = commanded_tilt[1] ;

	commanded_pitch_body_frame = commanded_pitch ;
	commanded_roll_body_frame = commanded_roll ;

	//		Compute orientation errors

	//		Compute the orientation of the virtual quad (which is used only for yaw control)
	//		Set the earth vertical to match in both frames (since we are interested only in yaw)

	target_orientation[6] = rmat[6] ;
	target_orientation[7] = rmat[7] ;
	target_orientation[8] = rmat[8] ;

	//		renormalize to align other two axes int16_to the the plane perpendicular to the vertical
	matrix_normalize( target_orientation ) ;

	//		Rotate the virtual quad around the earth vertical axis according to the commanded yaw rate
	yaw_step = commanded_yaw * yaw_command_gain ;
	VectorScale( 3 , yaw_vector , &target_orientation[6] , yaw_step ) ;
	VectorAdd( 3, yaw_vector , yaw_vector , yaw_vector ) ; // doubles the vector
	MatrixRotate( target_orientation , yaw_vector ) ;

	//		Compute the misalignment between target and actual
	MatrixTranspose( 3 , 3 , target_orientation_transposed , target_orientation )	;
	MatrixMultiply ( 3 , 3 , 3 , orientation_error_matrix , target_orientation_transposed , rmat ) ;

	//		Compute orientation errors
	yaw_error = ( orientation_error_matrix[1] - orientation_error_matrix[3] )/2 ;
    // weight yaw error according to Tx order
    yaw_error = int_scale(yaw_error, yaw_control_weight);
    //yaw_error = yaw_control_weight;

	roll_error = rmat[6] - (-commanded_roll_body_frame + roll_control*commanded_tilt_gain) ;
	pitch_error = -rmat[7] - (-commanded_pitch_body_frame + pitch_control*commanded_tilt_gain) ;

	//		Compute the signals that are common to all 4 motors
	long_accum.WW = __builtin_mulus ( (uint16_t) (RMAX*ACCEL_K ) , accelEarth[2] ) ;
	accel_feedback = long_accum._.W1 ;



	//		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Compute the error integrals%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	if ( (flags._.integral_pid_term) && (current_orientation == F_HOVER) )
	{
	    roll_quad_error_integral.WW += ((__builtin_mulus ( (uint16_t) (32.0*tilt_ki/40.), roll_error ))>>5) ;
	    if ( roll_quad_error_integral.WW > MAXIMUM_ERROR_INTEGRAL )
	    {
		roll_quad_error_integral.WW = MAXIMUM_ERROR_INTEGRAL ;
	    }
	    if ( roll_quad_error_integral.WW < - MAXIMUM_ERROR_INTEGRAL )
	    {
		roll_quad_error_integral.WW =  - MAXIMUM_ERROR_INTEGRAL ;
	    }

	    pitch_quad_error_integral.WW += ((__builtin_mulus ( (uint16_t) (32.0*tilt_ki/40.), pitch_error ))>>5) ;
	    if ( pitch_quad_error_integral.WW > MAXIMUM_ERROR_INTEGRAL )
	    {
		pitch_quad_error_integral.WW = MAXIMUM_ERROR_INTEGRAL ;
	    }
	    if ( pitch_quad_error_integral.WW < - MAXIMUM_ERROR_INTEGRAL )
	    {
		pitch_quad_error_integral.WW =  - MAXIMUM_ERROR_INTEGRAL ;
	    }

	    yaw_quad_error_integral.WW += ((__builtin_mulus ( (uint16_t) (32.0*yaw_ki/40.), yaw_error ))>>5) ;
	    if ( yaw_quad_error_integral.WW > MAXIMUM_ERROR_INTEGRAL )
	    {
		yaw_quad_error_integral.WW = MAXIMUM_ERROR_INTEGRAL ;
	    }
	    if ( yaw_quad_error_integral.WW < - MAXIMUM_ERROR_INTEGRAL )
	    {
		yaw_quad_error_integral.WW =  - MAXIMUM_ERROR_INTEGRAL ;
	    }
	}
	else
	{
	    roll_quad_error_integral.WW  = 0;
	    pitch_quad_error_integral.WW  = 0;
	    yaw_quad_error_integral.WW  = 0;
	}
	//		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%End Compute the error integrals%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


	//		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%roll stabilization%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	//		Compute the PID signals on roll_error

	long_accum.WW = __builtin_mulus ( tilt_kp , roll_error ) << 2  ;
	desired_roll = -long_accum._.W1 ;

	roll_intgrl = limit_value(roll_quad_error_integral._.W1 << 2, -(int16_t)(TILT_ERROR_INTEGRAL_LIMIT), (int16_t)(TILT_ERROR_INTEGRAL_LIMIT)); 

	desired_roll -= roll_intgrl;

	//		compute error between angle_rate and first PID output
	roll_rate = -omegaAccum[1];
	roll_rate_error = roll_rate - desired_roll;

	//		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Compute the derivatives%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	roll_rate_error_delta = roll_rate_error - roll_rate_error_previous;
	roll_rate_error_previous = roll_rate_error ;
	roll_rate_error_delta_filt = exponential_filter(roll_rate_error_delta, &roll_rate_error_delta_filt_flt, (float)(TILT_RATE_DELTA_FILTER));

	//		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%End Compute the derivatives%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	//      compute PID on omega_error
	long_accum.WW = __builtin_mulus ( tilt_rate_kp , roll_rate_error ) << 2 ;
	roll_quad_control = -long_accum._.W1 ;
	long_accum.WW = __builtin_mulus ( tilt_rate_kd , roll_rate_error_delta_filt ) << 2 ;
	roll_quad_control -= long_accum._.W1 ;

	//		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%End roll stabilization%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


	//		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%pitch stabilization%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	//		Compute the PID signals on pitch_error
	//		pitch_error is -rmat7, with rmat7 the pitch angle
	long_accum.WW = __builtin_mulus ( tilt_kp , pitch_error ) << 2  ;
	desired_pitch = -long_accum._.W1 ;

	pitch_intgrl = limit_value(pitch_quad_error_integral._.W1 << 2, -(int16_t)(TILT_ERROR_INTEGRAL_LIMIT), (int16_t)(TILT_ERROR_INTEGRAL_LIMIT));

	desired_pitch -= pitch_intgrl;

	pitch_rate = -omegagyro[0];
	//exponential_filter(pitch_rate, &pitch_rate_filtered_flt, (float)(80), (int16_t)(HEARTBEAT_HZ));		
	pitch_rate_error = pitch_rate - desired_pitch;

	//		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Compute the derivatives%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	pitch_rate_error_delta = pitch_rate_error - pitch_rate_error_previous ;
	pitch_rate_error_previous = pitch_rate_error ;
	pitch_rate_error_delta_filt = exponential_filter(pitch_rate_error_delta, &pitch_rate_error_delta_filt_flt, (float)(TILT_RATE_DELTA_FILTER));

	//		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%End Compute the derivatives%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	//      compute PID on omega_error
	long_accum.WW = __builtin_mulus ( tilt_rate_kp , pitch_rate_error ) << 2 ;
	pitch_quad_control = -long_accum._.W1 ;
	long_accum.WW = __builtin_mulus ( tilt_rate_kd , pitch_rate_error_delta_filt ) << 2 ;
	pitch_quad_control -= long_accum._.W1 ;


	//		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%End pitch stabilization%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


	//		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%yaw stabilization%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	long_accum.WW = __builtin_mulus ( yaw_kp , yaw_error ) << 2  ;
	desired_yaw = -long_accum._.W1 ;

	yaw_intgrl = limit_value(yaw_quad_error_integral._.W1 << 2, -16384, 16384);

	desired_yaw -= yaw_intgrl ;

	//		compute error between angle_rate and first PID output
	//		use minus omegagyro to be coherent with yaw_error
	yaw_rate = -omegagyro[2];
	yaw_rate_error = yaw_rate - desired_yaw;

	//      compute PID on omega_error
	long_accum.WW = __builtin_mulus ( yaw_rate_kp , yaw_rate_error ) << 2 ;
	yaw_quad_control = -long_accum._.W1 ;

	//		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%End yaw stabilization%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#ifdef CONFIG_PLUS

	pitch_body_frame_control = pitch_quad_control ;
	roll_body_frame_control = roll_quad_control ;

#endif

#ifdef CONFIG_X

	pitch_body_frame_control = 3*(( pitch_quad_control - roll_quad_control )/4) ;
	roll_body_frame_control = 3*(( pitch_quad_control + roll_quad_control )/4) ;

#endif

	//		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%motor output%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	if (!(current_orientation == F_HOVER))
	{
	    motor_A = motor_B = motor_C = motor_D = pwManual[THROTTLE_INPUT_CHANNEL];
	}
	else
	{
	    motor_A = motor_B = motor_C = motor_D = pwManual[THROTTLE_HOVER_INPUT_CHANNEL];

	    if((udb_servo_pulsesat(pwManual[THROTTLE_HOVER_INPUT_CHANNEL]) - udb_servo_pulsesat(udb_pwTrim[THROTTLE_HOVER_INPUT_CHANNEL])) > HOVER_THROTTLE_MIN*(2.0*SERVORANGE))
	    {
		//apply roll, pitch, yaw stabilization
#if (MOTOR_A_POSITION == 1)
		//	Mix in the yaw, pitch, and roll signals to the motors
		motor_A += +yaw_quad_control - pitch_body_frame_control ;
		motor_B += -yaw_quad_control - roll_body_frame_control ;
		motor_C += +yaw_quad_control + pitch_body_frame_control ;
		motor_D += -yaw_quad_control + roll_body_frame_control ;
#elif (MOTOR_A_POSITION == 3)
		//	Mix in the yaw, pitch, and roll signals to the motors
		motor_A += +yaw_quad_control + pitch_body_frame_control ;
		motor_B += -yaw_quad_control - roll_body_frame_control ;
		motor_C += +yaw_quad_control - pitch_body_frame_control ;
		motor_D += -yaw_quad_control + roll_body_frame_control ;
#endif
		//limit max throttle of each engine
		motor_A = limit_value(motor_A, throttlemin, throttlemax);
		motor_B = limit_value(motor_B, throttlemin, throttlemax);
		motor_C = limit_value(motor_C, throttlemin, throttlemax);
		motor_D = limit_value(motor_D, throttlemin, throttlemax);
	    }  
	}

	//		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%end motor output%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	throttle1 = motor_A;
	throttle2 = motor_B;
	throttle3 = motor_C;
	throttle4 = motor_D;
	mean_throttle = (motor_A + motor_B + motor_C + motor_D) >> 2;

	udb_pwOut[MOTOR_A_OUTPUT_CHANNEL] = udb_servo_pulsesat( motor_A ) ;		
	udb_pwOut[MOTOR_B_OUTPUT_CHANNEL] = udb_servo_pulsesat( motor_B ) ;
	udb_pwOut[MOTOR_C_OUTPUT_CHANNEL] = udb_servo_pulsesat( motor_C ) ;
	udb_pwOut[MOTOR_D_OUTPUT_CHANNEL] = udb_servo_pulsesat( motor_D ) ;
    }
    /* while(1){} */
}

#ifndef MOTOR_A_POSITION
#error ("You have not defined the position of motor A")
#endif

#ifndef CONFIG_PLUS
#ifndef CONFIG_X
#error ("You have not selected a configuration in options.h, select either CONFIG_PLUS or CONFIG_X.")
#endif
#endif

#ifdef CONFIG_PLUS
#ifdef CONFIG_X
#error ("You have selected both CONFIG_PLUS and CONFIG_X in options.h. Select just one of them."
#endif
#endif


#if  (( ( int16_t ) + MAX_YAW_RATE   < 50 ) || ( ( int16_t ) + MAX_YAW_RATE > 500 ))
#error ("MAX_YAW_RATE must be between 50.0 and 500.0 degrees/second.")
#endif

#if (((int16_t) + MAX_TILT) > 45)
#error ("MAX_TILT mus be less than or equal to 45 degrees."
#endif
