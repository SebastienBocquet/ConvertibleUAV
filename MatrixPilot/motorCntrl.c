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

extern int16_t theta[3] ;
extern void matrix_normalize ( int16_t[] ) ;
extern void MatrixRotate( int16_t[] , int16_t[] ) ;
extern int commanded_tilt_gain ;

int16_t desired_roll;
int16_t desired_pitch;
int16_t desired_yaw;
int16_t roll_quad_control ;
int16_t pitch_quad_control ;
int16_t yaw_quad_control ;
int16_t accel_feedback ;
int16_t theta_previous[2] = { 0 , 0 } ;
int16_t theta_delta[2] ;

int16_t pwManual[NUM_INPUTS+1] ;
int16_t commanded_roll ;
int16_t commanded_pitch ;
int16_t commanded_yaw ;

int16_t pitch_body_frame_control;
int16_t roll_body_frame_control;

int16_t roll_error ;
int16_t pitch_error ;
int16_t yaw_error ;

int16_t roll_error_previous = 0 ;
int16_t pitch_error_previous = 0 ;
int16_t yaw_error_previous = 0 ;

union longww roll_quad_error_integral = { 0 } ;
union longww pitch_quad_error_integral = { 0 } ;
union longww yaw_quad_error_integral = { 0 } ;

int16_t target_orientation[9] = { RMAX , 0 , 0 , 0 , RMAX , 0 , 0 , 0 , RMAX } ;

const int16_t yaw_command_gain = ((long) MAX_YAW_RATE )*(0.03) ;

void motorCntrl(void)
{
	int16_t temp ;
	
	int16_t min_throttle ;
	
	int16_t motor_A ;
	int16_t motor_B ;
	int16_t motor_C ;
	int16_t motor_D ;

	int16_t commanded_roll_body_frame ;
	int16_t commanded_pitch_body_frame ;

	int16_t commanded_tilt[3] ;

	int16_t roll_error_delta ;
	int16_t pitch_error_delta ;
	int16_t yaw_error_delta ;

	int16_t roll_rate;
	int16_t pitch_rate;
	int16_t yaw_rate;

	int16_t roll_rate_error;
	int16_t pitch_rate_error;
	int16_t yaw_rate_error;

	union longww long_accum ;
//	union longww accum ; // debugging temporary

	int16_t yaw_step ;
	int16_t yaw_vector[3] ;
	int16_t target_orientation_transposed[9] ;
	int16_t orientation_error_matrix[9] ;
	
	// If radio is off, use udb_pwTrim values instead of the udb_pwIn values
	for (temp = 0; temp <= NUM_INPUTS; temp++)
		if (udb_flags._.radio_on)
			pwManual[temp] = udb_pwIn[temp];
		else
			pwManual[temp] = udb_pwTrim[temp];
	
	
	if (!dcm_flags._.calib_finished)
	{
		// Leave at 0 (no PWM pulses) until calibrated.
		udb_pwOut[MOTOR_A_OUTPUT_CHANNEL] = 0 ;
		udb_pwOut[MOTOR_B_OUTPUT_CHANNEL] = 0 ;
		udb_pwOut[MOTOR_C_OUTPUT_CHANNEL] = 0 ;
		udb_pwOut[MOTOR_D_OUTPUT_CHANNEL] = 0 ;
	}
	else if (abs(pwManual[THROTTLE_HOVER_INPUT_CHANNEL]-udb_pwTrim[THROTTLE_HOVER_INPUT_CHANNEL])< MANUAL_DEADBAND )
	{

		motor_A = motor_B = motor_C = motor_D = pwManual[THROTTLE_HOVER_INPUT_CHANNEL] + REVERSE_IF_NEEDED(THROTTLE_HOVER_CHANNEL_REVERSED, throttle_hover_control); ;

		VectorCopy ( 9 , target_orientation , rmat ) ;

		//insert yawCorr, pitchCorr and roll_nav_corr to control gps navigation in quad mode
		commanded_roll =  ( pwManual[AILERON_INPUT_CHANNEL] 
						- udb_pwTrim[AILERON_INPUT_CHANNEL]) ;
		commanded_pitch = ( pwManual[ELEVATOR_INPUT_CHANNEL] 
						- udb_pwTrim[ELEVATOR_INPUT_CHANNEL] ) ;
		commanded_yaw = ( pwManual[RUDDER_INPUT_CHANNEL] 
						- udb_pwTrim[RUDDER_INPUT_CHANNEL] )  ;
#ifdef CONFIG_PLUS
		commanded_pitch_body_frame = commanded_pitch ;
		commanded_roll_body_frame = commanded_roll ;
#endif

#ifdef CONFIG_X
		commanded_pitch_body_frame =  3*(( commanded_pitch - commanded_roll )/4) ; // approximation to .707, not critcal
		commanded_roll_body_frame = 3*(( commanded_pitch + commanded_roll )/4) ; 
#endif

		motor_A += + commanded_yaw - commanded_pitch_body_frame ;
		motor_B += - commanded_yaw - commanded_roll_body_frame ;
		motor_C += + commanded_yaw + commanded_pitch_body_frame ;
		motor_D += - commanded_yaw + commanded_roll_body_frame ;

		udb_pwOut[MOTOR_A_OUTPUT_CHANNEL] = udb_servo_pulsesat( motor_A ) ;		
		udb_pwOut[MOTOR_B_OUTPUT_CHANNEL] = udb_servo_pulsesat( motor_B ) ;
		udb_pwOut[MOTOR_C_OUTPUT_CHANNEL] = udb_servo_pulsesat( motor_C ) ;
		udb_pwOut[MOTOR_D_OUTPUT_CHANNEL] = udb_servo_pulsesat( motor_D ) ;

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

//#ifdef CONFIG_PLUS

		commanded_pitch_body_frame = commanded_pitch ;
		commanded_roll_body_frame = commanded_roll ;
//
//#endif
//
//#ifdef CONFIG_X
//
//		commanded_pitch_body_frame =  3*(( commanded_pitch - commanded_roll )/4) ; // approximation to .707, not critcal
//		commanded_roll_body_frame = 3*(( commanded_pitch + commanded_roll )/4) ; 
//
//#endif



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

//		Compute the misalignment between target and actaul
		MatrixTranspose( 3 , 3 , target_orientation_transposed , target_orientation )	;
		MatrixMultiply ( 3 , 3 , 3 , orientation_error_matrix , target_orientation_transposed , rmat ) ;

//		Compute orientation errors
		roll_error = commanded_roll_body_frame + rmat[6] ;
		pitch_error = commanded_pitch_body_frame - rmat[7] ;
		yaw_error = ( orientation_error_matrix[1] - orientation_error_matrix[3] )/2 ;

//		Compute the signals that are common to all 4 motors
		min_throttle = udb_pwTrim[THROTTLE_HOVER_INPUT_CHANNEL] ;
		long_accum.WW = __builtin_mulus ( (uint16_t) (RMAX*ACCEL_K ) , accelEarth[2] ) ;
		accel_feedback = long_accum._.W1 ;
		motor_A = motor_B = motor_C = motor_D = pwManual[THROTTLE_HOVER_INPUT_CHANNEL] - accel_feedback ;


//		Compute the error integrals
		roll_quad_error_integral.WW += ((__builtin_mulus ( (uint16_t) (32.0*RMAX*TILT_KI/40.), roll_error ))>>5) ;
		if ( roll_quad_error_integral.WW > MAXIMUM_ERROR_INTEGRAL )
		{
			roll_quad_error_integral.WW = MAXIMUM_ERROR_INTEGRAL ;
		}
		if ( roll_quad_error_integral.WW < - MAXIMUM_ERROR_INTEGRAL )
		{
			roll_quad_error_integral.WW =  - MAXIMUM_ERROR_INTEGRAL ;
		}

		pitch_quad_error_integral.WW += ((__builtin_mulus ( (uint16_t) (32.0*RMAX*TILT_KI/40.), pitch_error ))>>5) ;
		if ( pitch_quad_error_integral.WW > MAXIMUM_ERROR_INTEGRAL )
		{
			pitch_quad_error_integral.WW = MAXIMUM_ERROR_INTEGRAL ;
		}
		if ( pitch_quad_error_integral.WW < - MAXIMUM_ERROR_INTEGRAL )
		{
			pitch_quad_error_integral.WW =  - MAXIMUM_ERROR_INTEGRAL ;
		}

		yaw_quad_error_integral.WW += ((__builtin_mulus ( (uint16_t) (32.0*RMAX*YAW_KI/40.), yaw_error ))>>5) ;
		if ( yaw_quad_error_integral.WW > MAXIMUM_ERROR_INTEGRAL )
		{
			yaw_quad_error_integral.WW = MAXIMUM_ERROR_INTEGRAL ;
		}
		if ( yaw_quad_error_integral.WW < - MAXIMUM_ERROR_INTEGRAL )
		{
			yaw_quad_error_integral.WW =  - MAXIMUM_ERROR_INTEGRAL ;
		}

//		Compute the derivatives
// theta_delta seems to be zero all the time
		theta_delta[0] = theta[0] - theta_previous[0] ;
		theta_delta[1] = theta[1] - theta_previous[1] ;

		theta_previous[0] = theta[0] ;
		theta_previous[1] = theta[1] ;

		//debugger and cropper la derivee
		roll_error_delta = roll_error - roll_error_previous;
		roll_error_previous = roll_error ;

		pitch_error_delta = pitch_error - pitch_error_previous ;
		pitch_error_previous = pitch_error ;

		yaw_error_delta = yaw_error - yaw_error_previous ;
		yaw_error_previous = yaw_error ;


//		roll stabilization

//		Compute the PID(DD) signals on roll_error
		//long_accum.WW = __builtin_mulus ( (uint16_t) (RMAX*TILT_KP) , roll_error ) << 2  ;
		long_accum.WW = __builtin_mulus ( (uint16_t) (HEARTBEAT_HZ*TILT_KP) , roll_error - roll_error_previous )  ;
		if ( long_accum.WW > RMAX )
		{
			long_accum.WW = RMAX ;
		}
		if ( long_accum.WW < - RMAX )
		{
			long_accum.WW =  - RMAX ;
		}
		desired_roll = -(int16_t)long_accum.WW;

		long_accum.WW = __builtin_mulus ( (uint16_t) (RMAX*TILT_KD) , roll_error_delta ) << 2  ;
		desired_roll -= long_accum._.W1 ;

		long_accum.WW = __builtin_mulus ( (uint16_t) (RMAX*TILT_KDD) , -theta_delta[1] ) << 2 ;
		desired_roll -= long_accum._.W1 ;

		desired_roll -= roll_quad_error_integral._.W1 << 2  ;

//		compute error between angle_rate and first PID output
		roll_rate = -omegaAccum[1];
		roll_rate_error = roll_rate - desired_roll;

//      compute PID (only P for the moment) on omega_error
		long_accum.WW = __builtin_mulus ( (uint16_t) (RMAX*TILT_RATE_KP) , roll_rate_error ) << 2 ;
		roll_quad_control = -long_accum._.W1 ;

		
//		pitch stabilization

//		Compute the PID(DD) signals on pitch_error
//		pitch_error is -rmat7, with rmat7 the pitch angle
		long_accum.WW = __builtin_mulus ( (uint16_t) (RMAX*TILT_KP) , pitch_error ) << 2  ;
		desired_pitch = -long_accum._.W1 ;

		long_accum.WW = __builtin_mulus ( (uint16_t) (RMAX*TILT_KD) , pitch_error_delta ) << 2  ;
		desired_pitch -= long_accum._.W1 ;

		long_accum.WW = __builtin_mulus ( (uint16_t) (RMAX*TILT_KDD) , -theta_delta[0] ) << 2 ;
		desired_pitch -= long_accum._.W1 ;

		desired_pitch -= pitch_quad_error_integral._.W1 << 2  ;

//		compute error between angle_rate and first PID output
//		to be coherent with the definition of pitch_error, pitch_rate is set as minus the pitch rate given by omega_gyro
		long_accum.WW = (__builtin_mulss(rmat[8] , omegagyro[0])
	               - __builtin_mulss(rmat[6] , omegagyro[2])) << 1;
		pitch_rate = -long_accum._.W1;
		pitch_rate_error = pitch_rate - desired_pitch;

//      compute PID (only P for the moment) on omega_error
		long_accum.WW = __builtin_mulus ( (uint16_t) (RMAX*TILT_RATE_KP) , pitch_rate_error ) << 2 ;
		pitch_quad_control = -long_accum._.W1 ;


//		yaw stabilization

		long_accum.WW = __builtin_mulus ( (uint16_t) (RMAX*YAW_KP) , yaw_error ) << 2  ;
		desired_yaw = -long_accum._.W1 ;

		long_accum.WW = __builtin_mulus ( (uint16_t) (RMAX*YAW_KD) , yaw_error_delta ) << 2  ;
		desired_yaw -= long_accum._.W1 ;

		desired_yaw -= yaw_quad_error_integral._.W1 << 2  ;

//		compute error between angle_rate and first PID output
//		use minus omegagyro to be coherent with yaw_error
		yaw_rate = -omegagyro[2];
		yaw_rate_error = yaw_rate - desired_yaw;

		//      compute PID (only P for the moment) on omega_error
		long_accum.WW = __builtin_mulus ( (uint16_t) (RMAX*YAW_RATE_KP) , yaw_rate_error ) << 2 ;
		yaw_quad_control = -long_accum._.W1 ;
		

		//output telemetry
		additional_int16_export1 = roll_error;
		additional_int16_export2 = pitch_error;
		additional_int16_export3 = yaw_error;

		additional_int16_export4 = roll_rate;
		additional_int16_export5 = pitch_rate;
		additional_int16_export6 = yaw_rate;

		additional_int16_export7 = roll_quad_error_integral._.W1;
		additional_int16_export8 = pitch_quad_error_integral._.W1;
		additional_int16_export9 = yaw_quad_error_integral._.W1;


#ifdef CONFIG_PLUS

		pitch_body_frame_control = pitch_quad_control ;
		roll_body_frame_control = roll_quad_control ;

#endif

#ifdef CONFIG_X

		pitch_body_frame_control = 3*(( pitch_quad_control - roll_quad_control )/4) ;
		roll_body_frame_control = 3*(( pitch_quad_control + roll_quad_control )/4) ;

#endif


#if (MOTOR_A_POSITION == 1)
//		Mix in the yaw, pitch, and roll signals int16_to the motors
		motor_A += +yaw_quad_control - pitch_body_frame_control ;
		motor_B += -yaw_quad_control - roll_body_frame_control ;
		motor_C += +yaw_quad_control + pitch_body_frame_control ;
		motor_D += -yaw_quad_control + roll_body_frame_control ;
#elif (MOTOR_A_POSITION == 3)
//		Mix in the yaw, pitch, and roll signals int16_to the motors
		motor_A += +yaw_quad_control + pitch_body_frame_control ;
		motor_B += -yaw_quad_control + roll_body_frame_control ;
		motor_C += +yaw_quad_control - pitch_body_frame_control ;
		motor_D += -yaw_quad_control - roll_body_frame_control ;
#endif
//		Send the signals out to the motors
		udb_pwOut[MOTOR_A_OUTPUT_CHANNEL] = udb_servo_pulsesat( motor_A ) ;		
		udb_pwOut[MOTOR_B_OUTPUT_CHANNEL] = udb_servo_pulsesat( motor_B ) ;
		udb_pwOut[MOTOR_C_OUTPUT_CHANNEL] = udb_servo_pulsesat( motor_C ) ;
		udb_pwOut[MOTOR_D_OUTPUT_CHANNEL] = udb_servo_pulsesat( motor_D ) ;

	}
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