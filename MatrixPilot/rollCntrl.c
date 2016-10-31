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
#include "mode_switch.h"
#include "../libUDB/heartbeat.h"

#if (USE_CONFIGFILE == 1)
#include "config.h"
#include "redef.h"

	uint16_t yawkdail;
	uint16_t rollkp;
	uint16_t rollkd;

#elif ((SERIAL_OUTPUT_FORMAT == SERIAL_MAVLINK) || (GAINS_VARIABLE == 1))
	uint16_t yawkdail			= (uint16_t)(YAWKD_AILERON*SCALEGYRO*RMAX);
	uint16_t rollkp				= (uint16_t)(ROLLKP*RMAX);
	uint16_t rollkd				= (uint16_t)(ROLLKD*SCALEGYRO*RMAX);
#else 
	const uint16_t yawkdail		= (uint16_t)(YAWKD_AILERON*SCALEGYRO*RMAX);
	const uint16_t rollkp		= (uint16_t)(ROLLKP*RMAX);
	const uint16_t rollkd		= (uint16_t)(ROLLKD*SCALEGYRO*RMAX);
#endif	

#if (USE_CONFIGFILE == 1)
	uint16_t hoverrollkp;
	uint16_t hoverrollkd;
#elif ((SERIAL_OUTPUT_FORMAT == SERIAL_MAVLINK) || (GAINS_VARIABLE == 1))
	uint16_t hoverrollkp		= (uint16_t)(HOVER_ROLLKP*RMAX);
	uint16_t hoverrollkd		= (uint16_t)(HOVER_ROLLKD*SCALEGYRO*RMAX);
    fractional hoverrolloffset = HOVER_ROLL_OFFSET*RMAX;
	uint16_t hoverrollnavkp = (uint16_t)(HOVER_ROLLNAVKP*RMAX);
	uint16_t hoverrollnavki = (uint16_t)(HOVER_ROLLNAVKI*RMAX);
	int32_t limitintegralrollNav = (int32_t)(LIMIT_INTEGRAL_ROLLNAV);
#else
	const uint16_t hoverrollkp	= (uint16_t)(HOVER_ROLLKP*RMAX);
	const uint16_t hoverrollkd	= (uint16_t)(HOVER_ROLLKD*SCALEGYRO*RMAX);
    const fractional hoverrolloffset = HOVER_ROLL_OFFSET*RMAX;
	const uint16_t hoverrollnavkp = (uint16_t)(HOVER_ROLLNAVKP*RMAX);
	const uint16_t hoverrollnavki = (uint16_t)(HOVER_ROLLNAVKI*RMAX);
	const int32_t limitintegralrollNav = (int32_t)(LIMIT_INTEGRAL_ROLLNAV);
#endif

int16_t flap_min = (int16_t)(-FLAP_ANGLE_MAX*(SERVORANGE/60));
int16_t rollOffset;
float rollNavDeflection_filtered_flt = 0;
float rollAngle_filtered_flt = 0;
float roll_error_instant_filtered_flt = 0;
int16_t previous_rollAngle = 0;
int32_t error_integral_roll=0;
int16_t roll_counter=0;

void normalRollCntrl(void);
void hoverRollCntrl(void);

#if (USE_CONFIGFILE == 1)
void init_rollCntrl(void)
{
	yawkdail 	= (uint16_t)(YAWKD_AILERON*SCALEGYRO*RMAX);
	rollkp 		= (uint16_t)(ROLLKP*RMAX);
	rollkd 		= (uint16_t)(ROLLKD*SCALEGYRO*RMAX);

	hoverrollkp = (uint16_t)(HOVER_ROLLKP*RMAX);
	hoverrollkd = (uint16_t)(HOVER_ROLLKD*SCALEGYRO*RMAX);
}
#endif

void rollCntrl(void)
{
#ifdef TestGains
    if (!flight_mode_switch_waypoints()) flags._.GPS_steering = 0; // turn off navigation
	flags._.pitch_feedback = 1;
#endif

	if (canStabilizeHover() && current_orientation == F_HOVER)
	{
		hoverRollCntrl();
	}
	else
	{
		normalRollCntrl();
	}
}

void normalRollCntrl(void)
{
	union longww rollAccum = { 0 };
	union longww gyroRollFeedback;
	union longww gyroYawFeedback;

	fractional rmat6;
	fractional omegaAccum2;

	if (!canStabilizeInverted() || !desired_behavior._.inverted)
	{
		rmat6 = rmat[6];
		omegaAccum2 = omegaAccum[2];
	}
	else
	{
		rmat6 = -rmat[6];
		omegaAccum2 = -omegaAccum[2];
	}

	if (AILERON_NAVIGATION && flags._.GPS_steering)
	{
		rollAccum._.W1 = determine_navigation_deflection('a');
	}

	if (ROLL_STABILIZATION_AILERONS && flags._.pitch_feedback)
	{
		gyroRollFeedback.WW = __builtin_mulus(rollkd , omegaAccum[1]);
		rollAccum.WW += __builtin_mulsu(rmat6 , rollkp);
	}
	else
	{
		gyroRollFeedback.WW = 0;
	}

	if (YAW_STABILIZATION_AILERON && flags._.pitch_feedback)
	{
		gyroYawFeedback.WW = __builtin_mulus(yawkdail, omegaAccum2);
	}
	else
	{
		gyroYawFeedback.WW = 0;
	}

    rollOffset=0;

	roll_control = (int32_t)rollAccum._.W1 - (int32_t)gyroRollFeedback._.W1 - (int32_t)gyroYawFeedback._.W1;
	// Servo reversing is handled in servoMix.c

    //flap control
    flap_control = (int16_t)(FLAP_OFFSET + (udb_pwIn[FLAP_INPUT_CHANNEL] - 2000)*(2*FLAP_ANGLE_MAX)/120+flap_min);
}


void hoverRollCntrl(void)
{
	int16_t rollNavDeflection;
	union longww gyroRollFeedback;
    union longww rollAccum = { 0 };
    fractional rmat2;
    fractional rmat5;
    fractional rmat8;
    int16_t roll_error_instant = 0;
	int16_t roll_error_instant_filtered = 0;
    int16_t averaged_roll_error = 0;
    int16_t rollAngle = 0;
	int16_t rollAngle_filtered = 0;
	int16_t roll_nav_corr = 0;
	int16_t rollNavDeflection_filtered = 0;

	if (flags._.pitch_feedback)
	{
#ifdef TestGains
        rollNavDeflection = 0;
#else
//		if (HOVERING_WAYPOINT_MODE_XY && AILERON_NAVIGATION && flags._.GPS_steering)
//		{
//			rollNavDeflection = (tofinish_line > HOVER_NAV_MAX_PITCH_RADIUS/2) ? determine_navigation_deflection('h') : 0;
//		}
//		else
//		{ 

#if (MANUAL_TARGET_HEIGHT == 0)
		//FLAP_INPUT_CHANNEL controls the target roll nav deflection
	    rollNavDeflection = compute_pot_order(udb_pwIn[FLAP_INPUT_CHANNEL], -128, 127)<<7;
		rollNavDeflection_filtered = exponential_filter(rollNavDeflection, &rollNavDeflection_filtered_flt, 5., (int16_t)(HEARTBEAT_HZ));
#else
		rollNavDeflection = 0;
		rollNavDeflection_filtered = 0;
#endif
//		}
#endif

		if (roll_counter==0)
    	{
        	error_integral_roll=0;
		}

        //computation of roll_angle valid at 360deg around z axis
        
        rmat2=rmat[2];
        rmat5=rmat[5];
        rollAngle = (int16_t)(-arcsine(rmat2));
        if (rmat5 < 0 && previous_rollAngle < 0) rollAngle = -rollAngle-127;
        if (rmat5 < 0 && previous_rollAngle > 0) rollAngle = -rollAngle+127;
        previous_rollAngle = rollAngle;
		rollAngle=rollAngle<<7;
		
        rollAngle_filtered = exponential_filter(rollAngle, &rollAngle_filtered_flt, 5., (int16_t)(HEARTBEAT_HZ));

		//compute instantaneous error on roll angle
        roll_error_instant = rollAngle - rollAngle_filtered;
		roll_error_instant_filtered = exponential_filter(roll_error_instant, &roll_error_instant_filtered_flt, 80., (int16_t)(HEARTBEAT_HZ));

        //correction on roll_error_instant, active if the plane has a significant pitch angle (can happen to ensure pitch equilibrium in wind)
        rmat8 = rmat[8];
        //roll_error_instant = roll_error_instant * (1 - abs(rmat8)/16384)**2
		//roll_error_instant ranges from -16384 to 16383
        roll_error_instant_filtered = (int16_t)(__builtin_mulss(roll_error_instant_filtered, (16384 - abs(rmat8)))>>14);
        roll_error_instant_filtered = (int16_t)(__builtin_mulss(roll_error_instant_filtered, (16384 - abs(rmat8)))>>14);

		//KD controller on instantaneous error on roll angle
		gyroRollFeedback.WW = __builtin_mulus(hoverrollkd , omegaAccum[1]);
        rollAccum.WW = __builtin_mulsu(-roll_error_instant_filtered*2, hoverrollkp);

		//compute error between averaged roll angle and the target roll angle
        averaged_roll_error = rollAngle_filtered - rollNavDeflection_filtered;
        //averaged_roll_error = averaged_roll_error * (1 - abs(rmat8)/16384) + rmat6 * abs(rmat8)
        averaged_roll_error = (int16_t)(__builtin_mulsu(averaged_roll_error, (16384 - abs(rmat8)))>>14);
        int16_t rmat68 = (int16_t)(__builtin_mulss(rmat[6], abs(rmat8))>>14);
        averaged_roll_error = averaged_roll_error - rmat68;

		//compute PI controller on averaged_roll_error
		//roll_nav_corr ranges from -16384 to 16383
		roll_nav_corr=compute_pi_block(-2*(roll_error_instant_filtered + averaged_roll_error), 0, hoverrollnavkp, hoverrollnavki, &error_integral_roll, 
                                    (int16_t)(HEARTBEAT_HZ), limitintegralrollNav, 1);

    	if (roll_counter < RMAX)
    	{
        	roll_counter+=1;
    	}

        additional_int16_export1 = rollAngle*2;
        additional_int16_export2 = averaged_roll_error*2;
        additional_int16_export3 = rmat[2];
        additional_int16_export5 = roll_error_instant_filtered*2;
        additional_int16_export6 = (int16_t)(error_integral_roll / (int16_t)(HEARTBEAT_HZ));
        additional_int16_export7 = roll_nav_corr*2;
        additional_int16_export8 = rollNavDeflection_filtered*2;
        additional_int16_export9 = rmat[8];
		
	}
	else
	{
		rollNavDeflection = 0;
		rollNavDeflection_filtered = 0;
		gyroRollFeedback.WW = 0;
	}

    //roll offset proportional to throttle to compensate propeller torque
    //tmp=-__builtin_mulsu(hoverrolloffset, (int16_t)(throttle_control/2));
    //rollOffset=(int16_t)(tmp>>14);
	//for the moment it is set to zero, it should be taken into account by the interagl term in the PI contoller on roll_averaged_error
    rollOffset=0;

	roll_control = 0;
	//roll_control = - (int32_t)gyroRollFeedback._.W1;
	//roll_control += roll_nav_corr;

    flap_control = (int16_t)(FLAP_OFFSET);

#if (TEST == 1)
    //printf("rmat[2] %d\n rmat[5] %d\n rmat[8] %d\n rmat[6] %d\n rmat6_128 %d\n rollNavDeflection %d\n rollAngle %d\n rollAngle_filtered %d\n averaged_roll_error %d\n roll_error_instant %d\n roll_control %d\n", rmat[2], rmat[5], rmat[8], rmat[6], rmat6_128, rollNavDeflection, rollNavDeflection_filtered, rollAngle, rollAngle_filtered, averaged_roll_error, roll_error_instant, roll_control );
    printf("%d %d %d %d %d %d %d %d %d %d %d %d\n", rmat[2], rmat[5], rmat[8], rmat[6], rmat6_128>>7, rollNavDeflection>>7, rollAngle>>7, rollAngle_filtered>>7, averaged_roll_error>>7, roll_error_instant>>7, roll_control );
#endif
}
