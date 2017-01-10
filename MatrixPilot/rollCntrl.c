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

#define HOVERPTOWP ((int32_t)(HOVER_ANGLE_TOWARDS_WP*(RMAX/57.3)))

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
	int32_t limitintegralrollToWP = (int32_t)(LIMIT_INTEGRAL_ROLLTOWP);
    float invdeltafilterroll = (float)(HOVER_INV_DELTA_FILTER_ROLL);
#else
	const uint16_t hoverrollkp	= (uint16_t)(HOVER_ROLLKP*RMAX);
	const uint16_t hoverrollkd	= (uint16_t)(HOVER_ROLLKD*SCALEGYRO*RMAX);
    const fractional hoverrolloffset = HOVER_ROLL_OFFSET*RMAX;
    const uint16_t hoverrollToWPkp = (uint16_t)(HOVER_ROLLTOWPKP*RMAX);
    const uint16_t hoverrollToWPki = (uint16_t)(HOVER_ROLLTOWPKI*RMAX);
	const int32_t limitintegralrollToWP = (int32_t)(LIMIT_INTEGRAL_ROLLTOWP);
    const float invdeltafilterroll = (float)(HOVER_INV_DELTA_FILTER_ROLL);
#endif

int16_t flap_min = (int16_t)(-FLAP_ANGLE_MAX*(SERVORANGE/60));
int16_t hovering_roll_dir;
int32_t roll_error_integral = 0;
float roll_error_filtered_flt = 0.;
int16_t roll_error_filtered = 0;
int16_t roll_hover_counter = 0;

int16_t hover_error_x = 0;
int16_t hover_error_integral_x = 0;

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

	roll_control = (int32_t)rollAccum._.W1 - (int32_t)gyroRollFeedback._.W1 - (int32_t)gyroYawFeedback._.W1;
	// Servo reversing is handled in servoMix.c

    //flap control
    flap_control = (int16_t)(FLAP_OFFSET + (udb_pwIn[FLAP_INPUT_CHANNEL] - 2000)*(2*FLAP_ANGLE_MAX)/120+flap_min);
}


void hoverRollCntrl(void)
{
	union longww rollAccum;
    int16_t rollCorr = 0;
	int16_t rollToWP = 0;

    if (flags._.pitch_feedback && HOVERING_WAYPOINT_MODE_XY && flags._.GPS_steering)
	{
        //error along yaw axis between aircraft position and goal (origin point here) in cm

        if (hover_counter==0)
        {
            roll_error_filtered = 0;
            roll_error_integral = 0;
        }

        determine_navigation_deflection('y');
            
        int32_t tmp = __builtin_mulss(hovering_roll_dir, tofinish_line);
        int32_t tmp2 = __builtin_mulss((int16_t)(tmp/MAX_HOVERING_RADIUS), HOVERPTOWP);

        //roll_error ranges from -(max angle to WP in hover) to +(max angle to WP in hover)

        //filter error
        rollToWP = -exponential_filter(tmp2>>14, &roll_error_filtered_flt, invdeltafilterroll, (int16_t)(HEARTBEAT_HZ));

        //limit yaw to max angle
        rollToWP = limit_value(rollToWP, -HOVERPTOWP, HOVERPTOWP);

        //yawToWP = (tofinish_line > HOVER_NAV_MAX_PITCH_RADIUS) ?
	    //    HOVERPTOWP : (HOVERPTOWP / (100*HOVER_NAV_MAX_PITCH_RADIUS) * error);  

        rollAccum.WW = __builtin_mulsu(rollToWP, hoverrollkp);

        int16_t roll_error = rollToWP;

		//PI controller on roll_error
		rollCorr = compute_pi_block(roll_error, 0, hoverrollToWPkp, hoverrollToWPki, &roll_error_integral, 
                                    (int16_t)(HEARTBEAT_HZ), limitintegralrollToWP, (hover_counter > nb_sample_wait));

		hover_error_x = roll_error;
        hover_error_integral_x = (int16_t)(roll_error_integral / (int16_t)(HEARTBEAT_HZ));
	}
	else
	{
		rollCorr = 0;
	}

	roll_control = -rollCorr;

    flap_control = (int16_t)(FLAP_OFFSET);
}
