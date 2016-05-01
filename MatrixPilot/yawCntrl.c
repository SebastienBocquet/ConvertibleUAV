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

//to add HOVER_YAW_TOWARDS_WP
#define HOVERPTOWP ((int16_t)(HOVER_ANGLE_TOWARDS_WP*(RMAX/57.3)))
#define COEF_MAX            1
#define COEF_SCALING        (RMAX/COEF_MAX)

#if (USE_CONFIGFILE == 1)
#include "config.h"
#include "redef.h"

	uint16_t yawkdrud;
	uint16_t rollkprud;
	uint16_t rollkdrud;
	uint16_t hoveryawkp;
	uint16_t hoveryawkd;
    int16_t hoveryawoffset;

#elif ((SERIAL_OUTPUT_FORMAT == SERIAL_MAVLINK) || (GAINS_VARIABLE == 1))
	uint16_t yawkdrud			= (uint16_t)(YAWKD_RUDDER*SCALEGYRO*RMAX);
	uint16_t rollkprud			= (uint16_t)(ROLLKP_RUDDER*RMAX);
	uint16_t rollkdrud			= (uint16_t)(ROLLKD_RUDDER*SCALEGYRO*RMAX);
	uint16_t hoveryawkp			= (uint16_t)(HOVER_YAWKP*RMAX);
	uint16_t hoveryawkd			= (uint16_t)(HOVER_YAWKD*SCALEGYRO*RMAX);
    int16_t hoveryawoffset = (int16_t)(HOVER_YAW_OFFSET*(SERVORANGE/60));
    uint16_t hoveryawToWPkp = (uint16_t)(HOVER_YAWTOWPKP*COEF_SCALING);
    uint16_t hoveryawToWPki = (uint16_t)(HOVER_YAWTOWPKI*COEF_SCALING);
    float invdeltafilteryaw = (float)(HOVER_INV_DELTA_FILTER_YAW);
#else
	const uint16_t yawkdrud		= (uint16_t)(YAWKD_RUDDER*SCALEGYRO*RMAX);
	const uint16_t rollkprud	= (uint16_t)(ROLLKP_RUDDER*RMAX);
	const uint16_t rollkdrud	= (uint16_t)(ROLLKD_RUDDER*SCALEGYRO*RMAX);
	const uint16_t hoveryawkp	= (uint16_t)(HOVER_YAWKP*RMAX);
	const uint16_t hoveryawkd	= (uint16_t)(HOVER_YAWKD*SCALEGYRO*RMAX);
    const int16_t hoveryawoffset = (int16_t)(HOVER_YAW_OFFSET*(SERVORANGE/60));
    const uint16_t hoveryawToWPkp = (uint16_t)(HOVER_YAWTOWPKP*COEF_SCALING);
    const uint16_t hoveryawToWPki = (uint16_t)(HOVER_YAWTOWPKI*COEF_SCALING);
    const float invdeltafilteryaw = (float)(HOVER_INV_DELTA_FILTER_YAW);
#endif

void normalYawCntrl(void);
void hoverYawCntrl(void);
int16_t hovering_yaw_dir;
int16_t yaw_error_integral = 0;
float yaw_error_filtered_flt = 0.;
int16_t yaw_error_filtered = 0;
int16_t yaw_hover_counter = 0;

int16_t hover_error_y = 0;
int16_t hover_error_integral_y = 0;

#if (USE_CONFIGFILE == 1)
void init_yawCntrl(void)
{
	yawkdrud 	= (uint16_t)(YAWKD_RUDDER*SCALEGYRO*RMAX);
	rollkprud 	= (uint16_t)(ROLLKP_RUDDER*RMAX);
	rollkdrud 	= (uint16_t)(ROLLKD_RUDDER*SCALEGYRO*RMAX);
	hoveryawkp 	= (uint16_t)(HOVER_YAWKP*RMAX);
	hoveryawkd 	= (uint16_t)(HOVER_YAWKD*SCALEGYRO*RMAX);
    hoveryawoffset = (int16_t)(HOVER_YAW_OFFSET*(SERVORANGE/60));
    uint16_t hoveryawToWPkp = (uint16_t)(HOVER_YAWTOWPKP*COEF_SCALING);
    uint16_t hoveryawToWPki = (uint16_t)(HOVER_YAWTOWPKI*COEF_SCALING);
}
#endif

void yawCntrl(void)
{
#ifdef TestGains
	if (!flight_mode_switch_waypoints()) flags._.GPS_steering = 0; // turn off navigation
	flags._.pitch_feedback = 1; // turn on stabilization
#endif

	if (canStabilizeHover() && current_orientation == F_HOVER)
	{
		hoverYawCntrl();
	}
	else
	{
		normalYawCntrl();
	}
}

void normalYawCntrl(void)
{
	int16_t yawNavDeflection;
	union longww rollStabilization;
	union longww gyroYawFeedback;
	int16_t ail_rud_mix;
 
	if (RUDDER_NAVIGATION && flags._.GPS_steering)
	{
		yawNavDeflection = determine_navigation_deflection('y');
		
		if (canStabilizeInverted() && current_orientation == F_INVERTED)
		{
			yawNavDeflection = -yawNavDeflection;
		}
	}
	else
	{
		yawNavDeflection = 0;
	}

	if (YAW_STABILIZATION_RUDDER && flags._.pitch_feedback)
	{
		gyroYawFeedback.WW = __builtin_mulus(yawkdrud, omegaAccum[2]);
	}
	else
	{
		gyroYawFeedback.WW = 0;
	}

	rollStabilization.WW = 0; // default case is no roll rudder stabilization
	if (ROLL_STABILIZATION_RUDDER && flags._.pitch_feedback)
	{
		if (!desired_behavior._.inverted && !desired_behavior._.hover)  // normal
		{
			rollStabilization.WW = __builtin_mulsu(rmat[6] , rollkprud);
		}
		else if (desired_behavior._.inverted) // inverted
		{
			rollStabilization.WW = - __builtin_mulsu(rmat[6] , rollkprud);
		}
		rollStabilization.WW -= __builtin_mulus(rollkdrud , omegaAccum[1]);
	}

	if (flags._.pitch_feedback)
	{
		int16_t ail_offset = (udb_flags._.radio_on) ? (udb_pwIn[AILERON_INPUT_CHANNEL] - udb_pwTrim[AILERON_INPUT_CHANNEL]) : 0;
		ail_rud_mix = MANUAL_AILERON_RUDDER_MIX * REVERSE_IF_NEEDED(AILERON_CHANNEL_REVERSED, ail_offset);
		if (canStabilizeInverted() && current_orientation == F_INVERTED) ail_rud_mix = -ail_rud_mix;
	}
	else
	{
		ail_rud_mix = 0;
	}

	yaw_control = (int32_t)yawNavDeflection 
				- (int32_t)gyroYawFeedback._.W1 
				+ (int32_t)rollStabilization._.W1 
				+ ail_rud_mix;
	// Servo reversing is handled in servoMix.c
}

void hoverYawCntrl(void)
{
	union longww yawAccum;
	union longww gyroYawFeedback;

	if (flags._.pitch_feedback)
	{
        int16_t yawToWP;

        if (HOVERING_WAYPOINT_MODE_XY && flags._.GPS_steering)
		{
            int16_t yaw_error;  //error along yaw axis between aircraft position and goal (origin point here) in cm
            int16_t yawTarget = 0;

            if (yaw_hover_counter==0)
            {
                yaw_error_filtered = 0;
                yaw_error_integral = 0;
            }

		    if (yaw_hover_counter < RMAX)
		    {
		        yaw_hover_counter+=1;
		    }

            determine_navigation_deflection('h');
            compute_hovering_dir();
            
            int32_t tmp = __builtin_mulss(hovering_yaw_dir, tofinish_line);

            int32_t tmp2 = __builtin_mulss((int16_t)(tmp/MAX_HOVERING_RADIUS), HOVERPTOWP);

            //yaw_error ranges from -(max angle to WP in hover) to +(max angle to WP in hover)
            yaw_error = tmp2>>14; 

            //filter error
            yaw_error_filtered = exponential_filter(yaw_error, &yaw_error_filtered_flt, invdeltafilteryaw, (int16_t)(HEARTBEAT_HZ));
            
            //PI controller
            yawToWP = compute_pid_block(yaw_error_filtered, yawTarget, hoveryawToWPkp, hoveryawToWPki, &yaw_error_integral, 
                                    (int16_t)(HEARTBEAT_HZ), (yaw_hover_counter > nb_sample_wait));
            
            //limit yaw to max angle
            yawToWP = limit_value(yawToWP, -HOVERPTOWP, HOVERPTOWP);

            //yawToWP = (tofinish_line > HOVER_NAV_MAX_PITCH_RADIUS) ?
			//    HOVERPTOWP : (HOVERPTOWP / (100*HOVER_NAV_MAX_PITCH_RADIUS) * error);  

            hover_error_y = yaw_error_filtered;
            hover_error_integral_y = (int16_t)(yaw_error_integral / (int16_t)(HEARTBEAT_HZ));
        }
        else
		{
			yawToWP = 0;
		}

		gyroYawFeedback.WW = __builtin_mulus(hoveryawkd , omegaAccum[2]);
		
		int16_t yawInput = (udb_flags._.radio_on == 1) ? REVERSE_IF_NEEDED(RUDDER_CHANNEL_REVERSED, udb_pwIn[RUDDER_INPUT_CHANNEL] - udb_pwTrim[RUDDER_INPUT_CHANNEL]) : 0;
		int16_t manualYawOffset = 0; //yawInput * (int16_t)(RMAX/2000);
		
        yawAccum.WW = __builtin_mulsu(rmat[6] + yawToWP + manualYawOffset , hoveryawkp);
	}
	else
	{
		gyroYawFeedback.WW = 0;
		yawAccum.WW = 0;
	}

	yaw_control = (int32_t)yawAccum._.W1 - (int32_t)gyroYawFeedback._.W1 + hoveryawoffset;
}
