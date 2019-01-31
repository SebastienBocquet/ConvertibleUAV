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

#if (USE_CONFIGFILE == 1)
#include "config.h"
#include "redef.h"

	uint16_t yawkdrud;
	uint16_t rollkprud;
	uint16_t rollkdrud;

#elif ((SERIAL_OUTPUT_FORMAT == SERIAL_MAVLINK) || (GAINS_VARIABLE == 1))
	uint16_t yawkdrud			= (uint16_t)(YAWKD_RUDDER*SCALEGYRO*RMAX);
	uint16_t rollkprud			= (uint16_t)(ROLLKP_RUDDER*RMAX);
	uint16_t rollkdrud			= (uint16_t)(ROLLKD_RUDDER*SCALEGYRO*RMAX);
#else
	const uint16_t yawkdrud		= (uint16_t)(YAWKD_RUDDER*SCALEGYRO*RMAX);
	const uint16_t rollkprud	= (uint16_t)(ROLLKP_RUDDER*RMAX);
	const uint16_t rollkdrud	= (uint16_t)(ROLLKD_RUDDER*SCALEGYRO*RMAX);
#endif

void normalYawCntrl(void);
void hoverYawCntrl(void);

#if (USE_CONFIGFILE == 1)
void init_yawCntrl(void)
{
	yawkdrud 	= (uint16_t)(YAWKD_RUDDER*SCALEGYRO*RMAX);
	rollkprud 	= (uint16_t)(ROLLKP_RUDDER*RMAX);
	rollkdrud 	= (uint16_t)(ROLLKD_RUDDER*SCALEGYRO*RMAX);
    uint16_t hoveryawToWPkp = (uint16_t)(HOVER_YAWTOWPKP*COEF_SCALING);
    uint16_t hoveryawToWPki = (uint16_t)(HOVER_YAWTOWPKI*COEF_SCALING);
}
#endif

void yawCntrl(void)
{
#ifdef TestGains
	if (!flight_mode_switch_waypoints()) flags._.GPS_steering = 0; // turn off navigation
	//flags._.pitch_feedback = 1; // turn on stabilization
#endif

	if (current_orientation == F_HOVER)
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
    int16_t plane_to_init_yaw_angle = 0;

#if (MANUAL_HEADING == 1)
    plane_to_init_yaw_angle = compute_pot_order(udb_pwIn[INPUT_CHANNEL_AUX1], -128, 127)<<8;
#endif

	//additional_int16_export5 = plane_to_north;

    yaw_control = plane_to_init_yaw_angle;
}
