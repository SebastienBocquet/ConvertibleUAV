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
#include "airspeedCntrl.h"
#include "mode_switch.h"
#include "../libUDB/heartbeat.h"

//  If the state machine selects pitch feedback, compute it from the pitch gyro and accelerometer.

#define ANGLE_90DEG (RMAX/(2*57.3))
#define RTLKICK ((int32_t)(RTL_PITCH_DOWN*(RMAX/57.3)))
#define INVNPITCH ((int32_t)(INVERTED_NEUTRAL_PITCH*(RMAX/57.3)))
#define HOVERPTOWP ((int32_t)(HOVER_ANGLE_TOWARDS_WP*(RMAX/57.3)))

#if (USE_CONFIGFILE == 1)
#include "config.h"
#include "redef.h"

	uint16_t pitchgain;
	uint16_t pitchkd;
	uint16_t hoverpitchkp;
	uint16_t hoverpitchkd;
	uint16_t rudderElevMixGain;
	uint16_t rollElevMixGain;
    int16_t hoverpitchffset;

#elif ((SERIAL_OUTPUT_FORMAT == SERIAL_MAVLINK) || (GAINS_VARIABLE == 1))
	uint16_t pitchgain = (uint16_t)(PITCHGAIN*RMAX);
	uint16_t pitchkd = (uint16_t) (PITCHKD*SCALEGYRO*RMAX);
	uint16_t hoverpitchkp = (uint16_t)(HOVER_PITCHKP*RMAX);
	uint16_t hoverpitchkd = (uint16_t) (HOVER_PITCHKD*SCALEGYRO*RMAX);
	uint16_t rudderElevMixGain = (uint16_t)(RMAX*RUDDER_ELEV_MIX);
	uint16_t rollElevMixGain = (uint16_t)(RMAX*ROLL_ELEV_MIX);
    //this is to impose an offset on the axis angle of the plane
    //int16_t hoverpitchoffset = (int16_t)(HOVER_PITCH_OFFSET*(RMAX/57.3));
    //this is to impose an offset to the control surface angle
    int16_t hoverpitchoffset = (int16_t)(HOVER_PITCH_OFFSET*(SERVORANGE/60));
    uint16_t hoverpitchToWPkp = (uint16_t)(HOVER_PITCHTOWPKP*RMAX);
    uint16_t hoverpitchToWPki = (uint16_t)(HOVER_PITCHTOWPKI*RMAX);
	int32_t limitintegralpitchToWP = (int32_t)(LIMIT_INTEGRAL_PITCHTOWP);
    float invdeltafilterpitch = (float)(HOVER_INV_DELTA_FILTER_PITCH);
#else
	const uint16_t pitchgain = (uint16_t)(PITCHGAIN*RMAX);
	const uint16_t pitchkd = (uint16_t) (PITCHKD*SCALEGYRO*RMAX);
	const uint16_t hoverpitchkp = (uint16_t)(HOVER_PITCHKP*RMAX);
	const uint16_t hoverpitchkd = (uint16_t) (HOVER_PITCHKD*SCALEGYRO*RMAX);
	const uint16_t rudderElevMixGain = (uint16_t)(RMAX*RUDDER_ELEV_MIX);
	const uint16_t rollElevMixGain = (uint16_t)(RMAX*ROLL_ELEV_MIX);
    const int16_t hoverpitchoffset = (int16_t)(HOVER_PITCH_OFFSET*(SERVORANGE/60));
    const uint16_t hoverpitchToWPkp = (uint16_t)(HOVER_PITCHTOWPKP*RMAX);
    const uint16_t hoverpitchToWPki = (uint16_t)(HOVER_PITCHTOWPKI*RMAX);
	const int32_t limitintegralpitchToWP = (int32_t)(LIMIT_INTEGRAL_PITCHTOWP);
    const float invdeltafilterpitch = (float)(HOVER_INV_DELTA_FILTER_PITCH);
#endif

int16_t pitchrate;
int16_t navElevMix;
int16_t elevInput;

int16_t hovering_pitch_dir;
int32_t pitch_error_integral = 0;
float pitch_error_filtered_flt = 0.;
int16_t pitch_error_filtered = 0;
int16_t pitch_hover_counter = 0;

int16_t hover_error_x = 0;
int16_t hover_error_integral_x = 0;

void normalPitchCntrl(void);
void hoverPitchCntrl(void);

#if (USE_CONFIGFILE == 1)
void init_pitchCntrl(void)
{
	pitchgain = (uint16_t)(PITCHGAIN*RMAX);
	pitchkd = (uint16_t) (PITCHKD*SCALEGYRO*RMAX);
	hoverpitchkp = (uint16_t)(HOVER_PITCHGAIN*RMAX);
	hoverpitchkd = (uint16_t) (HOVER_PITCHKD*SCALEGYRO*RMAX);
	rudderElevMixGain = (uint16_t)(RMAX*RUDDER_ELEV_MIX);
	rollElevMixGain = (uint16_t)(RMAX*ROLL_ELEV_MIX);
    hoverpitchoffset = (int16_t)(HOVER_PITCH_OFFSET*(SERVORANGE/60));
    uint16_t hoverpitchToWPkp = (uint16_t)(HOVER_PITCHTOWPKP*COEF_SCALING);
    uint16_t hoverpitchToWPki = (uint16_t)(HOVER_PITCHTOWPKI*COEF_SCALING);
}
#endif

void pitchCntrl(void)
{
#ifdef TestGains
    if (flight_mode_switch_waypoints())
    {
        flags._.GPS_steering = 1; // turn navigation off
	    flags._.pitch_feedback = 1; // turn stabilization on
        flags._.test_hover_throttle = 1; //allow to pass into dead_reckoning even if GPS is not initialized   
    }
    else
    {
	    flags._.GPS_steering = 0; // turn navigation off
	    flags._.pitch_feedback = 1; // turn stabilization on
        flags._.test_hover_throttle = 1; //allow to pass into dead_reckoning even if GPS is not initialized
    }
#endif

	if (canStabilizeHover() && desired_behavior._.hover)
	{
		hoverPitchCntrl();
	}
	else
	{
		normalPitchCntrl();
	}
}

void normalPitchCntrl(void)
{
	union longww pitchAccum;
	int16_t rtlkick;
//	int16_t aspd_adj;
//	fractional aspd_err, aspd_diff;

	fractional rmat6;
	fractional rmat7;
	fractional rmat8;

	if (!canStabilizeInverted() || current_orientation != F_INVERTED)
	{
		rmat6 = rmat[6];
		rmat7 = rmat[7];
		rmat8 = rmat[8];
	}
	else
	{
		rmat6 = -rmat[6];
		rmat7 = -rmat[7];
		rmat8 = -rmat[8];
		pitchAltitudeAdjust = -pitchAltitudeAdjust - INVNPITCH;
	}

	navElevMix = 0;
	if (flags._.pitch_feedback)
	{
		if (RUDDER_OUTPUT_CHANNEL != CHANNEL_UNUSED && RUDDER_INPUT_CHANNEL != CHANNEL_UNUSED) {
			pitchAccum.WW = __builtin_mulsu(rmat6 , rudderElevMixGain) << 1;
			pitchAccum.WW = __builtin_mulss(pitchAccum._.W1 ,
			    REVERSE_IF_NEEDED(RUDDER_CHANNEL_REVERSED, udb_pwTrim[RUDDER_INPUT_CHANNEL] - udb_pwOut[RUDDER_OUTPUT_CHANNEL])) << 3;
			navElevMix += pitchAccum._.W1;
		}

		pitchAccum.WW = __builtin_mulsu(rmat6 , rollElevMixGain) << 1;
		pitchAccum.WW = __builtin_mulss(pitchAccum._.W1 , rmat[6]) >> 3;
		navElevMix += pitchAccum._.W1;
	}

	pitchAccum.WW = (__builtin_mulss(rmat8 , omegagyro[0])
	               - __builtin_mulss(rmat6 , omegagyro[2])) << 1;
	pitchrate = pitchAccum._.W1;
	
	if (!udb_flags._.radio_on && flags._.GPS_steering)
	{
		rtlkick = RTLKICK;
	}
	else
	{
		rtlkick = 0;
	}

#if(GLIDE_AIRSPEED_CONTROL == 1)
	fractional aspd_pitch_adj = gliding_airspeed_pitch_adjust();
#endif

	if (PITCH_STABILIZATION && flags._.pitch_feedback)
	{
#if(GLIDE_AIRSPEED_CONTROL == 1)
		pitchAccum.WW = __builtin_mulsu(rmat7 - rtlkick + aspd_pitch_adj + pitchAltitudeAdjust, pitchgain) 
		              + __builtin_mulus(pitchkd , pitchrate);
#else
		pitchAccum.WW = __builtin_mulsu(rmat7 - rtlkick + pitchAltitudeAdjust, pitchgain) 
		              + __builtin_mulus(pitchkd , pitchrate);
#endif
	}
	else
	{
		pitchAccum.WW = 0;
	}

	pitch_control = (int32_t)pitchAccum._.W1 + navElevMix;
}

void hoverPitchCntrl(void)
{
	union longww pitchAccum;
	int16_t manualPitchOffset;
    int16_t pitchCorr;

	if (flags._.pitch_feedback)
	{
		pitchAccum.WW = (__builtin_mulss(-rmat[7] , omegagyro[0])
		               - __builtin_mulss(rmat[6] , omegagyro[1])) << 1;
		pitchrate = pitchAccum._.W1;
		
		int16_t elevInput = (udb_flags._.radio_on == 1) ?
		    REVERSE_IF_NEEDED(ELEVATOR_CHANNEL_REVERSED, udb_pwIn[ELEVATOR_INPUT_CHANNEL] - udb_pwTrim[ELEVATOR_INPUT_CHANNEL]) : 0;

#if (MANUAL_TARGET_HEIGHT == 0)
	    manualPitchOffset = compute_pot_order(udb_pwIn[CAMERA_PITCH_INPUT_CHANNEL], -128, 127)*128;
#else
		manualPitchOffset = 0;
#endif
		int16_t pitchToWP;

#ifdef TestGains
        pitchToWP = 0;
#else
		if (HOVERING_WAYPOINT_MODE_XY && flags._.GPS_steering)
		{

            if (pitch_hover_counter==0)
            {
                pitch_error_filtered = 0;
                pitch_error_integral = 0;
            }

		    if (pitch_hover_counter < RMAX)
		    {
		        pitch_hover_counter+=1;
		    }

            determine_navigation_deflection('h');
            compute_hovering_dir();

            int32_t tmp = __builtin_mulss(hovering_pitch_dir, tofinish_line);
            
            int32_t tmp2 = __builtin_mulss((int16_t)(tmp/MAX_HOVERING_RADIUS), HOVERPTOWP);
            
			//pitchToWP = error along pitch axis between aircraft position and goal (origin point here) in cm
			//filter error
            pitchToWP = -exponential_filter(tmp2>>14, &pitch_error_filtered_flt, invdeltafilterpitch, (int16_t)(HEARTBEAT_HZ));

            //limit pitch to max angle
            pitchToWP = limit_value(pitchToWP, -HOVERPTOWP, HOVERPTOWP);   
		}
		else
		{
			pitchToWP = 0;
		}
#endif 

        //pitchAccum.WW = __builtin_mulsu(rmat[8] - pitchToWP + manualPitchOffset , hoverpitchkp)
		//              + __builtin_mulus(hoverpitchkd , pitchrate);

		pitchAccum.WW = __builtin_mulus(hoverpitchkd , pitchrate);

        //PI controller
        int16_t pitch_error = rmat[8] - pitchToWP + manualPitchOffset;

		//limit value to -HOVERPTOWP : 85*RMAX/57.3
        pitch_error = limit_value(pitch_error, -HOVERPTOWP, (int32_t)(HOVER_ANGLE_TOWARDS_WP*(RMAX/57.3)));

        pitchCorr = compute_pi_block(pitch_error, 0, hoverpitchToWPkp, hoverpitchToWPki, &pitch_error_integral, 
                                    (int16_t)(HEARTBEAT_HZ), limitintegralpitchToWP, (pitch_hover_counter > nb_sample_wait));

		hover_error_x = pitch_error;
        hover_error_integral_x = (int16_t)(pitch_error_integral / (int16_t)(HEARTBEAT_HZ));
	}
	else
	{
		pitchAccum.WW = 0;
        pitchCorr = 0;
	}
	pitch_control = -pitchCorr + (int32_t)pitchAccum._.W1 + hoverpitchoffset;
}
