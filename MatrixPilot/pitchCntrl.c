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

//why 57.3 instead of 90. ?
#define ANGLE_90DEG (RMAX/(2*57.3))
#define RTLKICK ((int32_t)(RTL_PITCH_DOWN*(RMAX/57.3)))
#define INVNPITCH ((int32_t)(INVERTED_NEUTRAL_PITCH*(RMAX/57.3)))

#if (USE_CONFIGFILE == 1)
#include "config.h"
#include "redef.h"

	uint16_t pitchgain;
	uint16_t pitchkd;
	uint16_t hoverpitchkp;
	uint16_t hoverpitchkd;
	uint16_t rudderElevMixGain;
	uint16_t rollElevMixGain;
    uint16_t hoverpitchToWPkp;
    uint16_t hoverpitchToWPki;
    uint16_t hoverpitchToWPvkp;
	int32_t limitintegralpitchToWP;
    int16_t limittargetpitchV;

#elif ((SERIAL_OUTPUT_FORMAT == SERIAL_MAVLINK) || (GAINS_VARIABLE == 1))
	uint16_t pitchgain = (uint16_t)(PITCHGAIN*RMAX);
	uint16_t pitchkd = (uint16_t) (PITCHKD*SCALEGYRO*RMAX);
	uint16_t rudderElevMixGain = (uint16_t)(RMAX*RUDDER_ELEV_MIX);
	uint16_t rollElevMixGain = (uint16_t)(RMAX*ROLL_ELEV_MIX);
    uint16_t hoverpitchToWPkp = (uint16_t)(HOVER_PITCHTOWPKP*RMAX);
    uint16_t hoverpitchToWPki = (uint16_t)(HOVER_PITCHTOWPKI*RMAX);
    uint16_t hoverpitchToWPvkp = (uint16_t)(HOVER_PITCHTOWPVKP*RMAX);
	int32_t limitintegralpitchToWP = (int32_t)(LIMIT_INTEGRAL_PITCHTOWP);
    int16_t limittargetpitchV = (int16_t)(HOVER_LIMIT_TARGETVPITCH);
#else
	const uint16_t pitchgain = (uint16_t)(PITCHGAIN*RMAX);
	const uint16_t pitchkd = (uint16_t) (PITCHKD*SCALEGYRO*RMAX);
	const uint16_t rudderElevMixGain = (uint16_t)(RMAX*RUDDER_ELEV_MIX);
	const uint16_t rollElevMixGain = (uint16_t)(RMAX*ROLL_ELEV_MIX);
    uint16_t hoverpitchToWPkp = (uint16_t)(HOVER_PITCHTOWPKP*RMAX);
    uint16_t hoverpitchToWPvkp = (uint16_t)(HOVER_PITCHTOWPVKP*RMAX);
	const int32_t limitintegralpitchToWP = (int32_t)(LIMIT_INTEGRAL_PITCHTOWP);
    const int32_t limitintegralpitchVToWP = (int32_t)(LIMIT_INTEGRAL_VPITCHTOWP);
#ifdef TestGPSPositioning
    uint16_t hoverpitchToWPki = 0;
    const int16_t limittargetpitchV = RMAX;
#else
    uint16_t hoverpitchToWPki = (uint16_t)(HOVER_PITCHTOWPKI*RMAX);
    const int16_t limittargetpitchV = (int16_t)(HOVER_LIMIT_TARGETVPITCH);
#endif
#endif

int16_t pitchrate;
int16_t navElevMix;
int16_t elevInput;

int16_t hovering_pitch_order;
int16_t target_pitch = 0;
int16_t pitch_v_target = 0;
int32_t pitch_error_integral = 0;
int32_t pitch_v_error_integral = 0;
int16_t pitch_hover_corr = 0;

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
        flags._.GPS_steering = 1; // turn navigation on
        //flags._.pitch_feedback = 1;
    }
    else
    {
	    flags._.GPS_steering = 0; // turn navigation off
        //flags._.pitch_feedback = 1;
    }
#endif

	if (current_orientation == F_HOVER || desired_behavior._.hover)
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
    int16_t max_tilt_sine = sine((int8_t)(MAX_TILT*.7111));

    if (flags._.pitch_feedback && flags._.GPS_steering)
	{
        //error along y axis between aircraft position and goal (origin point here) in cm
        
        if (control_position_hold)
        {
            target_pitch = compute_target_pitch(hovering_pitch_order, tofinish_line_factor10, max_tilt_sine);
        }
        else
        {
            target_pitch = 0;
            pitch_error_integral = 0;
            pitch_v_error_integral = 0;
        }

        //additional_int16_export4 = target_pitch;
        
        uint16_t horizontal_air_speed = vector2_mag(IMUvelocityx._.W1 - estimatedWind[0], 
	                                   IMUvelocityy._.W1 - estimatedWind[1]);
        
#ifdef TestGPSPositioning
        horizontal_air_speed = 0;
#endif
        
		//PI controller on pitch angle
		pitch_v_target = compute_pi_block(-rmat[7], -target_pitch, hoverpitchToWPkp, hoverpitchToWPki, &pitch_error_integral, 
                                    (int16_t)(SERVO_HZ), limitintegralpitchToWP, control_position_hold);
        
        //additional_int16_export3 = -rmat[7];
                
        pitch_v_target = limit_value(pitch_v_target, -limittargetpitchV, limittargetpitchV);
        
        pitch_hover_corr = compute_pi_block(horizontal_air_speed, pitch_v_target, hoverpitchToWPvkp, 0, &pitch_v_error_integral, 
                                  (int16_t)(SERVO_HZ), limitintegralpitchVToWP, control_position_hold);
        
        //additional_int16_export8 = pitch_hover_corr;
	}
	else
	{
		pitch_hover_corr = 0;
	}

	pitch_control = pitch_hover_corr;
}
