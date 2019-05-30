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
#include "../libUDB/lidarAltitude.h"
#include "../libUDB/sonarAltitude.h"
#include "../libUDB/barometerAltitude.h"
#include "../libUDB/barometer.h"
#include "../libDCM/libDCM_internal.h"

#if (ALTITUDE_GAINS_VARIABLE != 1)

union longww throttleFiltered = { 0 };

#define THROTTLEFILTSHIFT   12

#define COEF_MAX            10  //maximum value of pid coef
#define COEF_SCALING        (RMAX/COEF_MAX)

#define MAXTHROTTLE         (2.0*SERVORANGE*ALT_HOLD_THROTTLE_MAX)
#define FIXED_WP_THROTTLE   (2.0*SERVORANGE*RACING_MODE_WP_THROTTLE)

#define THROTTLEHEIGHTGAIN  (((ALT_HOLD_THROTTLE_MAX - ALT_HOLD_THROTTLE_MIN)*2.0*SERVORANGE)/(HEIGHT_MARGIN*2.0))

#define PITCHATMAX          (ALT_HOLD_PITCH_MAX*(RMAX/57.3))
#define PITCHATMIN          (ALT_HOLD_PITCH_MIN*(RMAX/57.3))
#define PITCHATZERO         (ALT_HOLD_PITCH_HIGH*(RMAX/57.3))
#define PITCHHEIGHTGAIN     ((PITCHATMAX - PITCHATMIN) / (HEIGHT_MARGIN*2.0))
#define HEIGHTTHROTTLEGAIN  ((1.5*(HEIGHT_TARGET_MAX-HEIGHT_TARGET_MIN)* 1024.0) / (SERVORANGE*SERVOSAT))
#define VZ_CORR_16           VZ_CORR*RMAX
#define RAMPE_TIME_LANDING 3
#define RAMPE_DECREMENT -RMAX / (80 * RAMPE_TIME_LANDING)

#define NOT_CLOSE_TO_GROUND_DETECT_TIME 2 //is seconds
#define ALT_SENSOR_UNCERTAINTY 15 //in cm

int16_t aircraft_mass       = AIRCRAFT_MASS;
int16_t max_thrust          = MAX_THRUST;

int32_t speed_height = 0;
int16_t pitchAltitudeAdjust = 0;
boolean filterManual = false;

int16_t desiredHeight;

void normalAltitudeCntrl(void);
void manualThrottle(int16_t throttleIn);
void manualHoverThrottle(int16_t throttleIn);
void hoverAltitudeCntrl(void);

// Variables required for mavlink.  Used in AltitudeCntrlVariable and airspeedCntrl
#if (SERIAL_OUTPUT_FORMAT == SERIAL_MAVLINK)
// External variables
int16_t height_target_min           = HEIGHT_TARGET_MIN;
int16_t height_target_max           = HEIGHT_TARGET_MAX;
int16_t height_margin               = HEIGHT_MARGIN;
fractional alt_hold_throttle_min    = ALT_HOLD_THROTTLE_MIN * RMAX;
fractional alt_hold_throttle_max    = ALT_HOLD_THROTTLE_MAX * RMAX;
int16_t alt_hold_pitch_min          = ALT_HOLD_PITCH_MIN;
int16_t alt_hold_pitch_max          = ALT_HOLD_PITCH_MAX;
int16_t alt_hold_pitch_high         = ALT_HOLD_PITCH_HIGH;
int16_t rtl_pitch_down              = RTL_PITCH_DOWN;
int16_t hoverthrottlemax = (int16_t)(2.0*SERVORANGE*(HOVER_THROTTLE_MAX));
int16_t hoverthrottlemin = (int16_t)(2.0*SERVORANGE*(HOVER_THROTTLE_MIN));
int16_t hoverthrottleoffset = (int16_t)((2.0*SERVORANGE*(AIRCRAFT_MASS/100))/MAX_THRUST);
float invdeltafiltertargetz = (float)(HOVER_INV_DELTA_FILTER_TARGETZ);
float invdeltafilterlidar = (float)(HOVER_INV_DELTA_FILTER_LIDAR);
float invdeltafiltersonar = (float)(HOVER_INV_DELTA_FILTER_SONAR);
float invdeltafilterbaro = (float)(HOVER_INV_DELTA_FILTER_BARO);
float invdeltafilteraccel = (float)(HOVER_INV_DELTA_FILTER_ACCEL);
int16_t hovertargetheightmin = (int16_t)(HOVER_TARGET_HEIGHT_MIN);
int16_t hovertargetheightmax = (int16_t)(HOVER_TARGET_HEIGHT_MAX);
int16_t hovertargetvzmin = (int16_t)(HOVER_TARGET_VZ_MIN);
int16_t hovertargetvzmax = (int16_t)(HOVER_TARGET_VZ_MAX);
int16_t limittargetvz = (int16_t)(HOVER_LIMIT_TARGETVZ);
int16_t limittargetaccz = (int16_t)(HOVER_LIMIT_TARGETACCZ);
uint16_t hovertargetzkp = (uint16_t)(HOVER_ZKP*COEF_SCALING);
uint16_t hovertargetzki = (uint16_t)(HOVER_ZKI*COEF_SCALING);
int32_t limitintegralz = (int32_t)(LIMIT_INTEGRAL_Z);
uint16_t hoverthrottlezkp = (uint16_t)(HOVER_ZKP*COEF_SCALING);
uint16_t hoverthrottlezki = (uint16_t)(HOVER_ZKI*COEF_SCALING);
uint16_t hoverthrottlevzkp = (uint16_t)(HOVER_VZKP*COEF_SCALING);
uint16_t hoverthrottlevzki = (uint16_t)(HOVER_VZKI*COEF_SCALING);
int32_t limitintegralvz = (int32_t)(LIMIT_INTEGRAL_VZ);
uint16_t hoverthrottleacczkp = (uint16_t)(HOVER_ACCZKP*COEF_SCALING);
uint16_t hoverthrottleacczki = (uint16_t)(HOVER_ACCZKI*COEF_SCALING);
int32_t limitintegralaccz = (int32_t)(LIMIT_INTEGRAL_ACCZ);
#else
const int16_t hoverthrottlemax = (int16_t)(2.0*SERVORANGE*(HOVER_THROTTLE_MAX));
const int16_t hoverthrottlemin = (int16_t)(2.0*SERVORANGE*(HOVER_THROTTLE_MIN));
const int16_t hoverthrottleoffset = (int16_t)(900); //(2.0*SERVORANGE*(AIRCRAFT_MASS/100))/MAX_THRUST);
const float invdeltafilterlidar = (float)(HOVER_INV_DELTA_FILTER_LIDAR);
const float invdeltafiltersonar = (float)(HOVER_INV_DELTA_FILTER_SONAR);
const float invdeltafiltertargetz = (float)(HOVER_INV_DELTA_FILTER_TARGETZ);
const float invdeltafilterbaro = (float)(HOVER_INV_DELTA_FILTER_BARO);
const float invdeltafilteraccel = (float)(HOVER_INV_DELTA_FILTER_ACCEL);
const int16_t hovertargetheightmin = (int16_t)(HOVER_TARGET_HEIGHT_MIN);
const int16_t hovertargetheightmax = (int16_t)(HOVER_TARGET_HEIGHT_MAX);
const int16_t hovertargetvzmin = (int16_t)(HOVER_TARGET_VZ_MIN);
const int16_t hovertargetvzmax = (int16_t)(HOVER_TARGET_VZ_MAX);
const uint16_t hoverthrottlezkp = (uint16_t)(HOVER_ZKP*COEF_SCALING);
const int32_t limitintegralz = (int32_t)(LIMIT_INTEGRAL_Z);
const uint16_t hoverthrottlevzkp = (uint16_t)(HOVER_VZKP*COEF_SCALING);
const int32_t limitintegralvz = (int32_t)(LIMIT_INTEGRAL_VZ);
const uint16_t hoverthrottleacczkp = (uint16_t)(HOVER_ACCZKP*COEF_SCALING);
const int32_t limitintegralaccz = (int32_t)(LIMIT_INTEGRAL_ACCZ);
#ifdef TestAltitude
    const uint16_t hoverthrottlezki = 0;
    const uint16_t hoverthrottlevzki = 0;
    const uint16_t hoverthrottleacczki = 0;
    const int16_t limittargetvz = RMAX;
    const int16_t limittargetaccz = RMAX;
#else
    const uint16_t hoverthrottlezki = (uint16_t)(HOVER_ZKI*COEF_SCALING);
    const uint16_t hoverthrottlevzki = (uint16_t)(HOVER_VZKI*COEF_SCALING);
    const uint16_t hoverthrottleacczki = (uint16_t)(HOVER_ACCZKI*COEF_SCALING);
    const int16_t limittargetvz = (int16_t)(HOVER_LIMIT_TARGETVZ);
    const int16_t limittargetaccz = (int16_t)(HOVER_LIMIT_TARGETACCZ);
#endif
#endif

int16_t error_z;
int16_t error_vz;
int16_t error_accz;
int16_t target_vz_bis;
int16_t target_accz_bis;
float z_filtered_flt=0.;
int16_t z_filtered=0;
float z_filtered32_flt=0.;
int32_t z_filtered32=0;
float z_target_filtered_flt=0.;
int16_t z_target_filtered=0;
float target_vz_filtered_flt=0.;
int16_t target_vz_filtered=0;
float vz_filtered_flt=0.;
int16_t vz_filtered=0;
float accz_filtered_flt=0.;
int16_t accz_filtered=0;
int16_t hover_error_integral_z=0;
int32_t error_integral_z=0;
int32_t error_integral_vz=0;
int32_t error_integral_accz=0;
int32_t previous_z32;
int16_t auto_landing_ramp = RMAX;
int16_t throttle_control_mem = 0;
int16_t is_not_close_to_ground_counter = 0;

float invdeltafilterheight;
float invdeltafiltervz;
float vz_imu_flt=0.;

//failsafe
boolean no_altitude_measurement = true;
int16_t no_altitude_measurement_counter = 0;
int16_t z_target_mem = -1;

#if (SPEED_CONTROL == 1)  // speed control loop

// Initialize to the value from options.h.  Allow updating this value from LOGO/MavLink/etc.
// Stored in 10ths of meters per second
int16_t desiredSpeed = (DESIRED_SPEED*10);


int32_t excess_energy_height(void) // computes (1/2gravity)*(actual_speed^2 - desired_speed^2)
{
	int16_t speedAccum = 6 * desiredSpeed;
	int32_t equivalent_energy_air_speed = -(__builtin_mulss(speedAccum, speedAccum));
	int32_t equivalent_energy_ground_speed = equivalent_energy_air_speed;
	int16_t speed_component;
	union longww accum;
	union longww forward_ground_speed;

	speed_component = IMUvelocityx._.W1 - estimatedWind[0];
	accum.WW = __builtin_mulsu (speed_component , 37877);
	equivalent_energy_air_speed += __builtin_mulss (accum._.W1 , accum._.W1);

	speed_component = IMUvelocityy._.W1 - estimatedWind[1];
	accum.WW = __builtin_mulsu (speed_component , 37877);
	equivalent_energy_air_speed += __builtin_mulss (accum._.W1 , accum._.W1);

	speed_component = IMUvelocityz._.W1 - estimatedWind[2];
	accum.WW = __builtin_mulsu (speed_component , 37877);
	equivalent_energy_air_speed += __builtin_mulss (accum._.W1 , accum._.W1);

	//	compute the projection of the ground speed in the forward direction

	forward_ground_speed.WW =((__builtin_mulss(-IMUvelocityx._.W1 , rmat[1])
	                         + __builtin_mulss(IMUvelocityy._.W1 , rmat[4]))<<2);

	//	if we are going forward, add the energy, otherwise, subract it

	accum.WW = __builtin_mulsu (forward_ground_speed._.W1 , 37877);

	if (forward_ground_speed._.W1 > 0) 
	{
		equivalent_energy_ground_speed += __builtin_mulss (accum._.W1 , accum._.W1);
	}
	else
	{
		equivalent_energy_ground_speed -= __builtin_mulss (accum._.W1 , accum._.W1);
	}

//	return the smaller of the energies of ground and air speed
//	to keep both of them from getting too small

	if (equivalent_energy_ground_speed < equivalent_energy_air_speed)
	{
		return equivalent_energy_ground_speed;
	}
	else
	{
		return equivalent_energy_air_speed;
	}

}
#else

int32_t excess_energy_height(void)
{
	return 0;
}

#if (SERIAL_OUTPUT_FORMAT == SERIAL_MAVLINK)
// Initialize to the value from options.h.  Allow updating this value from LOGO/MavLink/etc.
// Stored in 10ths of meters per second
int16_t desiredSpeed = (DESIRED_SPEED*10);
#endif // SERIAL_OUTPUT_FORMAT

#endif //(SPEED_CONTROL == 1)  // speed control loop

void reset_altitude_control(void)
{
    error_integral_z=0;
    error_integral_vz=0;
    error_integral_accz=0;
    z_filtered_flt=(float)hovertargetheightmin;
    z_filtered32_flt=(float)hovertargetheightmin;
    z_target_filtered_flt=(float)hovertargetheightmin;
    target_vz_filtered_flt=0.;
    vz_filtered_flt=0.;
    accz_filtered_flt=0.;
    previous_z32=(int32_t)(hovertargetheightmin)*100;
    no_altitude_measurement_counter = 0;
    auto_landing_ramp = RMAX;
    throttle_control_mem = 0;
    z_target_mem = -1;
}

void determine_is_close_to_ground(int16_t throttle, int16_t z)
{
#ifdef TestAltitude
    flags._.is_close_to_ground = 0;
    return;
#endif
    
    if (flags._.is_close_to_ground)
    {
        if (throttle > hoverthrottlemin)
        {
            if (z > (HOVER_TARGET_HEIGHT_MIN + ALT_SENSOR_UNCERTAINTY))
            {
                is_not_close_to_ground_counter += 1;
            }
        }
    }
    else
    {
        if (z <= (HOVER_TARGET_HEIGHT_MIN + ALT_SENSOR_UNCERTAINTY))
        {
           is_not_close_to_ground_counter -= 1;
        }
    }
    
    if (is_not_close_to_ground_counter >= (NOT_CLOSE_TO_GROUND_DETECT_TIME * HEARTBEAT_HZ))
    {
        flags._.is_close_to_ground = 0;
        is_not_close_to_ground_counter = (NOT_CLOSE_TO_GROUND_DETECT_TIME * HEARTBEAT_HZ);
        LED_GREEN = LED_ON;
    }
    if (is_not_close_to_ground_counter < 0)
    {
        flags._.is_close_to_ground = 1;
        is_not_close_to_ground_counter = 0;
        LED_GREEN = LED_OFF;
    }
}

void altitudeCntrl(void)
{
	if (current_orientation == F_HOVER)
    {
        updateAltitudeMeasurement();

        determine_is_close_to_ground(
            udb_servo_pulsesat(udb_pwIn[THROTTLE_HOVER_INPUT_CHANNEL]) - udb_servo_pulsesat(udb_pwTrim[THROTTLE_HOVER_INPUT_CHANNEL]),
            z_filtered);
        
        if (canStabilizeHover())
        {
            hoverAltitudeCntrl();
        }
	}
	else
	{
		normalAltitudeCntrl();
	}
}

void set_throttle_control(int16_t throttle)
{
	int16_t throttleIn;

	if (flags._.altitude_hold_throttle || flags._.altitude_hold_pitch || filterManual || (canStabilizeHover() && current_orientation == F_HOVER))
	{
		if (udb_flags._.radio_on == 1)
		{
			throttleIn = udb_pwIn[THROTTLE_INPUT_CHANNEL];
		}
		else
		{
			throttleIn = udb_pwTrim[THROTTLE_INPUT_CHANNEL];
		}

		int16_t temp = throttleIn + REVERSE_IF_NEEDED(THROTTLE_CHANNEL_REVERSED, throttle);

		if (THROTTLE_CHANNEL_REVERSED)
		{
			if (temp > udb_pwTrim[THROTTLE_INPUT_CHANNEL]) throttle = throttleIn - udb_pwTrim[THROTTLE_INPUT_CHANNEL];
		}
		else
		{
			if (temp < udb_pwTrim[THROTTLE_INPUT_CHANNEL]) throttle = udb_pwTrim[THROTTLE_INPUT_CHANNEL] - throttleIn;
		}

		throttle_control = throttle;
	}
	else
	{
	    throttle_control = 0;
	}
}

void set_throttle_hover_control(int16_t throttle)
{
	int16_t throttleIn;

	if (flags._.altitude_hold_throttle || flags._.altitude_hold_pitch || filterManual || (canStabilizeHover() && current_orientation == F_HOVER))
	{
		if (udb_flags._.radio_on == 1)
		{
			throttleIn = udb_pwIn[THROTTLE_HOVER_INPUT_CHANNEL];
		}
		else
		{
			throttleIn = udb_pwTrim[THROTTLE_HOVER_INPUT_CHANNEL];
		}

		int16_t temp = throttleIn + REVERSE_IF_NEEDED(THROTTLE_HOVER_CHANNEL_REVERSED, throttle);

		if (THROTTLE_HOVER_CHANNEL_REVERSED)
		{
			if (temp > udb_pwTrim[THROTTLE_HOVER_INPUT_CHANNEL]) throttle = throttleIn - udb_pwTrim[THROTTLE_HOVER_INPUT_CHANNEL];
		}
		else
		{
			if (temp < udb_pwTrim[THROTTLE_HOVER_INPUT_CHANNEL]) throttle = udb_pwTrim[THROTTLE_HOVER_INPUT_CHANNEL] - throttleIn;
		}

		throttle_hover_control = throttle;
	}
	else
	{
	    throttle_hover_control = 0;
	}
}

boolean has_no_altitude_measurement(void)
{
    if (!flags._.is_close_to_ground)
    {
	    if (no_altitude_measurement)
		{
			no_altitude_measurement_counter += 1;
		}
        else
        {
            no_altitude_measurement_counter -= 1;
        }
	}
    else
    {
        no_altitude_measurement_counter = 0;
    }
    
    if (no_altitude_measurement_counter < 0);
    {
        no_altitude_measurement_counter = 0;
    }
    
    //si cas de panne dure plus d'une seconde, on declenche la manoeuvre failsafe
	if (no_altitude_measurement_counter > (1 * HEARTBEAT_HZ)) 
    {
        return true;
    }
    else
    {
        return false;
    }
    
}

void setTargetAltitude(int16_t targetAlt)
{
	desiredHeight = targetAlt;
}

void normalAltitudeCntrl(void)
{
	union longww throttleAccum;
	union longww pitchAccum;
	int16_t throttleIn;
	int16_t throttleInOffset;
	union longww heightError = { 0 };

	speed_height = excess_energy_height(); // equivalent height of the airspeed

	if (udb_flags._.radio_on == 1)
	{
		throttleIn = udb_pwIn[THROTTLE_INPUT_CHANNEL];
		// keep the In and Trim throttle values within 2000-4000 to account for
		// Spektrum receivers using failsafe values below 2000.
		throttleInOffset = udb_servo_pulsesat(udb_pwIn[THROTTLE_INPUT_CHANNEL]) - udb_servo_pulsesat(udb_pwTrim[THROTTLE_INPUT_CHANNEL]);
	}
	else
	{
		throttleIn = udb_pwTrim[THROTTLE_INPUT_CHANNEL];
		throttleInOffset = 0;
	}

	if (flags._.altitude_hold_throttle || flags._.altitude_hold_pitch)
	{
		if (THROTTLE_CHANNEL_REVERSED) throttleInOffset = - throttleInOffset;
		
	if (flags._.GPS_steering)
		{
		if (desired_behavior._.takeoff || desired_behavior._.altitude)
			{
				desiredHeight = goal.height;
			}
			else
			{
				desiredHeight = goal.fromHeight + (((goal.height - goal.fromHeight) * (int32_t)progress_to_goal)>>12) ;
			}
		}
		else
		{
#if (ALTITUDEHOLD_STABILIZED == AH_PITCH_ONLY)
			// In stabilized mode using pitch-only altitude hold, use desiredHeight as
			// set from the state machine upon entering stabilized mode in ent_stabilizedS().
#elif (ALTITUDEHOLD_STABILIZED == AH_FULL)
			// In stabilized mode using full altitude hold, use the throttle stick value to determine desiredHeight,
			desiredHeight =((__builtin_mulss((int16_t)(HEIGHTTHROTTLEGAIN), throttleInOffset - ((int16_t)(DEADBAND)))) >> 11) 
			                + (int16_t)(HEIGHT_TARGET_MIN);
#endif
			if (desiredHeight < (int16_t)(HEIGHT_TARGET_MIN)) desiredHeight = (int16_t)(HEIGHT_TARGET_MIN);
			if (desiredHeight > (int16_t)(HEIGHT_TARGET_MAX)) desiredHeight = (int16_t)(HEIGHT_TARGET_MAX);
		}

		if (throttleInOffset < (int16_t)(DEADBAND) && udb_flags._.radio_on)
		{
			pitchAltitudeAdjust = 0;
			throttleAccum.WW  = 0;
		}
		else
		{
			heightError._.W1 = - desiredHeight;
			heightError.WW = (heightError.WW + IMUlocationz.WW + speed_height) >> 13;
			if (heightError._.W0 < (- (int16_t)(HEIGHT_MARGIN*8.0)))
			{
				throttleAccum.WW = (int16_t)(MAXTHROTTLE);
			}
			else if (heightError._.W0 > (int16_t)(HEIGHT_MARGIN*8.0))
			{
				throttleAccum.WW = 0;
			}
			else
			{
				throttleAccum.WW = (int16_t)(MAXTHROTTLE) + (__builtin_mulss((int16_t)(THROTTLEHEIGHTGAIN), (-heightError._.W0 - (int16_t)(HEIGHT_MARGIN*8.0)))>>3);
				if (throttleAccum.WW > (int16_t)(MAXTHROTTLE)) throttleAccum.WW = (int16_t)(MAXTHROTTLE);
			}

			heightError._.W1 = - desiredHeight;
			heightError.WW = (heightError.WW + IMUlocationz.WW - speed_height) >> 13;
			if (heightError._.W0 < (- (int16_t)(HEIGHT_MARGIN*8.0)))
			{
				pitchAltitudeAdjust = (int16_t)(PITCHATMAX);
			}
			else if (heightError._.W0 > (int16_t)(HEIGHT_MARGIN*8.0))
			{
				pitchAltitudeAdjust = (int16_t)(PITCHATZERO);
			}
			else
			{
				pitchAccum.WW = __builtin_mulss((int16_t)(PITCHHEIGHTGAIN) , - heightError._.W0 - (int16_t)(HEIGHT_MARGIN*8.0))>>3;
				pitchAltitudeAdjust = (int16_t)(PITCHATMAX) + pitchAccum._.W0;
			}

#if (RACING_MODE == 1)
			if (flags._.GPS_steering)
			{
				throttleAccum.WW = (int32_t)(FIXED_WP_THROTTLE);
			}
#endif
		}

		if (!flags._.altitude_hold_throttle)
		{
			manualThrottle(throttleIn);
		}
		else if (flags._.GPS_steering && desired_behavior._.land)
		{
			// place a ceiling, in other words, go down, but not up.
			if (pitchAltitudeAdjust > 0)
			{
				pitchAltitudeAdjust = 0;
			}
			
			throttleFiltered.WW += (((int32_t)(udb_pwTrim[THROTTLE_INPUT_CHANNEL] - throttleFiltered._.W1))<<THROTTLEFILTSHIFT);
			set_throttle_control(throttleFiltered._.W1 - throttleIn);
			filterManual = true;
		}
		else
		{
			// Servo reversing is handled in servoMix.c
			int16_t throttleOut = udb_servo_pulsesat(udb_pwTrim[THROTTLE_INPUT_CHANNEL] + throttleAccum.WW);
			throttleFiltered.WW += (((int32_t)(throttleOut - throttleFiltered._.W1)) << THROTTLEFILTSHIFT);
			set_throttle_control(throttleFiltered._.W1 - throttleIn);
			filterManual = true;
		}
		
		if (!flags._.altitude_hold_pitch)
		{
			pitchAltitudeAdjust = 0;
		}
	}
	else
	{
		pitchAltitudeAdjust = 0;
		manualThrottle(throttleIn);
	}
}

void manualThrottle(int16_t throttleIn)
{
	int16_t throttle_control_pre;

	throttleFiltered.WW += (((int32_t)(throttleIn - throttleFiltered._.W1)) << THROTTLEFILTSHIFT);

	if (filterManual) {
		// Continue to filter the throttle control value in manual mode to avoid large, instant
		// changes to throttle value, which can burn out a brushed motor.  But after fading over
		// to the new throttle value, stop applying the filter to the throttle out to allow
		// faster control.
		throttle_control_pre = throttleFiltered._.W1 - throttleIn;
		if (throttle_control_pre < 10) filterManual = false;
	}
	else {
		throttle_control_pre = 0;
	}

	set_throttle_control(throttle_control_pre);
}

void manualHoverThrottle(int16_t throttleIn)
{
	int16_t throttle_control_pre;

	throttleFiltered.WW += (((int32_t)(throttleIn - throttleFiltered._.W1)) << THROTTLEFILTSHIFT);

	if (filterManual) {
		// Continue to filter the throttle control value in manual mode to avoid large, instant
		// changes to throttle value, which can burn out a brushed motor.  But after fading over
		// to the new throttle value, stop applying the filter to the throttle out to allow
		// faster control.
		throttle_control_pre = throttleFiltered._.W1 - throttleIn;
		if (throttle_control_pre < 10) filterManual = false;
	}
	else {
		throttle_control_pre = 0;
	}

	set_throttle_hover_control(throttle_control_pre);
}

int16_t compute_vz_alt_sensor(int16_t z)
{
    int32_t z32 = (int32_t)(z)*100;
	z_filtered32 = exponential_filter32(z32, &z_filtered32_flt, invdeltafilterheight);
    int32_t vz_alt_sensor32=(z_filtered32-previous_z32)*((int32_t)(HEARTBEAT_HZ));
	previous_z32=z_filtered32;
    return (int16_t)(vz_alt_sensor32/100);
}

void updateAltitudeMeasurement(void)
{
    int16_t z;
    int16_t vz;
    int16_t accz;
    no_altitude_measurement=true;
    invdeltafilterheight=(float)(HEARTBEAT_HZ);
    invdeltafiltervz=(float)(HEARTBEAT_HZ);
    
    //by default use IMU altitude and velocity
    z=100*IMUlocationz._.W1+50;
    vz=IMUvelocityz._.W1;
    //z acceleration is provided by accelerometers
    accz=accelEarth[2];
    
    //determine height to ground (in cm) according to available sensor and validity
    
    //if barometer is used, use its measurement
#if ( BAROMETER_ALTITUDE == 1 )
    
    estBaroAltitude();
    
    if (udb_flags._.baro_valid && !flags._.is_close_to_ground)
    {
#if (SILSIM != 1)
        z=(int16_t)(get_barometer_altitude());
#endif
        invdeltafilterheight=invdeltafilterbaro;
		invdeltafiltervz=8.;
        //if you do not trust barometer altitude for altitude control, set no_altitude_measurement to true, otherwise set to false
        no_altitude_measurement=false;
        setFailureSonarDist(z);
    }
#endif

    //if sonar is used, use sonar altitude
#if ( USE_SONAR == 1 )

    calculate_sonar_height_above_ground();

	if (udb_flags._.sonar_height_valid && !flags._.is_close_to_ground)
	{
        z=sonar_height_to_ground;
        invdeltafilterheight=invdeltafiltersonar;
        invdeltafiltervz=8.;
		no_altitude_measurement=false;
        setFailureLidarDist(z);
	}
#endif
    
    //if lidar is used, use lidar altitude (higher priority in order to accurately control landing)
#if ( USE_LIDAR == 1 )

    calculate_lidar_height_above_ground();

	if (udb_flags._.lidar_height_valid)
	{
        z=lidar_height_to_ground;
        invdeltafilterheight=invdeltafilterlidar;
        invdeltafiltervz=40.;
		no_altitude_measurement=false;
	}
#endif
      
#if (HILSIM == 1 && SILSIM == 0)
    //in HILSIM mode, replace z from sonar or barometer by GPS altitude (GPSlocation.z is in meters)
    z = IMUlocationz._.W1*100+50;
#endif
    
    z_filtered = exponential_filter(z, &z_filtered_flt, invdeltafilterheight);
    
#ifdef TestAltitude
    vz_filtered = 0;
    accz_filtered = 0;
#else
    if (!no_altitude_measurement)
    {
        //if sonar or barometer is valid, compute vz as the derivative in time of filtered z,
        //the VZ_CORR parameter allows to mix the altitude sensor derivative with the IMU vz
        //VZ_CORR=1 means only altitude sensor derivative is considered
        int16_t vz_avg = exponential_filter(IMUvelocityz._.W1, &vz_imu_flt, 1);
        vz = (__builtin_mulsu(IMUvelocityz._.W1 - vz_avg, RMAX - VZ_CORR_16)
                    + __builtin_mulsu(compute_vz_alt_sensor(z_filtered), VZ_CORR_16))>>14;
    }
    vz_filtered = exponential_filter(vz, &vz_filtered_flt, invdeltafiltervz);
    accz_filtered = exponential_filter(accz, &accz_filtered_flt, invdeltafilteraccel);
#endif
    
}

void hoverAltitudeCntrl(void)
{
    int16_t z_target;
    int16_t vz_target;
    int16_t target_vz;
    int16_t target_accz;
    int16_t throttle;
    int16_t throttle_control_pre;
    
	no_altitude_measurement=true;
    pitchAltitudeAdjust = 0;
    desiredHeight = 0;
    throttle_control_pre = 0;

    //compute target altitude and smooth with exponential filtering

    z_target = 0;
    vz_target = 0;

    if (flags._.GPS_steering)
    {
        //GPS mode
        z_target = goal.fromHeight + (((goal.height - goal.fromHeight) * (int32_t)progress_to_goal)>>12) ;

        if ((goal.height - goal.fromHeight) > 0)
        { 
            vz_target = hovertargetvzmax;
        }
        else
        {
            vz_target = hovertargetvzmin;
        }
    }
    else
    {
        z_target = compute_pot_order(udb_pwIn[INPUT_CHANNEL_AUX2], hovertargetheightmin, hovertargetheightmax);  
    }

    z_target_filtered = exponential_filter(z_target, &z_target_filtered_flt, invdeltafiltertargetz);
        
    //***************************************************//
    //PI controller on height to ground z
    error_z=z_filtered-z_target_filtered;
    target_vz=compute_pi_block(z_filtered, z_target_filtered, hoverthrottlezkp, hoverthrottlezki, &error_integral_z, 
                                    (int16_t)(HEARTBEAT_HZ), limitintegralz, !flags._.is_close_to_ground);
    target_vz_bis=limit_value(target_vz*COEF_MAX, -limittargetvz, limittargetvz);
    //***************************************************//

    //***************************************************//
    //PI controller on vertical velocity
    error_vz=vz_filtered-target_vz_bis;
    target_accz=compute_pi_block(vz_filtered, target_vz_bis, hoverthrottlevzkp, hoverthrottlevzki, &error_integral_vz, 
                                  (int16_t)(HEARTBEAT_HZ), limitintegralvz, !flags._.is_close_to_ground);
    target_accz_bis=limit_value(target_accz*COEF_MAX, -limittargetaccz, limittargetaccz);
    //***************************************************//

    //***************************************************//
    //PI controller on vertical acceleration
    error_accz=accz_filtered-target_accz_bis;
    throttle=compute_pi_block(accz_filtered, target_accz_bis, hoverthrottleacczkp, hoverthrottleacczki, &error_integral_accz, 
                               (int16_t)(HEARTBEAT_HZ), limitintegralaccz, !flags._.is_close_to_ground);
    throttle = throttle*COEF_MAX;
    
    if (current_flight_phase == F_MANUAL_TAKE_OFF)
    {
        throttle_control_pre = 0;
    }
    else
    {
        if (has_no_altitude_measurement())
        {
            throttle_control_pre = hoverthrottleoffset;
        }
        else
        {
            if (current_flight_phase == F_AUTO_LAND)
            {
                apply_ramp(&auto_landing_ramp, RAMPE_DECREMENT, 0, RMAX);
                throttle_control_pre = (int16_t)(__builtin_mulsu(throttle_control_mem-hoverthrottlemin, auto_landing_ramp)>>14)+hoverthrottlemin ;
            }
            else
            {
                throttle_control_pre=throttle;
                //***************************************************//

                //apply coefficient to make PI gains independent of aircraft mass and maximum propeller thrust
                //int32_t tmp=__builtin_mulsu(throttle_control_pre, 2*aircraft_mass);
                //throttle_control_pre=(int16_t)((tmp/100)/max_thrust);

                //add throttle offset
                throttle_control_pre+=hoverthrottleoffset;
                throttle_control_mem = throttle_control_pre;
            }
            
            //limit throttle value
            throttle_control_pre=limit_value(throttle_control_pre, hoverthrottlemin, hoverthrottlemax);
        }
    }

    //throttleFiltered.WW += (((int32_t)(throttleIn - throttleFiltered._.W1)) << THROTTLEFILTSHIFT);

    //if (filterManual) {
        // Continue to filter the throttle control value in manual mode to avoid large, instant
        // changes to throttle value, which can burn out a brushed motor.  But after fading over
        // to the new throttle value, stop applying the filter to the throttle out to allow
        // faster control.
        //throttle_control_pre = throttleFiltered._.W1 - throttleIn;
        //if (throttle_control_pre < 10) filterManual = false;
    //}
    //else {
    //	throttle_control_pre = 0;
    //}

    set_throttle_hover_control(throttle_control_pre);
}








#endif //(ALTITUDE_GAINS_VARIABLE != 1)




        //DEBUG: generation of a sinusoidal signal imposed as throttle. Goal is to observe corresponding 
        //sonar height, z velocity and z acceleration: it will allow to determine:
        // 1. if acceleration signal is meaningful
        // 2. responsiveness of the propulsion set to throttle order
        // 2. estimate the proportionnal gains
//        int8_t angle;
//        int16_t angle_bis;
//        int16_t synthetic_throttle;
//        int16_t sin;
//        int16_t amplitude=250;
//        int16_t frequency=1;
//        int16_t k =(int16_t)(frequency*((256*SCALE_GAIN)/HEARTBEAT_HZ)); //max frequency is (heartbeat/5) Hz (5 points per period)
//        
//        angle_bis=k*synthetic_throttle_counter;
//
//        if (angle_bis >= (256*SCALE_GAIN))
//        {
//            synthetic_throttle_counter=0;
//        }
//
//        angle=(int8_t)(angle_bis/SCALE_GAIN);
//        if (angle_bis >= (128*SCALE_GAIN))
//        {
//            angle=(int8_t)(angle_bis/SCALE_GAIN)-256;
//        }
//         
//        sin=sine(angle);
//        synthetic_throttle=(int16_t)(sin/(16384/amplitude));
//        synthetic_throttle_counter+=1;

        //throttle_control_pre+=synthetic_throttle;


//Kalman Filter

            //int32_t raw_mesure[] = {(int32_t)(-IMUvelocityz._.W1), (int32_t)(heightIn-targetHeight)};
            //int32_t raw_mesure[] = {0, (int32_t)(-targetHeight)};
            
//		    //	% Prédiction
//		    //	Xp = A*X;
//		    matrix22_vector_multiply(&xp_vect , a_mat , x_vect);
//
//		    //	Pp = A*P*A'+Q;
//            //tmp_mat=A*P
//		    matrix22_multiply(&tmp_mat , a_mat , p_mat);
//            //mat_transpose=A'
//		    matrix22_transpose(&mat_transpose, a_mat);	
//            //tmp_mat=tmp_mat*A'
//		    matrix22_multiply(&tmp_mat , tmp_mat , mat_transpose);
//            //	Pp = A*P*A'+Q;
//		    matrix22_add(&pp_mat, tmp_mat, q_mat);
//	    
//		    //	% Mise à jour    
//		    //	K = Pp*H'*inv(R+H*Pp*H');
//		    //tmp=H*Pp
//		    matrix22_multiply(&tmp_mat , h_mat , pp_mat);
//		    //mat_transpose=H'
//		    matrix22_transpose(&mat_transpose, h_mat);
//		    //tmp_mat=H*Pp*H'
//		    matrix22_multiply(&tmp_mat , tmp_mat , mat_transpose);
//		    //tmp_mat=R+H*Pp*H'
//		    matrix22_add(&tmp_mat, r_mat, tmp_mat);
//		    //tmp2_mat=Pp*H'
//		    matrix22_multiply(&tmp2_mat , pp_mat , mat_transpose);
//		    //K = Pp*H'*inv
//		    int32_t det = tmp_mat[0]*tmp_mat[3]-tmp_mat[1]*tmp_mat[2];
//            k_mat[0]=(int32_t)((tmp2_mat[0]*tmp_mat[3]-tmp2_mat[1]*tmp_mat[2])/det);
//		    k_mat[1]=(int32_t)((-tmp2_mat[0]*tmp_mat[1]+tmp2_mat[1]*tmp_mat[0])/det);
//            k_mat[2]=(int32_t)((tmp2_mat[2]*tmp_mat[3]-tmp2_mat[3]*tmp_mat[2])/det);
//            k_mat[3]=(int32_t)((-tmp2_mat[2]*tmp_mat[1]+tmp2_mat[3]*tmp_mat[0])/det);
//
//		    //	P = Pp - K*H*Pp;
//            //tmp_mat=K*H
//            matrix22_multiply(&tmp_mat , k_mat , h_mat);
//            //tmp_mat=tmp_mat*Pp
//            matrix22_multiply(&tmp_mat , tmp_mat , pp_mat);
//            //	P = Pp - K*H*Pp;
//            matrix22_sub(&p_mat, pp_mat, tmp_mat);
//
//		    //	X = Xp + K*(mesure(run, :)'-H*Xp);
//            //tmp_vect=H*Xp
//            matrix22_vector_multiply(&tmp_vect , h_mat , xp_vect);
//            //tmp2_vect=SCL*mesure-H*Xp
//            tmp2_vect[0]=(int32_t)(SCL)*raw_mesure[0]-tmp_vect[0];
//            tmp2_vect[1]=(int32_t)(SCL)*raw_mesure[1]-tmp_vect[1];
//            //tmp_vect=K*tmp2_vect
//            matrix22_vector_multiply(&tmp_vect , k_mat , tmp2_vect);
//            //	X = Xp + K*(mesure(run, :)'-H*Xp);
//            x_vect[0]=xp_vect[0]+tmp_vect[0];
//            x_vect[1]=xp_vect[1]+tmp_vect[1];
//
//            x_vect[0]=(int32_t)(x_vect[0]/SCL);
//            x_vect[1]=(int32_t)(x_vect[1]/SCL);
//
//            p_mat[0]=(int32_t)(p_mat[0]/(SCL*SCL));
//            p_mat[1]=(int32_t)(p_mat[1]/(SCL*SCL));
//            p_mat[2]=(int32_t)(p_mat[2]/(SCL*SCL));
//            p_mat[3]=(int32_t)(p_mat[3]/(SCL*SCL));
//
//            hover_state_zpoint=(int16_t)(x_vect[0]);
//            hover_state_z=(int16_t)(x_vect[1]);
