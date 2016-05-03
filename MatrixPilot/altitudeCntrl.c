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
#include "../libDCM/estAltitude.h"
#include "../libUDB/barometer.h"
#include "../libDCM/libDCM_internal.h"

#if (ALTITUDE_GAINS_VARIABLE != 1)

union longww throttleFiltered = { 0 };

#define THROTTLEFILTSHIFT   12

#define DEADBAND            150
#define DEADBAND_HOVER      300

#define MAXTHROTTLE         (2.0*SERVORANGE*ALT_HOLD_THROTTLE_MAX)
#define FIXED_WP_THROTTLE   (2.0*SERVORANGE*RACING_MODE_WP_THROTTLE)

#define THROTTLEHEIGHTGAIN  (((ALT_HOLD_THROTTLE_MAX - ALT_HOLD_THROTTLE_MIN)*2.0*SERVORANGE)/(HEIGHT_MARGIN*2.0))

#define PITCHATMAX          (ALT_HOLD_PITCH_MAX*(RMAX/57.3))
#define PITCHATMIN          (ALT_HOLD_PITCH_MIN*(RMAX/57.3))
#define PITCHATZERO         (ALT_HOLD_PITCH_HIGH*(RMAX/57.3))
#define PITCHHEIGHTGAIN     ((PITCHATMAX - PITCHATMIN) / (HEIGHT_MARGIN*2.0))

#define HEIGHTTHROTTLEGAIN  ((1.5*(HEIGHT_TARGET_MAX-HEIGHT_TARGET_MIN)* 1024.0) / (SERVORANGE*SERVOSAT))

#define HOVER_FAILSAFE_ALTITUDE 10000
#define COEF_MAX            10  //maximum value of pid coef
#define COEF_SCALING        (RMAX/COEF_MAX)
#define VZ_CORR_16           VZ_CORR*RMAX

int16_t aircraft_mass       = AIRCRAFT_MASS;
int16_t max_thrust          = MAX_THRUST;

int16_t nb_sample_wait = 80;

int32_t speed_height = 0;
int16_t pitchAltitudeAdjust = 0;
boolean filterManual = false;

int16_t desiredHeight;

void normalAltitudeCntrl(void);
void manualThrottle(int16_t throttleIn);
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
int16_t hoverthrottleoffset = (int16_t)(2.0*SERVORANGE*(HOVER_THROTTLE_OFFSET));
float invdeltafiltertargetz = (float)(HOVER_INV_DELTA_FILTER_TARGETZ);
float invdeltafiltersonar = (float)(HOVER_INV_DELTA_FILTER_SONAR);
float invdeltafilterbaro = (float)(HOVER_INV_DELTA_FILTER_BARO);
float invdeltafilteraccel = (float)(HOVER_INV_DELTA_FILTER_ACCEL);
int16_t hoverwaitacczstable = 0;  //in nb of 1/FE s
int16_t hovertargetheightmin = (int16_t)(HOVER_TARGET_HEIGHT_MIN);
int16_t hovertargetheightmax = (int16_t)(HOVER_TARGET_HEIGHT_MAX);
int16_t hovertargetvzmin = (int16_t)(HOVER_TARGET_VZ_MIN);
int16_t hovertargetvzmax = (int16_t)(HOVER_TARGET_VZ_MAX);
int16_t limittargetvz = (int16_t)(HOVER_LIMIT_TARGETVZ);
int16_t limittargetaccz = (int16_t)(HOVER_LIMIT_TARGETACCZ);
uint16_t hovertargetzkp = (uint16_t)(HOVER_ZKP*COEF_SCALING);
uint16_t hovertargetzki = (uint16_t)(HOVER_ZKI*COEF_SCALING);
uint16_t hoverthrottlevzkp = (uint16_t)(HOVER_VZKP*COEF_SCALING);
uint16_t hoverthrottlevzki = (uint16_t)(HOVER_VZKI*COEF_SCALING);
uint16_t hoverthrottleacczkp = (uint16_t)(HOVER_ACCZKP*COEF_SCALING);
uint16_t hoverthrottleacczki = (uint16_t)(HOVER_ACCZKI*COEF_SCALING);
#else
const int16_t hoverthrottlemax = (int16_t)(2.0*SERVORANGE*(HOVER_THROTTLE_MAX));
const int16_t hoverthrottlemin = (int16_t)(2.0*SERVORANGE*(HOVER_THROTTLE_MIN));
const int16_t hoverthrottleoffset = (int16_t)(2.0*SERVORANGE*(HOVER_THROTTLE_OFFSET));
const float invdeltafiltersonar = (float)(HOVER_INV_DELTA_FILTER_SONAR);
const float invdeltafiltertargetz = (float)(HOVER_INV_DELTA_FILTER_TARGETZ);
const float invdeltafilterbaro = (float)(HOVER_INV_DELTA_FILTER_BARO);
const float invdeltafilteraccel = (float)(HOVER_INV_DELTA_FILTER_ACCEL);
const int16_t hoverwaitacczstable = 0;  //in nb of 1/FE s
const int16_t hovertargetheightmin = (int16_t)(HOVER_TARGET_HEIGHT_MIN);
const int16_t hovertargetheightmax = (int16_t)(HOVER_TARGET_HEIGHT_MAX);
const int16_t hovertargetvzmin = (int16_t)(HOVER_TARGET_VZ_MIN);
const int16_t hovertargetvzmax = (int16_t)(HOVER_TARGET_VZ_MAX);
const int16_t limittargetvz = (int16_t)(HOVER_LIMIT_TARGETVZ);
const int16_t limittargetaccz = (int16_t)(HOVER_LIMIT_TARGETACCZ);
const uint16_t hoverthrottlezkp = (uint16_t)(HOVER_ZKP*COEF_SCALING);
const uint16_t hoverthrottlezki = (uint16_t)(HOVER_ZKI*COEF_SCALING);
const uint16_t hoverthrottlevzkp = (uint16_t)(HOVER_VZKP*COEF_SCALING);
const uint16_t hoverthrottlevzki = (uint16_t)(HOVER_VZKI*COEF_SCALING);
const uint16_t hoverthrottleacczkp = (uint16_t)(HOVER_ACCZKP*COEF_SCALING);
const uint16_t hoverthrottleacczki = (uint16_t)(HOVER_ACCZKI*COEF_SCALING);
#endif

int16_t hover_counter=0;
int16_t synthetic_throttle_counter=0;
float z_filtered_flt=0.;
int16_t z_filtered=0;
float z_filtered32_flt=0.;
int32_t z_filtered32=0;
float target_z_filtered_flt=0.;
int16_t target_z_filtered=0;
float target_vz_filtered_flt=0.;
int16_t target_vz_filtered=0;
float vz_filtered_flt=0.;
int16_t vz_filtered=0;
float accz_filtered_flt=0.;
int16_t accz_filtered=0;
int16_t hover_error_integral_z=0;
int16_t error_integral_z=0;
int16_t error_integral_vz=0;
int16_t error_integral_accz=0;
int16_t previous_z;
int32_t previous_z32;
boolean failsafe_throttle_mode = false;
boolean alt_sensor_failure = false;
int16_t min_hover_alt = (int16_t)(HOVER_ALTITUDE_MIN);
int16_t max_hover_alt = (int16_t)(HOVER_FAILSAFE_ALTITUDE);

int16_t hover_target_z=0;
int16_t hover_z=0;
int16_t hover_vz=0;
int16_t hover_accz=0;
int16_t hover_error_z=0;
int16_t hover_error_integral_z;
int16_t hover_error_vz=0;
int16_t hover_error_integral_vz;
int16_t hover_error_accz=0;
int16_t hover_target_vz=0;
int16_t hover_target_accz=0;

#if ( USE_SONAR == 1 )
   int16_t sonar_distance ;          // distance to target in centimeters
   int16_t sonar_height_to_ground ; // calculated distance to ground in Earth's Z Plane allowing for tilt
   unsigned char good_sample_count  = 0 ;  // Tracks the number of consequtive good samples up until SONAR_SAMPLE_THRESHOLD is reached.
//   fractional cos_pitch_roll ;  // tilt of the plane in UDB fractional units * 2.
   void calculate_sonar_height_above_ground(void);
#endif

//% Initialisation de Kalman
//#define SCL 40

////H = [1 0; 0 1];
//int32_t h_mat[] = {1, 0, 0, 1};
////R = [bruit_capteur(1)^2 0; 0 bruit_capteur(2)^2];
//int32_t r_mat[] = {(int32_t)(SCL*SCL)*12*12, 0, 0, (int32_t)(SCL*SCL)*7*7};
////A = fe*[1 0; 1/fe 1];
//int32_t a_mat[] = {(int32_t)(SCL), 0, (int32_t)(SCL/FE), (int32_t)(SCL)};
////Q = eye(2) * 1;
////Q(1, 1) = 20;
////Q(2, 2) = 0;
//int32_t q_mat[] = {(int32_t)(SCL*SCL*20), 0, 0, 0};
//
////X={zPoint, z}
////X = zeros(2, 1);
////X(1) = 0;
////X(2) = 0;
//int32_t x_vect[] = {0, 0};
//
////P = zeros(2, 2);
////P(1, 1) = 0;
////P(2, 2) = 0;
////P(1, 2) = 0;
////P(2, 1) = P(1, 2);
//int32_t p_mat[] = {0, 0, 0, 0};
//
//int32_t xp_vect[] = {0, 0};
//int32_t pp_mat[] = {0, 0, 0, 0};
//int32_t k_mat[] = {0, 0, 0, 0};
//int32_t tmp_mat[] = {0, 0, 0, 0};
//int32_t tmp2_mat[] = {0, 0, 0, 0};
//int32_t tmp_vect[] = {0, 0};
//int32_t tmp2_vect[] = {0, 0};
//int32_t mat_transpose[] = {0, 0, 0, 0};
//int32_t mat_inv[] = {0, 0, 0, 0};



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

void altitudeCntrl(void)
{
	if (canStabilizeHover() && current_orientation == F_HOVER)
	{
		hoverAltitudeCntrl();
	}
	else
	{
        //hoverAltitudeCntrl();
        hover_counter=0;
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

void hoverAltitudeCntrl(void)
{
    int16_t throttleIn = (udb_flags._.radio_on == 1) ? udb_pwIn[THROTTLE_INPUT_CHANNEL] : udb_pwTrim[THROTTLE_INPUT_CHANNEL];
    int16_t throttleInOffset = (udb_flags._.radio_on == 1) ? udb_servo_pulsesat(udb_pwIn[THROTTLE_INPUT_CHANNEL]) - udb_servo_pulsesat(udb_pwTrim[THROTTLE_INPUT_CHANNEL]) : 0;

    int16_t z_target;
    int16_t vz_target;
    int16_t z;
    int16_t error_z;
    int16_t target_vz;
    int16_t target_vz_bis;
    int16_t vz;
    int16_t error_vz;
    int16_t accz;
    int16_t target_accz;
    int16_t target_accz_bis;
    int16_t error_accz;
    int16_t throttle;
    float invdeltafilterheight;
    float invdeltafilterheight32;
    float invdeltafiltervz;

    pitchAltitudeAdjust = 0;
    desiredHeight = 0;
    int16_t throttle_control_pre = 0;


    //initialize filtered and integral quantities

    if (hover_counter==0)
    {
        error_integral_z=0;
        error_integral_vz=0;
        error_integral_accz=0;
        z_filtered_flt=(float)hovertargetheightmin;
        z_filtered32_flt=(float)hovertargetheightmin;
        target_z_filtered_flt=(float)hovertargetheightmin;
        target_vz_filtered_flt=0.;
        vz_filtered_flt=0.;
        accz_filtered_flt=0.;
        previous_z=hovertargetheightmin;
    }

    if (hover_counter < RMAX)
    {
        hover_counter+=1;
    }

    //determine height to ground (in cm) according to available sensor and validity, and scale

    //by default use IMU altitude and velocity
    z=100*IMUlocationz._.W1+50;
    invdeltafilterheight=(float)(HEARTBEAT_HZ);
    invdeltafilterheight32=(float)(HEARTBEAT_HZ);
    invdeltafiltervz=(float)(HEARTBEAT_HZ);
    vz=IMUvelocityz._.W1;

    alt_sensor_failure=true;

    //if barometer is used, use its measurement
#if ( BAROMETER_ALTITUDE == 1 )
    if (is_barometer_valid())
    {
        estAltitude();
        z=(int16_t)(get_barometer_altitude());
        invdeltafilterheight=invdeltafilterbaro;
        invdeltafilterheight32=2.;
        invdeltafiltervz=2.;
        alt_sensor_failure=false;
    }
#endif

    //if sonar is used, use sonar altitude (higher priority in order to accurately control landing)
#if ( USE_SONAR == 1 )

	calculate_sonar_height_above_ground();
	if (udb_flags._.sonar_height_valid)
	{
        z=sonar_height_to_ground;
        invdeltafilterheight=invdeltafiltersonar;
        invdeltafilterheight32=5.;
        invdeltafiltervz=4.;
        alt_sensor_failure=false;
	}
#endif
    
    //z acceleration is provided by accelerometers
    accz=accelEarth[2];

    //compute target altitude and smooth with exponential filtering

    z_target = 0;
    vz_target = 0;

if (flags._.GPS_steering)
{
    //a remplacer par desiredHeight et desiredVelocity issus des waypoints
    z_target = compute_target_alt();
    vz_target = compute_target_vz();
}
else
{
    int16_t tmp1 = udb_pwIn[FLAP_INPUT_CHANNEL] - 2233;
    tmp1=limit_value(tmp1, 0, 3823-2233);

#if (MANUAL_TARGET_HEIGHT == 1)

    int32_t tmp2 = __builtin_mulss(hovertargetheightmax-hovertargetheightmin, tmp1);
    z_target = (int16_t)(tmp2/(3823-2233))+hovertargetheightmin;  

#else

    int32_t tmp2 = __builtin_mulss(hovertargetvzmax-hovertargetvzmin, tmp1);
    vz_target = (int16_t)(tmp2/(3823-2233))+hovertargetvzmin;

#endif // end MANUAL_TARGET_HEIGHT
}

#if (HILSIM == 1)
//in HILSIM mode, replace z from sonar or barometer by GPS altitude (GPSlocation.z is in meters)
    z = IMUlocationz._.W1*100+50;
#endif

    //wait for z, vz and accz to be stable before computing filtering
    target_z_filtered = exponential_filter(z_target, &target_z_filtered_flt, invdeltafiltertargetz, (int16_t)(HEARTBEAT_HZ));
    target_vz_filtered = exponential_filter(vz_target, &target_vz_filtered_flt, invdeltafiltertargetz, (int16_t)(HEARTBEAT_HZ));

    z_filtered = exponential_filter(z, &z_filtered_flt, invdeltafilterheight, (int16_t)(HEARTBEAT_HZ));

    //if sonar or barometer is valid, compute vz as the derivative in time of filtered z,
    //the VZ_CORR parameter allows to mix the altitude sensor derivative with the IMU vz
    //VZ_CORR=1 means only altitude sensor derivative is considered

    if (alt_sensor_failure == false)
    {
        z_filtered32 = exponential_filter32(100*z, &z_filtered32_flt, invdeltafilterheight32, (int16_t)(HEARTBEAT_HZ));

        int32_t vz_alt_sensor32=(z_filtered32-previous_z32)*((int32_t)(HEARTBEAT_HZ));
        int16_t vz_alt_sensor=(int16_t)(vz_alt_sensor32/100);
        vz = (__builtin_mulsu(vz, RMAX - VZ_CORR_16) 
                    + __builtin_mulsu(vz_alt_sensor, VZ_CORR_16))>>14;
        //vz=vz_alt_sensor;
        previous_z32=z_filtered32;
    }

    vz_filtered = exponential_filter(vz, &vz_filtered_flt, invdeltafiltervz, (int16_t)(HEARTBEAT_HZ));
    accz_filtered = exponential_filter(accz, &accz_filtered_flt, invdeltafilteraccel, (int16_t)(HEARTBEAT_HZ));

    //***************************************************//
    //PI controller on height to ground z
    error_z=z_filtered-target_z_filtered;
    if (MANUAL_TARGET_HEIGHT == 1 || is_target_alt())
	{ 
        target_vz=compute_pid_block(z_filtered, target_z_filtered, hoverthrottlezkp, hoverthrottlezki, &error_integral_z, 
                                    (int16_t)(HEARTBEAT_HZ), (hover_counter > nb_sample_wait));
        target_vz_bis=limit_value(target_vz*COEF_MAX, -limittargetvz, limittargetvz);
    }
    else
    {
        target_vz_bis=target_vz_filtered;
    }

    //limit error_integral to avoid exceeding +/-16384
    error_integral_z=limit_value(error_integral_z, -RMAX, RMAX);

    //***************************************************//

    //***************************************************//
    //PI controller on vertical velocity
    error_vz=vz_filtered-target_vz_bis;
    target_accz=compute_pid_block(vz_filtered, target_vz_bis, hoverthrottlevzkp, hoverthrottlevzki, &error_integral_vz, 
                                  (int16_t)(HEARTBEAT_HZ), (hover_counter > nb_sample_wait));
    error_integral_vz=limit_value(error_integral_vz, -RMAX, RMAX);       
    target_accz_bis=limit_value(target_accz*COEF_MAX, -limittargetaccz, limittargetaccz);
    //***************************************************//

    //***************************************************//
    //PI controller on vertical acceleration
    error_accz=accz_filtered-target_accz_bis;
    throttle=compute_pid_block(accz_filtered, target_accz_bis, hoverthrottleacczkp, hoverthrottleacczki, &error_integral_accz, 
                               (int16_t)(HEARTBEAT_HZ), (hover_counter > nb_sample_wait));
    error_integral_accz=limit_value(error_integral_accz, -RMAX, RMAX);
    throttle_control_pre=throttle*COEF_MAX;
    //***************************************************//

    //apply coefficient to make PI gains independent of aircraft mass and maximum propeller thrust
    int32_t tmp=__builtin_mulsu(throttle_control_pre, 2*aircraft_mass);
    throttle_control_pre=(int16_t)((tmp/100)/max_thrust);

    //add throttle offset
    throttle_control_pre+=hoverthrottleoffset;

    if (alt_sensor_failure || z < min_hover_alt)
	{
        throttle_control_pre=hoverthrottleoffset;
        //later, use sinusoidal throttle command to visualize failsafe
    }

    if (z > (int16_t)(1.1*max_hover_alt))  //if max altitude is exceeded by 10%, reduce throttle
	{
        //20% of max throttle
        throttle_control_pre=(int16_t)(2.0*SERVORANGE*(0.2));
    }

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

    //limit throttle value
    throttle_control_pre=limit_value(throttle_control_pre, hoverthrottlemin, hoverthrottlemax);

    previous_z = z_filtered;

    //set variables for SERIAL_UDB_EXTRA log
    hover_z=z_filtered;
    hover_vz=vz_filtered;
    hover_accz=accz_filtered;
    hover_target_z=target_z_filtered;
    hover_error_z=error_z;
    hover_error_integral_z=(int16_t)(error_integral_z/(int16_t)(HEARTBEAT_HZ));
    hover_error_vz=error_vz;
    hover_error_integral_vz=(int16_t)(error_integral_vz/(int16_t)(HEARTBEAT_HZ));
    hover_target_vz=target_vz_bis;
    hover_target_accz=target_accz_bis;
    
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

    set_throttle_control(throttle_control_pre);

    //in case of panick, retrieve manual control as soon as throttle stick is pushed forward (higher priority) 
    if (throttleInOffset > (int16_t)(DEADBAND_HOVER))
    {
        manualThrottle(throttleIn);
    }

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

}

#if ( USE_SONAR == 1 )

// USEABLE_SONAR_DISTANCE may well vary with type of ground cover (e.g. long grass may be less).
// Pete Hollands ran the code with #define SERIAL_OUTPUT SERIAL_UDB_SONAR while flying low
// over his landing area, which was a freshly cut straw field. Post flight, he anlaysed the CSV telemetry into a spreadsheet graph,
// and determined that all measurements below 4 meters were true, as long as there were at least 3 consecutive measurements,
// that were less than 4 meters (400 centimeters).
#define SONAR_MINIMUM_DISTANCE                   90 // Normally, should be minimum possible sonar distance measurement (4 inch)
                                                     //here specifically for the SBACH 342, the sonar sees the elevator and returns 78cm.
                                                     //so we discard this value by setting to 90cm the minimum valid distance
#define OUT_OF_RANGE_DISTANCE          			 550 // Distance in centimeters that denotes "out of range" for your Sonar device.
#define NO_READING_RECEIVED_DISTANCE			9999 // Distance denotes that no sonar reading was returned from sonar device
#define SONAR_SAMPLE_THRESHOLD 					  3 // Number of readings before code deems "certain" of a true reading.
#define UDB_SONAR_PWM_UNITS_TO_CENTIMETERS       278  // 

uint16_t udb_pwm_sonar;
unsigned char no_readings_count  = 0 ;  // Tracks number of UDB frames since last sonar reading was sent by sonar device
int16_t distance_yaw_corr = 0;  //correction of sonar distance in cm, to account for yaw angle of the plane

void calculate_sonar_height_above_ground(void)
{
	if ( udb_flags._.sonar_updated == 1 ) 
	{	
		union longbbbb accum ;
		no_readings_count  = 0 ;

        accum.WW = __builtin_muluu( udb_pwm_sonar , UDB_SONAR_PWM_UNITS_TO_CENTIMETERS ) ;
		sonar_distance = accum._.W1 << 1 ;

        //analog input on analog input 1
        //sonar_distance = (int16_t)(udb_analogInputs[0].input/73 + 440);

		// RMAT 7 is the cosine of the tilt of the plane in pitch with respect to vertical axis (z)	;
//		cos_pitch_roll = rmat[7] ;
//		if ( cos_pitch_roll > 16383 )
//		{
//			cos_pitch_roll = 16383 ;
//		}
		if ( sonar_distance > USEABLE_SONAR_DISTANCE || sonar_distance < SONAR_MINIMUM_DISTANCE )
		{
			sonar_height_to_ground = OUT_OF_RANGE_DISTANCE ;
			good_sample_count = 0 ; 
            udb_flags._.sonar_height_valid = 0;
		}
		else 
		{
			good_sample_count++ ;
			if  (good_sample_count > SONAR_SAMPLE_THRESHOLD) 
			{
				good_sample_count = SONAR_SAMPLE_THRESHOLD ;
                //approximation of tan(yaw) = yaw
                distance_yaw_corr = __builtin_mulsu(rmat[6], HALF_SPAN)>>14;
//				accum.WW = __builtin_mulss(cos_pitch_roll, sonar_distance) ;
//				sonar_height_to_ground = -accum._.W1 << 2 ; 
                sonar_height_to_ground = sonar_distance + distance_yaw_corr;
                udb_flags._.sonar_height_valid = 1;
                //additional_int16_export5 = rmat[6];
	            //additional_int16_export3 = distance_yaw_corr;
			}
			else
			{
				sonar_height_to_ground = OUT_OF_RANGE_DISTANCE ;
                udb_flags._.sonar_height_valid = 0;
			}
		}
		udb_flags._.sonar_updated = 0 ;
		udb_flags._.sonar_print_telemetry = 1 ;
	}
	else
	{
		if ( no_readings_count < 7 ) // This assumes runnig at 40HZ UDB frame rate
		{
		 	no_readings_count++ ;
		}
		else
		{
	    	sonar_height_to_ground = NO_READING_RECEIVED_DISTANCE ;
		}
	}
	return ;
}
#endif


#endif //(ALTITUDE_GAINS_VARIABLE != 1)

