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
#include "airspeed_options.h"

int16_t current_orientation;
int16_t current_flight_phase;
union bfbts_word desired_behavior;
int16_t cyclesUntilStartTriggerAction = 0;
int16_t cyclesUntilStopTriggerAction = 0;
boolean currentTriggerActionValue = 0;
int16_t minimum_airspeed = MINIMUM_AIRSPEED * 100;
int16_t pulse_duration = TRIGGER_PULSE_DURATION;
int16_t pulse_period = TRIGGER_REPEAT_PERIOD;
int16_t trigger_counts = 0;
int16_t trigger_duration = 0;


void apply_ramp(int16_t *climb, int16_t increment, int16_t min_value, int16_t max_value)
{
    *climb += increment;
    *climb = limit_value(*climb, min_value, max_value);
}

void triggerActionSetValue(boolean newValue);

void init_flight_phase(void)
{
    current_flight_phase = F_MANUAL_TAKE_OFF;
}

void init_behavior(void)
{
	current_orientation = F_NORMAL;
	desired_behavior.W = current_orientation;

	setBehavior(current_orientation);
}

void setTriggerParams(int16_t pulse_duration, int16_t pulse_period)
{
    pulse_duration = pulse_duration;
    pulse_period = pulse_period;
}

void activateTrigger(int16_t duration)
{
    trigger_duration = duration;
}

void computeTriggerActivation()
{
    if (TRIGGER_TYPE != TRIGGER_TYPE_NONE)
    {
        if (trigger_duration > 0)
        {
            trigger_counts = trigger_duration / (int32_t)(1000/(SERVO_HZ));
        }
    }
    
    if(trigger_counts > 0)
    {
        setBehavior(F_TRIGGER);
        trigger_counts--;
        trigger_duration = 0;
    }
    else
    {
        setBehavior(F_NORMAL);
        triggerActionSetValue(0);
    }
}

void setBehavior(int16_t newBehavior)
{
	desired_behavior.W = newBehavior;

	if (desired_behavior.W & F_TRIGGER)
	{
		if (cyclesUntilStartTriggerAction == 0)
		{
			cyclesUntilStartTriggerAction = 1;
		}
	}
	else
	{
		cyclesUntilStartTriggerAction = 0;
	}
}

boolean canStabilizeInverted(void)
{
	return ((INVERTED_FLIGHT_STABILIZED_MODE && (flags._.pitch_feedback && !flags._.GPS_steering)) ||
	        (INVERTED_FLIGHT_WAYPOINT_MODE && (flags._.pitch_feedback && flags._.GPS_steering)));
}

boolean canStabilizeHover(void)
{
	return (HOVERING_STABILIZED_MODE && flags._.pitch_feedback);
}

void updateBehavior(void)
{          
	//in cm/s
	uint16_t horizontal_air_speed = vector2_mag(IMUvelocityx._.W1 - estimatedWind[0], 
	                                   IMUvelocityy._.W1 - estimatedWind[1]);

	int16_t current_altitude = IMUlocationz._.W1;
	int16_t transition_altitude = TRANSITION_ALTITUDE;
    boolean isHovering = HOVERING_STABILIZED_MODE && (horizontal_air_speed <= minimum_airspeed || current_altitude <= transition_altitude);

	if (current_orientation == F_INVERTED)
	{
		if (HOVERING_STABILIZED_MODE && rmat[7] < -14000)
		{
			current_orientation = F_HOVER;
		}
		else if (canStabilizeInverted() && rmat[8] < 6000)
		{
			current_orientation = F_INVERTED;
		}
		else
		{
			current_orientation = F_NORMAL;
		}
	}
	else if (current_orientation == F_HOVER)
	{
		//remain in hovering mode if (horizontal_air_speed < minimum_airspeed or altitude < MAX_HOVERING_ALTITUDE)
		if (isHovering)
		{
			current_orientation = F_HOVER;
		}
		else if (canStabilizeInverted() && rmat[8] < -6000)
		{
			current_orientation = F_INVERTED;
//			reset_manoeuvre();
		}
		else
		{
			current_orientation = F_NORMAL;
		}
	}
	else
	{
		if (canStabilizeInverted() && rmat[8] < -6000)
		{
			current_orientation = F_INVERTED;
		}
		//switch from normal to hovering if (horizontal_air_speed <= minimum_airspeed horizontal_air_speed < minimum_airspeed or altitude < MAX_HOVERING_ALTITUDE)
		else if (isHovering)
		{
			current_orientation = F_HOVER;
		}
		else
		{
			current_orientation = F_NORMAL;
		}
	}

    if (flags._.pitch_feedback) 
	{
		desired_behavior.W = current_orientation;
	}
	dcm_enable_yaw_drift_correction(1);
	
}

// This function is called every 25ms
void updateTriggerAction(void)
{
    computeTriggerActivation();
            
	if (cyclesUntilStopTriggerAction == 1)
	{
		triggerActionSetValue(TRIGGER_ACTION != TRIGGER_PULSE_HIGH);
		cyclesUntilStopTriggerAction = 0;
	}
	else if (cyclesUntilStopTriggerAction > 0)
	{
		cyclesUntilStopTriggerAction--;
	}
	if (cyclesUntilStartTriggerAction == 1 && (desired_behavior.W & F_TRIGGER))
	{
		if (TRIGGER_ACTION == TRIGGER_PULSE_HIGH || TRIGGER_ACTION == TRIGGER_PULSE_LOW)
		{
			triggerActionSetValue(TRIGGER_ACTION == TRIGGER_PULSE_HIGH);

			cyclesUntilStopTriggerAction = pulse_duration / (int32_t)(1000/(SERVO_HZ));
			cyclesUntilStartTriggerAction = 0;
		}
		else if (TRIGGER_ACTION == TRIGGER_TOGGLE)
		{
			triggerActionSetValue(!currentTriggerActionValue);

			cyclesUntilStopTriggerAction = 0;
			cyclesUntilStartTriggerAction = 0;
		}
		else if (TRIGGER_ACTION == TRIGGER_REPEATING)
		{
			triggerActionSetValue(TRIGGER_ACTION == TRIGGER_PULSE_HIGH);

			cyclesUntilStopTriggerAction = pulse_duration / (int32_t)(1000/(SERVO_HZ));
			cyclesUntilStartTriggerAction = pulse_period / (int32_t)(1000/(SERVO_HZ));
		}
	}
	else if (cyclesUntilStartTriggerAction > 0)
	{
		cyclesUntilStartTriggerAction--;
	}
}

void triggerActionSetValue(boolean newValue)
{
    additional_int16_export1 = (int16_t)(newValue);
    
	if (TRIGGER_TYPE == TRIGGER_TYPE_SERVO)
	{
		udb_pwOut[TRIGGER_OUTPUT_CHANNEL] = (newValue) ? TRIGGER_SERVO_HIGH : TRIGGER_SERVO_LOW;
	}
	else if (TRIGGER_TYPE == TRIGGER_TYPE_DIGITAL)
	{
		udb_set_action_state(newValue);
	}
	currentTriggerActionValue = newValue;
}

void updateFlightPhase()
{           
    int16_t throttle = udb_servo_pulsesat(udb_pwIn[THROTTLE_HOVER_INPUT_CHANNEL]) - udb_servo_pulsesat(udb_pwTrim[THROTTLE_HOVER_INPUT_CHANNEL]);
    
#ifdef TestAltitude
    flags._.engines_off = 1;
    flags._.is_close_to_ground = 0;
#endif
    
    if ((z_filtered > (int16_t)(HOVER_FAILSAFE_ALTITUDE)) || flags._.low_battery)
    {
        //if max altitude is exceeded, reduce throttle
    }
    
    if (current_flight_phase == F_MANUAL_TAKE_OFF)
    {
        if (!flags._.is_close_to_ground)
        {
            current_flight_phase = F_IS_IN_FLIGHT;
            LED_BLUE = LED_ON;
            setTriggerParams(1000, 500);
            activateTrigger(2000);
        }
        else
        {
            current_flight_phase = F_MANUAL_TAKE_OFF;
            LED_BLUE = LED_OFF;
        }
    }
    else if (current_flight_phase == F_IS_IN_FLIGHT)
    {
        if (canStabilizeHover() && flags._.is_close_to_ground)
        {
            current_flight_phase = F_AUTO_LAND;
            LED_BLUE = LED_OFF;
            LED_ORANGE = LED_ON;
            setTriggerParams(1000, 500);
            activateTrigger(3000);
        }
        else if (throttle < (int16_t)(2.0*SERVORANGE*(HOVER_THROTTLE_MIN)) && flags._.is_close_to_ground)
        {
            current_flight_phase = F_MANUAL_TAKE_OFF;
            LED_BLUE = LED_OFF;
            setTriggerParams(1000, 500);
            activateTrigger(1000);
            reset_altitude_control();
        }
        else
        {
            current_flight_phase = F_IS_IN_FLIGHT;
            LED_BLUE = LED_ON;
        }
    }
    else if (current_flight_phase == F_AUTO_LAND)
    {
        if (canStabilizeHover() && auto_landing_ramp <= 0)
        {
            current_flight_phase = F_ENGINE_OFF;
            LED_ORANGE = LED_OFF;
            setTriggerParams(1000, 500);
            activateTrigger(4000);
            reset_altitude_control();
        }
        else if (!canStabilizeHover())
        {
            current_flight_phase = F_IS_IN_FLIGHT;
            LED_BLUE = LED_ON;
            setTriggerParams(1000, 500);
            activateTrigger(2000);
        }
        else
        {
            current_flight_phase = F_AUTO_LAND;
            LED_ORANGE = LED_ON;
        }
    }
    else
    {        
        if (throttle < (int16_t)(2.0*SERVORANGE*(HOVER_THROTTLE_MIN)))
        {
            current_flight_phase = F_MANUAL_TAKE_OFF;
            setTriggerParams(1000, 500);
            activateTrigger(1000);
            flags._.engines_off = 0;
        }
        else
        {
            current_flight_phase = F_ENGINE_OFF;
            flags._.engines_off = 1;
        }
    }
}
