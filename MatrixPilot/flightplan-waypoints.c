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
#include "../libDCM/libDCM_internal.h"

//struct manoeuvreDef {int16_t channel; int32_t start_time; int32_t end_time; int16_t value; };

//struct vertical_segment { int16_t vz; int16_t alt_start; int16_t alt_end; int32_t duration; 
//                          boolean is_target_alt; int16_t flags; struct manoeuvreDef* manoeuvres; };
//
//int16_t current_manoeuvreValues[NUM_OUTPUTS];  


#if (FLIGHT_PLAN_TYPE == FP_WAYPOINTS)

struct relWaypointDef { struct relative3D loc; int16_t flags; struct relative3D viewpoint; };
struct waypointDef { struct waypoint3D loc; int16_t flags; struct waypoint3D viewpoint; };

#include "waypoints.h"

#if (SERIAL_OUTPUT_FORMAT == SERIAL_MAVLINK)
uint16_t  number_of_waypoints = ((sizeof waypoints) / sizeof (struct waypointDef));
#endif
#define NUMBER_POINTS ((sizeof waypoints) / sizeof (struct waypointDef))
#define NUMBER_RTL_POINTS ((sizeof rtlWaypoints) / sizeof (struct waypointDef))

int16_t waypointIndex = 0;

struct waypointDef *currentWaypointSet = (struct waypointDef*)waypoints;
int16_t numPointsInCurrentSet = NUMBER_POINTS;

struct waypointDef wp_inject;
uint8_t wp_inject_pos = 0;
#define WP_INJECT_READY 255
const uint8_t wp_inject_byte_order[] = {3, 2, 1, 0, 7, 6, 5, 4, 9, 8, 11, 10, 15, 14, 13, 12, 19, 18, 17, 16, 21, 20 };

//void setManoeuvre(struct manoeuvreDef* manoeuvres, int32_t relative_time)
//{
//    if (manoeuvres != NULL)
//    {
//        int8_t i;
//        flags._.manoeuvre=1;
//
//        for (i = 0 ; i < (sizeof manoeuvres); i++)
//        {
//            if ( relative_time > manoeuvres[i].start_time && relative_time < manoeuvres[i].end_time )
//            {
//                current_manoeuvreValues[manoeuvres[i].channel] = manoeuvres[i].value;
//            }
//        }    
//    }
//}
//
//void reset_manoeuvre()
//{
//    int8_t i;
//    flags._.manoeuvre = 0;
//
//    for (i = 0 ; i < (sizeof current_manoeuvreValues); i++)
//    {
//        current_manoeuvreValues[i] = -RMAX;
//    }
//}

// For a relative waypoint, wp_to_relative() just passes the relative
// waypoint location through unchanged.
// For an absolute waypoint, wp_to_relative() converts the waypoint's
// location from absolute to relative.
struct relWaypointDef wp_to_relative(struct waypointDef wp)
{
	struct relWaypointDef rel;
	
	if (wp.flags & F_ABSOLUTE)
	{
		rel.loc = dcm_absolute_to_relative(wp.loc);
		rel.viewpoint = dcm_absolute_to_relative(wp.viewpoint);
		
		rel.flags = wp.flags - F_ABSOLUTE;
	}
	else
	{
		rel.loc.x = wp.loc.x;
		rel.loc.y = wp.loc.y;
		rel.loc.z = wp.loc.z;
		
		rel.viewpoint.x = wp.viewpoint.x;
		rel.viewpoint.y = wp.viewpoint.y;
		rel.viewpoint.z = wp.viewpoint.z;
		
		rel.flags = wp.flags;
	}
	return rel;
}


// In the future, we could include more than 2 waypoint sets...
// flightplanNum is 0 for main waypoints, and 1 for RTL waypoints
void init_flightplan (int16_t flightplanNum)
{
	if (flightplanNum == 1) // RTL waypoint set
	{
		currentWaypointSet = (struct waypointDef*)rtlWaypoints;
		numPointsInCurrentSet = NUMBER_RTL_POINTS;
	}
	else if (flightplanNum == 0) // Main waypoint set
	{
		currentWaypointSet = (struct waypointDef*)waypoints;
		numPointsInCurrentSet = NUMBER_POINTS;
	}
	
	waypointIndex = 0;
	struct relWaypointDef current_waypoint = wp_to_relative(currentWaypointSet[0]);
	set_goal(GPSlocation , current_waypoint.loc);
	set_camera_view(current_waypoint.viewpoint);
	setBehavior(current_waypoint.flags);
	
	// udb_background_trigger();			// trigger navigation immediately
}

boolean use_fixed_origin(void)
{
#if (USE_FIXED_ORIGIN == 1)
	return 1;
#else
	return 0;
#endif
}

struct absolute3D get_fixed_origin(void)
{
	struct fixedOrigin3D origin = FIXED_ORIGIN_LOCATION;
	
	struct absolute3D standardizedOrigin;
	standardizedOrigin.x = origin.x;
	standardizedOrigin.y = origin.y;
	standardizedOrigin.z = (int32_t)(origin.z * 100);
	
	return standardizedOrigin;
}

void next_waypoint (void) 
{
	waypointIndex++;
	
	if (waypointIndex >= numPointsInCurrentSet) waypointIndex = 0;
	
	if (waypointIndex == 0)
	{
		if (numPointsInCurrentSet > 1)
		{
			struct relWaypointDef previous_waypoint = wp_to_relative(currentWaypointSet[numPointsInCurrentSet-1]);
			struct relWaypointDef current_waypoint  = wp_to_relative(currentWaypointSet[0]);
			set_goal(previous_waypoint.loc, current_waypoint.loc);
			set_camera_view(current_waypoint.viewpoint);
		}
		else
		{
			struct relWaypointDef current_waypoint = wp_to_relative(currentWaypointSet[0]);
			set_goal(GPSlocation, current_waypoint.loc);
			set_camera_view(current_waypoint.viewpoint);
		}
		setBehavior(currentWaypointSet[0].flags);
	}
	else
	{
		struct relWaypointDef previous_waypoint = wp_to_relative(currentWaypointSet[waypointIndex-1]);
		struct relWaypointDef current_waypoint = wp_to_relative(currentWaypointSet[waypointIndex]);
		set_goal(previous_waypoint.loc, current_waypoint.loc);
		set_camera_view(current_waypoint.viewpoint);
		setBehavior(current_waypoint.flags);
	}
	
#if	(DEADRECKONING == 0)
	compute_bearing_to_goal();
#endif
}

void run_flightplan(void)
{
	// first run any injected wp from the serial port
	if (wp_inject_pos == WP_INJECT_READY)
	{
		struct relWaypointDef current_waypoint = wp_to_relative(wp_inject);
		set_goal(GPSlocation, current_waypoint.loc);
		set_camera_view(current_waypoint.viewpoint);
		setBehavior(current_waypoint.flags);
		compute_bearing_to_goal();
		wp_inject_pos = 0;
	}
	
	// steering is based on cross track error.
 	// waypoint arrival is detected computing distance to the "finish line".
	
	// note: locations are measured in meters
	//		 velocities are in centimeters per second
	
	// locations have a range of +-32000 meters (20 miles) from origin
	
	if (desired_behavior._.altitude)
	{
#if (SILSIM == 1)
		if (std::abs(IMUheight - goal.height) < ((int16_t) HEIGHT_MARGIN))
#else
        if (abs(IMUheight - goal.height) < ((int16_t) HEIGHT_MARGIN))
#endif
			next_waypoint();
	}
	else
	{
		if (tofinish_line < WAYPOINT_RADIUS) // crossed the finish line
		{
			if (desired_behavior._.loiter)
				set_goal(GPSlocation, wp_to_relative(currentWaypointSet[waypointIndex]).loc);
			else
				next_waypoint();
		}
	}
}

void flightplan_live_begin(void)
{
	wp_inject_pos = 0;
}

void flightplan_live_received_byte(uint8_t inbyte)
{
	if (wp_inject_pos < sizeof(wp_inject_byte_order))
	{
		((uint8_t*)(&wp_inject))[wp_inject_byte_order[wp_inject_pos++]] = inbyte;
	}
	else if (wp_inject_pos == sizeof(wp_inject_byte_order))
	{
		wp_inject_pos++;
	}
}

void flightplan_live_commit(void)
{
	if (wp_inject_pos == sizeof(wp_inject_byte_order))
	{
		wp_inject_pos = WP_INJECT_READY;
	}
	else
	{
		wp_inject_pos = 0;
	}
}

#endif


//compute target altitude based on the user defined vertical segments

//#define NUMBER_SEGMENTS ((sizeof take_off_segment) / sizeof (struct vertical_segment))
//struct vertical_segment standby_segment = {0,0,0,INT32_MAX,1,0};
//struct vertical_segment current_segment;
//struct relative3D originPoint = {0, 0, 0};
//int16_t segmentIndex=0;
//int16_t current_altitude=0;
//int32_t init_time=0;
//int32_t start_time;
//unsigned char initialized_time=0;

//boolean is_terminated(int32_t time, int16_t current_altitude)
//{
//    if (current_segment.duration == 0)
//    {
//        if((current_segment.vz > 0 && current_altitude > current_segment.alt_end)
//              || (current_segment.vz < 0 && current_altitude  < current_segment.alt_end))
//        {
//            setBehavior(current_segment.flags);
//            return true;
//        }
//        else
//        {
//            return false;
//        }
//    }
//    else
//    {
//        if((time - start_time) > current_segment.duration)
//        {
//            setBehavior(current_segment.flags);
//            return true;
//        }
//        else
//        {
//            return false;
//        }
//    }
//}

//void next_segment(int32_t time, int16_t current_altitude) 
//{	
//    reset_manoeuvre();
//    set_goal(GPSlocation, originPoint);
//
//	if (segmentIndex >= NUMBER_SEGMENTS)
//    {
//        standby_segment.alt_start=current_altitude;
//        start_time = time;
//        current_segment=standby_segment;
//    }
//    else
//    {
//        current_segment=take_off_segment[segmentIndex];
//        start_time = time;
//    }
//
//    segmentIndex++;
//}
//
//boolean is_target_alt(void)
//{
//    return current_segment.is_target_alt;
//}
//
//int16_t compute_target_vz(void)
//{
//    return current_segment.vz;
//}
//
//int16_t compute_target_alt(void)
//{
//    return current_altitude;
//}
//
//void run_vertical_segments(void)
//{
//    int32_t time = tow.WW;
//
//    if (initialized_time==0)
//    {
//         init_time=time;
//         current_segment=take_off_segment[0];
//         start_time = 0;
//         segmentIndex++;
//         initialized_time=1;   
//    }
//
//    struct relative3D imu_location = { 0 , 0 , 0 };
//    imu_location.x = IMUlocationx._.W1;
//    imu_location.y = IMUlocationy._.W1;
//    imu_location.z = IMUlocationz._.W1;
//    set_goal(imu_location, originPoint);
//    compute_bearing_to_goal();
//
//    if (is_terminated(time-init_time, current_altitude))
//    {
//        next_segment(time-init_time, current_altitude);
//    }
//
//    setManoeuvre(current_segment.manoeuvres, time - init_time - start_time);
//    
//    current_altitude = (int16_t)(((int32_t)(current_segment.vz) * (time - init_time - start_time))/1000) + current_segment.alt_start;
//}


