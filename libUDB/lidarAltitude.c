/*
 * File:   lidarAltitude.c
 * Author: Seb
 *
 * Created on 1 janvier 2018, 18:45
 */


#include "defines.h"
#include "lidarAltitude.h"

#if ( USE_LIDAR == 1 )

//uint16_t udb_pwm_lidar = 0;
int16_t lidar_distance ;          // distance to target in centimeter
int16_t lidar_height_to_ground ; // calculated distance to ground in Earth's Z Plane allowing for tilt
int16_t failure_lidar_distance = OUT_OF_RANGE_DISTANCE ;
unsigned char lidar_good_sample_count  = 0 ;  // Tracks the number of consequtive good samples up until SONAR_SAMPLE_THRESHOLD is reached.
unsigned char lidar_no_readings_count  = 0 ;  // Tracks number of UDB frames since last sonar reading was sent by sonar device
#define NO_READING_RECEIVED_DISTANCE			9999 // Distance denotes that no sonar reading was returned from sonar device
#define LIDAR_SAMPLE_THRESHOLD 					   1 // Number of readings before code deems "certain" of a true reading.

void setFailureLidarDist(int16_t distance)
{
    failure_lidar_distance = distance ;
}
    

void calculate_lidar_height_above_ground(void)
{
    lidar_distance = OUT_OF_RANGE_DISTANCE;

    if ( udb_flags._.lidar_updated == 1 ) 
	{	
		lidar_no_readings_count  = 0 ;
    
#if ( USE_LIDAR_ON_PWM_INPUT_8	== 1 )
        //PWM input
        //should be : udb_pwm_lidar (in micro sec) / 10. Here there must be a scaling * 2 of the pwm pulse duration, so we divide by 20
        //udb_pwm_lidar is an unsigned int, so it range between 0 to 65535. So lidar distance can range between 0 to 65535/20=3276cm = 32.76m.
        lidar_distance = (int16_t)(udb_pwm_lidar / 20) ;
#endif
        
        if ( lidar_distance > USEABLE_LIDAR_DISTANCE || lidar_distance < LIDAR_MINIMUM_DISTANCE )
		{
			lidar_height_to_ground = failure_lidar_distance ;
			lidar_good_sample_count = 0 ; 
            udb_flags._.lidar_height_valid = 0;
#if (LED_ORANGE_SONAR_CHECK == 1)
			LED_ORANGE = LED_ON;
#endif
		}
		else 
		{
			lidar_good_sample_count++ ;
			if  (lidar_good_sample_count > LIDAR_SAMPLE_THRESHOLD) 
			{
				lidar_good_sample_count = LIDAR_SAMPLE_THRESHOLD ;
                lidar_height_to_ground = lidar_distance;
                udb_flags._.lidar_height_valid = 1;
#if (LED_ORANGE_SONAR_CHECK == 1)
				LED_ORANGE = LED_OFF;
#endif
			}
			else
			{
				lidar_height_to_ground = failure_lidar_distance ;
                udb_flags._.lidar_height_valid = 0;
#if (LED_ORANGE_SONAR_CHECK == 1)
				LED_ORANGE = LED_ON;
#endif
			}
		}
        
        udb_flags._.lidar_updated = 0 ;
	}  
    else
	{
		if ( lidar_no_readings_count < 7 )
		{
		 	lidar_no_readings_count++ ;
		}
		else
		{
	    	lidar_height_to_ground = NO_READING_RECEIVED_DISTANCE ;
			udb_flags._.lidar_height_valid = 0;
#if (LED_ORANGE_SONAR_CHECK == 1)
			LED_ORANGE = LED_ON;
#endif
            setTriggerParams(SENSOR_FAILURE_PULSE_PERIOD, SENSOR_FAILURE_PULSE_DURATION);
            activateTrigger(SENSOR_FAILURE_PULSE_PERIOD);
		}
	}
            
    return;
}
#endif
