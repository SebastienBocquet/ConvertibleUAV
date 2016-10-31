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

const int max_tilt = (int) (MAX_TILT*.7111) ;  // maximum tilt in byte cicular
int commanded_tilt_gain ;

#if (TEST == 1)
#include <assert.h>
#endif

#if (USE_TELELOG == 1)
#include "telemetry_log.h"
#endif

#if (USE_USB == 1)
#include "preflight.h"
#endif

#if (USE_CONFIGFILE == 1)
#include "config.h"
#endif

//	main program for testing the IMU.

#if (SILSIM == 1)
int mp_argc;
char **mp_argv;
int main(int argc, char** argv)
{
	// keep these values available for later
	mp_argc = argc;
	mp_argv = argv;
#else
int main(void)
{
	commanded_tilt_gain = sine ( max_tilt ) / 1000 ;
	mcu_init();
#endif
#if (USE_TELELOG == 1)
	log_init();
#endif
#if (USE_USB == 1)
	preflight();
#endif
	udb_init();
	dcm_init();
#if (USE_CONFIGFILE == 1)
	init_config();
#endif
	init_servoPrepare();
	init_states();
	init_behavior();
	init_serial();

#if (TEST == 1)

    void test_expfilter()
    {
        printf("exponential_filter\n");
        int16_t x = 2;
        float x_filtered = 2.0;
        int16_t result = exponential_filter(x, &x_filtered, 80., 80);
        printf("result x_filtered %d %f\n", result, x_filtered);
	    assert(result == 2);
        assert(x_filtered == 2.0);
        printf("exponential_filter PASSED\n");
    }
	
	void debug_hovering_roll_control()
	{
	
	//debugging of hoverRollCntrl

    int16_t i;

    //set inputs

    //activate stabilized mode
    flags._.pitch_feedback = 1;

    //90° pitch angle
    rmat[8] = 0; 

    //target roll angle  = 0°
    udb_pwIn[FLAP_INPUT_CHANNEL] = 3028;

    printf("rmat2 rmat5 rmat8 rmat6 rmat6_128 rollNavDeflection, rollNavDeflection_filtered, rollAngle, rollAngle_filtered, angle_delta, roll_corr, roll_control\n");

    printf("Simulate a 180°/-180° turn around vertical axis\n");
    rmat[5] = 1;
    for(i = 0; i > -16385; i=i-2048)
    {
        rmat[2] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
    rmat[5] = -1;
    for(i = -16384; i < 1; i=i+2048)
    {
        rmat[2] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
    rmat[5] = -1;
    for(i = 0; i > -16385; i=i-2048)
    {
        rmat[2] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
    rmat[5] = 1;
    for(i = -16384; i < 1; i=i+2048)
    {
        rmat[2] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
    rmat[5] = 1;
    for(i = 0; i < 16385; i=i+2048)
    {
        rmat[2] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
    rmat[5] = -1;
    for(i = 16384; i > -1; i=i-2048)
    {
        rmat[2] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
    rmat[5] = -1;
    for(i = 0; i < 16385; i=i+2048)
    {
        rmat[2] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
    rmat[5] = 1;
    for(i = 16385; i > -1; i=i-2048)
    {
        rmat[2] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
    printf("################\n");
    printf("Simulate pitch angle varying from vertical to horizontal at 45° roll angle\n");
    rmat[5] = 1;
    //45° roll angle in vertical position
    rmat[2] = 8192;
    //45° roll angle in horizontal position
    rmat[6] = 8192;
    //pitch angle varying from vertical to horizontal
    for(i = 0; i < 16385; i=i+2048)
    {
        rmat[8] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
    printf("################\n");
    printf("Simulate roll angle variation at 45° pitch for target roll angle = 180°\n");
    rmat[5] = -1;
    //45° pitch angle
    rmat[8] = 8192;
    rmat[6] = 0;
    udb_pwIn[FLAP_INPUT_CHANNEL] = 3823;
    for(i = -16384; i < 16385; i=i+2048)
    {
        rmat[2] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
    rmat[2] = 0;
    for(i = -16384; i < 16385; i=i+2048)
    {
        rmat[6] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
    printf("Simulate roll angle variation at 45° pitch for target roll angle = -180°\n");
    //45° pitch angle
    rmat[8] = 8192;
    rmat[6] = 0;
    udb_pwIn[FLAP_INPUT_CHANNEL] = 2233;
    for(i = -16384; i < 16385; i=i+2048)
    {
        rmat[2] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
    rmat[2] = 0;
    for(i = -16384; i < 16385; i=i+2048)
    {
        rmat[6] = i;
        //compute
        hoverRollCntrl();
    }

    printf("################\n");
    printf("################\n");
    printf("Simulate roll angle variation at 90° pitch for target roll angle = 180°\n");
    //target roll angle  = 180°
    udb_pwIn[FLAP_INPUT_CHANNEL] = 3823;

    //90° pitch angle
    rmat[8] = 0; 
    rmat[6] = 0;

    rmat[5] = 1;
    for(i = 0; i > -16385; i=i-2048)
    {
        rmat[2] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
    rmat[5] = -1;
    for(i = -16384; i < 1; i=i+2048)
    {
        rmat[2] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
    rmat[5] = -1;
    for(i = 0; i > -16385; i=i-2048)
    {
        rmat[2] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
    rmat[5] = 1;
    for(i = -16384; i < 1; i=i+2048)
    {
        rmat[2] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");

    printf("Simulate roll angle variation at 90° pitch for target roll angle = -180°\n");
    //target roll angle  = -180°
    udb_pwIn[FLAP_INPUT_CHANNEL] = 2233;

    rmat[5] = 1;
    for(i = 0; i < 16385; i=i+2048)
    {
        rmat[2] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
    rmat[5] = -1;
    for(i = 16384; i > -1; i=i-2048)
    {
        rmat[2] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
    rmat[5] = -1;
    for(i = 0; i < 16385; i=i+2048)
    {
        rmat[2] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
    rmat[5] = 1;
    for(i = 16385; i > -1; i=i-2048)
    {
        rmat[2] = i;
        //compute
        hoverRollCntrl();
    }
    printf("################\n");
	}
	
	// void write_debug_file()
	// {
	   // FILE* fichier = NULL;
		// fichier = fopen("test.txt", "w");

	   // int8_t test;
	   // for(int i = 0; i < 300; i=i+1)
	   // {
		   // test = (int8_t)i;
		   // fprintf(fichier, "%d %d\n", i, test);
	   // }

	   // fclose(fichier);
	   // system("pause");
	// }
	
	void test_hover_roll_control_1()
	{
        //test that for vertical position with IMU roll angle = 0, the roll control is 0

	    //overwrite inputs
        rmat[8] = 0;
		rmat[6] = 0;
		rmat[2] = 0;
        udb_pwIn[FLAP_INPUT_CHANNEL] = 2233;
	    
        //execute
        hoverRollCntrl();

        //output
        printf("roll_control %d\n", roll_control);
	    assert(roll_control == 0);
        printf("hover_roll_control_1 PASSED\n");   
	}

    void test_hover_roll_control_2()
	{
        //test that in hovering horizontal position, rmat2 has no effect

	    //overwrite inputs
        rmat[8] = 16384; //pitch angle
		rmat[6] = 0; //roll angle in horizontal position
        udb_pwIn[FLAP_INPUT_CHANNEL] = 3028;
    	//activate stabilized mode
    	flags._.pitch_feedback = 1;

		int16_t delta_control = 0;
		int16_t previous_control = 0;
        int16_t i;

		for(i = 0; i < 16384; i=i+128)
    	{
			rmat[2] = i;  //roll angle in vertical position
        	//execute
        	hoverRollCntrl();	 
			printf("rmat2 roll_control delta_control %d %d %d\n", rmat[2], roll_control);
			assert(roll_control == 0);
		}

        printf("hover_roll_control_2 PASSED\n");   
	}

	void test_hover_roll_control_3()
	{
        //test that in hovering horizontal position, for increasing rmat6, roll control decreases

	    //overwrite inputs
        rmat[8] = 16384; //pitch angle
		rmat[2] = 0; //roll angle in vertical position
        udb_pwIn[FLAP_INPUT_CHANNEL] = 3028;
    	//activate stabilized mode
    	flags._.pitch_feedback = 1;

		int16_t delta_control = 0;
		int16_t previous_control = 0;
        int16_t i;

		for(i = 0; i < 16384; i=i+128)
    	{
			rmat[6] = i;  //roll angle in vertical position
        	//execute
        	hoverRollCntrl();	 
            delta_control = roll_control - previous_control;
            previous_control = roll_control;
			printf("rmat6 roll_control delta_control %d %d %d\n", rmat[6], roll_control, delta_control);
			assert(delta_control >= 0);
		}

        printf("hover_roll_control_3 PASSED\n");   
	}

	void test_hover_roll_control_4()
	{
        //test that in hovering vertical position, for increasing rmat2, roll control decreases

	    //overwrite inputs
        rmat[8] = 0; //pitch angle
		rmat[2] = 0; //roll angle in vertical position
        udb_pwIn[FLAP_INPUT_CHANNEL] = 3028;
    	//activate stabilized mode
    	flags._.pitch_feedback = 1;

		int16_t delta_control = 0;
		int16_t previous_control = 0;
        int16_t i;

	    printf("Simulate a 180°/-180° turn around vertical axis\n");
	    rmat[5] = 1;
	    for(i = 0; i > -16385; i=i-2048)
	    {
	        rmat[2] = i;
	        //compute
	        hoverRollCntrl();
			delta_control = roll_control - previous_control;
            previous_control = roll_control;
			printf("rmat6 roll_control delta_control %d %d %d\n", rmat[2], roll_control, delta_control);
			assert(delta_control >= 0);
	    }
	    printf("################\n");
	    rmat[5] = -1;
		previous_control = 0;
	    for(i = -16384; i < 1; i=i+2048)
	    {
	        rmat[2] = i;
	        //compute
	        hoverRollCntrl();
			delta_control = roll_control - previous_control;
            previous_control = roll_control;
			printf("rmat6 roll_control delta_control %d %d %d\n", rmat[2], roll_control, delta_control);
			assert(delta_control >= 0);
	    }
	    printf("################\n");
	    rmat[5] = 1;
		previous_control = 0;
	    for(i = 0; i < 16385; i=i+2048)
	    {
	        rmat[2] = i;
	        //compute
	        hoverRollCntrl();
			delta_control = roll_control - previous_control;
            previous_control = roll_control;
			printf("rmat6 roll_control delta_control %d %d %d\n", rmat[2], roll_control, delta_control);
			assert(delta_control <= 0);
	    }
	    printf("################\n");
	    rmat[5] = -1;
		previous_control = 0;
	    for(i = 16384; i > -1; i=i-2048)
	    {
	        rmat[2] = i;
	        //compute
	        hoverRollCntrl();
			delta_control = roll_control - previous_control;
            previous_control = roll_control;
			printf("rmat6 roll_control delta_control %d %d %d\n", rmat[2], roll_control, delta_control);
			assert(delta_control <= 0);
	    }
	    printf("################\n");
	    printf("################\n");

        printf("hover_roll_control_4 PASSED\n");   
	}

	void test_hover_altitude_control_1()
	{
        //test that in hovering, for sonar height less than minimum allowed altitude, throttle = throttle offset

	    //overwrite inputs
		flags._.pitch_feedback = 1;
		flags._.GPS_steering = 0;
		current_orientation = F_HOVER;
        nb_sample_wait = 0;
		hover_counter = 0;

        accelEarth[2] = 0;
		IMUlocationz._.W1 = 101;

		udb_flags._.sonar_height_valid = 1;
		sonar_height_to_ground = 80;

		udb_flags._.baro_valid = 1;
		barometer_altitude = 0;

		int16_t z_target = (int16_t)(HOVER_TARGET_HEIGHT_MIN);
		int16_t vz_target = 0;

		hoverAltitudeCntrl();
        printf("throttle_control %d\n", throttle_control);
        assert(throttle_control == (int16_t)(2.0*SERVORANGE*(HOVER_THROTTLE_MAX)));
        printf("hover_altitude_control_1 PASSED\n");   
	}

	void test_hover_altitude_control_2()
	{
        //test that in hovering, for sonar height = target height, throttle control = throttle offset

	    //overwrite inputs
		flags._.pitch_feedback = 1;
		flags._.GPS_steering = 0;
		current_orientation = F_HOVER;
        nb_sample_wait = 0;
		hover_counter = 0;

        accelEarth[2] = 0;
		IMUlocationz._.W1 = 101;

		udb_flags._.sonar_height_valid = 1;
		sonar_height_to_ground = (int16_t)(HOVER_TARGET_HEIGHT_MIN);

		udb_flags._.baro_valid = 1;
		barometer_altitude = 0;

		int16_t z_target = (int16_t)(HOVER_TARGET_HEIGHT_MIN);
		int16_t vz_target = 0;

		int16_t i;
		for(i = 0; i < 300; i=i+1)
	    {
			hoverAltitudeCntrl();
		}
        printf("throttle_control %d\n", throttle_control);
        assert(throttle_control == (int16_t)(2.0*SERVORANGE*(HOVER_THROTTLE_OFFSET)));
        printf("hover_altitude_control_2 PASSED\n");   
	}

	void test_hover_altitude_control_3()
	{
        //test that in hovering, for sonar height higher than target height, throttle control < throttle offset

	    //overwrite inputs
		flags._.pitch_feedback = 1;
		flags._.GPS_steering = 0;
		current_orientation = F_HOVER;
        nb_sample_wait = 0;
		hover_counter = 0;

        accelEarth[2] = 0;
		IMUlocationz._.W1 = 101;

		udb_flags._.sonar_height_valid = 1;
		sonar_height_to_ground = (int16_t)(HOVER_TARGET_HEIGHT_MIN) + 50;

		udb_flags._.baro_valid = 1;
		barometer_altitude = 0;

		int16_t z_target = (int16_t)(HOVER_TARGET_HEIGHT_MIN);
		int16_t vz_target = 0;

		int16_t i;
		for(i = 0; i < 300; i=i+1)
	    {
			hoverAltitudeCntrl();
		}
        printf("throttle_control %d\n", throttle_control);
        assert(throttle_control == (int16_t)(2.0*SERVORANGE*(HOVER_THROTTLE_MIN)));
        printf("hover_altitude_control_3 PASSED\n");   
	}

	void test_hover_altitude_control_4()
	{
        //test that in hovering, if altitude exceeds 100m, 

	    //overwrite inputs

		flags._.pitch_feedback = 1;
		flags._.GPS_steering = 0;
		current_orientation = F_HOVER;
        nb_sample_wait = 0;
		hover_counter = 0;

        accelEarth[2] = 0;
		IMUlocationz._.W1 = 101;

		udb_flags._.sonar_height_valid = 1;
		sonar_height_to_ground = 10001;

		udb_flags._.baro_valid = 1;
		barometer_altitude = 0;

		int16_t z_target = (int16_t)(HOVER_TARGET_HEIGHT_MIN);
		int16_t vz_target = 0;

		hoverAltitudeCntrl();

        printf("throttle_control %d\n", throttle_control);
        assert(throttle_control == (int16_t)(2.0*SERVORANGE*(HOVER_THROTTLE_MIN)));
        printf("hover_altitude_control_4 PASSED\n");   
	}

	void test_hover_altitude_control_5()
	{
        //test out of range

	    //overwrite inputs
		flags._.pitch_feedback = 1;
		flags._.GPS_steering = 0;

        accelEarth[2] = 0;
		IMUlocationz._.W1 = 101;

		udb_flags._.sonar_height_valid = 1;
		sonar_height_to_ground = 10001;

		udb_flags._.baro_valid = 1;
		barometer_altitude = 0;

		current_orientation = F_HOVER;
        nb_sample_wait = 0;
		hover_counter = 0;

		hoverAltitudeCntrl();

        printf("throttle_control %d\n", throttle_control);
        assert(throttle_control == (int16_t)(2.0*SERVORANGE*(HOVER_THROTTLE_MIN)));
        printf("hover_altitude_control_5 PASSED\n");   
	}




    printf("Start unit tests\n");

    test_expfilter();
	debug_hovering_roll_control();
    //test_hover_roll_control_1();
 	//test_hover_roll_control_2();
	//test_hover_roll_control_3();
	//test_hover_roll_control_4();
	//test_hover_altitude_control_1();
	//test_hover_altitude_control_2();
	//test_hover_altitude_control_3();
	//test_hover_altitude_control_4();

	system("pause");

#else
	udb_run();
#endif
	
	// This never returns.

	return 0;
}
