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
#if (USE_TELELOG == 1)
#include "telemetry_log.h"
#endif
#include "../libUDB/heartbeat.h"
#if (SILSIM != 1)
#include "../libUDB/libUDB_internal.h" // Needed for access to RCON
#endif
#include "../libDCM/libDCM_internal.h" // Needed for access to internal DCM values
#include "../libUDB/barometerAltitude.h"
#include <string.h>

int16_t additional_int16_export1 = 0;
int16_t additional_int16_export2 = 0;
int16_t additional_int16_export3 = 0;
int16_t additional_int16_export4 = 0;
int16_t additional_int16_export5 = 0;
int16_t additional_int16_export6 = 0;
int16_t additional_int16_export7 = 0;
int16_t additional_int16_export8 = 0;
int16_t additional_int16_export9 = 0;

int16_t voltage = 0;
int16_t current = 0;
int16_t mAh_used = 0;

#if (SERIAL_OUTPUT_FORMAT != SERIAL_MAVLINK) // All MAVLink telemetry code is in MAVLink.c

#if (FLY_BY_DATALINK_ENABLED == 1)
#include "fly_by_datalink.h"
#endif

#include <stdarg.h>


union intbb voltage_milis = {0};
union intbb voltage_temp;

void sio_newMsg(uint8_t);
void sio_voltage_low(uint8_t inchar);
void sio_voltage_high(uint8_t inchar);

void sio_fp_data(uint8_t inchar);
void sio_fp_checksum(uint8_t inchar);

void sio_cam_data(uint8_t inchar);
void sio_cam_checksum(uint8_t inchar);

void sio_fbdl_data(unsigned char inchar);

char fp_high_byte;
uint8_t fp_checksum;

void (*sio_parse)(uint8_t inchar) = &sio_newMsg;


#define SERIAL_BUFFER_SIZE 512
char serial_buffer[SERIAL_BUFFER_SIZE+1];
int16_t sb_index = 0;
int16_t end_index = 0;

void init_serial()
{
#if (SERIAL_OUTPUT_FORMAT == SERIAL_OSD_REMZIBI)
	dcm_flags._.nmea_passthrough = 1;
#endif

    udb_serial_set_rate(19200);
}

////////////////////////////////////////////////////////////////////////////////
// 
// Receive Serial Commands
//

void udb_serial_callback_received_byte(uint8_t rxchar)
{
	(*sio_parse)(rxchar); // parse the input byte
}

void sio_newMsg(uint8_t inchar)
{
	switch (inchar)
	{
	case 'V':
		sio_parse = &sio_voltage_high;
		break;
	
#if (FLIGHT_PLAN_TYPE == FP_LOGO)
	case 'L':
#else
	case 'W':
#endif
		fp_high_byte = -1; // -1 means we don't have the high byte yet (0-15 means we do)
		fp_checksum = 0;
		sio_parse = &sio_fp_data;
		flightplan_live_begin();
		break;

#if (CAM_USE_EXTERNAL_TARGET_DATA == 1)
	case 'T':
		fp_high_byte = -1; // -1 means we don't have the high byte yet (0-15 means we do)
		fp_checksum = 0;
		sio_parse = &sio_cam_data;
		camera_live_begin();
		break;
#endif

#if (FLY_BY_DATALINK_ENABLED == 1)
	case 'F':
		fp_checksum = 'F';
		sio_parse = &sio_fbdl_data;
		fbdl_live_begin();
		break;
#endif

	default:
		// error ?
		break;
	} // switch
}

void sio_voltage_high(uint8_t inchar)
{
	voltage_temp.BB = 0; // initialize our temp variable
	voltage_temp._.B1 = inchar;
	sio_parse = &sio_voltage_low;
}

void sio_voltage_low(uint8_t inchar)
{
	voltage_temp._.B0 = inchar;
	voltage_temp.BB = voltage_temp.BB * 2; // convert to voltage
	voltage_milis.BB = voltage_temp.BB;
	sio_parse = &sio_newMsg;
}

int8_t hex_char_val(uint8_t inchar)
{
	if (inchar >= '0' && inchar <= '9')
	{
		return (inchar - '0');
	}
	else if (inchar >= 'A' && inchar <= 'F')
	{
		return (inchar - 'A' + 10);
	}
	return -1;
}

// For UDB Logo instructions, bytes should be passed in using the following format
// (Below, an X represents a hex digit 0-F.  Mulit-digit values are MSB first.)
// L            begin remote Logo command
// XX   byte:   command
// XX   byte:   subcommand
// X    0-1:    do fly
// X    0-1:    use param
// XXXX word:   argument
// *            done with command data
// XX   byte:   checksum should equal the sum of the 10 bytes before the *, mod 256
//
// For example: "L0201000005*E8" runs:
// the DO command(02) for subroutine 01 with fly and param off(00) and an argument of 0005


// For classic Waypoints, bytes should be passed in using the following format
// (Below, an X represents a hex digit 0-F.  Mulit-digit values are MSB first.)
// W                    begin remote Waypoint command
// XXXXXXXX int32_t:    waypoint X value
// XXXXXXXX int32_t:    waypoint Y value
// XXXX     word:       waypoint Z value
// XXXX     word:       flags
// XXXXXXXX int32_t:    cam view X value
// XXXXXXXX int32_t:    cam view Y value
// XXXX     word:       cam view Z value
// *                    done with command data
// XX       byte:       checksum should equal the sum of the 44 bytes before the *, mod 256
//
// For example: "W0000006400000032000F020000000000000000000000*67" represents:
// the waypoint { {100, 50, 15}, F_INVERTED, {0, 0, 0} }
//

void sio_fp_data(uint8_t inchar)
{
	if (inchar == '*')
	{
		fp_high_byte = -1;
		sio_parse = &sio_fp_checksum;
	}
	else
	{
		int8_t hexVal = hex_char_val(inchar);
		if (hexVal == -1)
		{
			sio_parse = &sio_newMsg;
			return;
		}
		else if (fp_high_byte == -1)
		{
			fp_high_byte = hexVal * 16;
		}
		else
		{
			flightplan_live_received_byte(fp_high_byte + hexVal);
			fp_high_byte = -1;
		}
		fp_checksum += inchar;
	}
}

void sio_fp_checksum(uint8_t inchar)
{
	int8_t hexVal = hex_char_val(inchar);

	if (hexVal == -1)
	{
		sio_parse = &sio_newMsg;
	}
	else if (fp_high_byte == -1)
	{
		fp_high_byte = hexVal * 16;
	}
	else
	{
		uint8_t v = fp_high_byte + hexVal;
		if (v == fp_checksum)
		{
			flightplan_live_commit();
		}
		sio_parse = &sio_newMsg;
	}
}

////////////////////////////////////////////////////////////////////////////////
// 
// Output Serial Data
//

#if (USE_TELELOG == 1)
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

void serial_output(char* format, ...)
{
	char telebuf[200];

	va_list arglist;
	va_start(arglist, format);
	
	int16_t len = vsnprintf(telebuf, sizeof(telebuf), format, arglist);

//	static int maxlen = 0;
//	if (len > maxlen) {
//		maxlen = len;
//		printf("maxlen %u\r\n", maxlen);
//	}

	int16_t start_index = end_index;
	int16_t remaining = (SERIAL_BUFFER_SIZE - start_index);
	if (remaining < len) {
		printf("SERBUF discarding %u bytes\r\n", len - remaining);
	}
	if (remaining > 1)
	{
		strncpy((char*)(&serial_buffer[start_index]), telebuf, MIN(remaining, len));
		end_index = start_index + MIN(remaining, len);
		serial_buffer[end_index] = '\0';
	}
	if (sb_index == 0)
	{
		udb_serial_start_sending_data();
	}
	log_telemetry(telebuf, len);

	va_end(arglist);
}
#else
// add this text to the output buffer
void serial_output(char* format, ...)
{
	va_list arglist;
	
	va_start(arglist, format);
	
	int16_t start_index = end_index;
	int16_t remaining = SERIAL_BUFFER_SIZE - start_index;

	if (remaining > 1)
	{
		int16_t wrote = vsnprintf((char*)(&serial_buffer[start_index]), (size_t)remaining, format, arglist);
		end_index = start_index + wrote;
	}

	if (sb_index == 0)
	{
		udb_serial_start_sending_data();
	}

	va_end(arglist);
}
#endif // USE_TELELOG

int16_t udb_serial_callback_get_byte_to_send(void)
{
	uint8_t txchar = serial_buffer[ sb_index++ ];

	if (txchar)
	{
		return txchar;
	}
	else
	{
		sb_index = 0;
		end_index = 0;
	}
	return -1;
}

#if (SERIAL_OUTPUT_FORMAT == SERIAL_TELEMETRY)

extern int16_t waypointIndex;
extern int16_t segmentIndex;
    
void serial_output_8hz(void)
{   
    if (udb_heartbeat_counter % (HEARTBEAT_HZ/HEARTBEAT_UDB_TELEMETRY) == 0)
	{
        static int16_t pwOut_save[NUM_OUTPUTS + 1];
        int16_t throttle_offset = 2242;

        serial_output(
                     "F2;T:%li;"
                      "vo%i;cu%i;au%i;"
                      "cpu:%u;"
                      "ma%i;mb%i;mc%i;"
                      ,
            tow.WW,
            voltage, current, mAh_used,
            (uint16_t)udb_cpu_load(),
#if (MAG_YAW_DRIFT == 1)
            magFieldEarth[0],magFieldEarth[1],magFieldEarth[2]
#else
            (int16_t)0, (int16_t)0, (int16_t)0
#endif // MAG_YAW_DRIFT
            );
            // Approximate time passing between each telemetry line, even though
            // we may not have new GPS time data each time through.
            //when using 4Hz output
            //if (tow.WW > 0) tow.WW += 250; 
//#ifdef TestGains
//            tow.WW += (int16_t)(1000/HEARTBEAT_UDB_TELEMETRY);
//#else
//            if (tow.WW > 0) tow.WW += (int16_t)(1000/HEARTBEAT_UDB_TELEMETRY);
//#endif

            // Save  pwIn and PwOut buffers for printing next time around
//            int16_t i;
//				for (i=0; i <= NUM_INPUTS; i++)
//					pwIn_save[i] = udb_pwIn[i];
//            for (i=0; i < NUM_OUTPUTS; i++)
//                pwOut_save[i] = udb_pwOut[i];
//				for (i= 1; i <= NUM_INPUTS; i++)
//					serial_output("p%ii%i:",i,pwIn_save[i]);
//            for (i=4; i <= NUM_OUTPUTS; i++)
//                serial_output("p%io:%i;",i,pwOut_save[i]);

            serial_output("t1%i;t2%i;t3%i;", tele_throttle1-throttle_offset, tele_throttle2-throttle_offset, tele_throttle3-throttle_offset);
            serial_output("mt%i;th%i;", tele_mean_throttle-throttle_offset, throttle_hover_control);
//            serial_output("ix%i;iy%i;iz%i;", 
//                    (int16_t)(100*IMUlocationx._.W1), 
//                    (int16_t)(100*IMUlocationy._.W1), 
//                    (int16_t)(100*IMUlocationz._.W1));
            serial_output("vz%i;", IMUvelocityz._.W1);
            
            serial_output("ye%i;pe%i;re%i;",
                yaw_error, pitch_error, roll_error);
            
            serial_output("yo%i;",
                omegagyro[2]);
            
//           serial_output("mf%i;mi%i;",
//                100*flags._.mag_failure, 100*flags._.invalid_mag_reading);
//            //GPS X-Y positioning : navigation
//            serial_output("ey%i;tf%i;ph%i;",
//                earth_yaw, tofinish_line_factor10, 100*(int16_t)(control_position_hold));
//            
//            //GPS X-Y positioning : pitch control
//            serial_output("hp%i;tp%i;pa%i;pv%i;pc%i;",
//                hovering_pitch_order, target_pitch, -rmat[7], pitch_v_target, pitch_hover_corr);
//            
//            //GPS X-Y positioning : roll control
//            serial_output("hr%i;tr%i;ra%i;rv%i;rc%i;",
//                hovering_roll_order, target_roll, rmat[6], roll_v_target, roll_hover_corr);
//            
#if ( USE_LIDAR	== 1 )
            serial_output("ld%i;lv%i;", lidar_distance, 100*udb_flags._.lidar_height_valid) ;
#endif
#if ( USE_SONAR	== 1 )
            serial_output("sd%i;sv%i;", sonar_distance, 100*udb_flags._.sonar_height_valid) ;
#endif

#if ( BAROMETER_ALTITUDE == 1 && SILSIM != 1)            
            serial_output("ba%li;",
                get_barometer_altitude());
#endif       
        serial_output("end;\r\n");
    }
}

#else // If SERIAL_OUTPUT_FORMAT is set to SERIAL_NONE, or is not set

void serial_output_8hz(void)
{
}

#endif
#endif // (SERIAL_OUTPUT_FORMAT != SERIAL_MAVLINK)
