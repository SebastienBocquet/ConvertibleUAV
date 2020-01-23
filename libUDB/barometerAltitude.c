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

#include "libDCM.h"
#include "barometer.h"
#include "barometerAltitude.h"
#include <stdio.h>
#include <stdlib.h>
#if (SILSIM == 1)
#include <cmath>
#endif

//  The origin is recorded as the altitude of the plane during power up

long barometer_pressure_gnd = 0;
int barometer_temperature_gnd = 0;

long barometer_altitude;      // above sea level altitude - ASL (millimeters)
long barometer_agl_altitude;  // above ground level altitude - AGL
long barometer_pressure;
int barometer_temperature;

inline int get_barometer_temperature(void) { return barometer_temperature; }
inline long get_barometer_pressure(void) { return barometer_pressure; }
inline long get_barometer_altitude(void) { return barometer_altitude; }
inline long get_barometer_agl_altitude(void) { return barometer_agl_altitude; }

void altimeter_calibrate(void) {
  barometer_temperature_gnd = barometer_temperature;
  barometer_pressure_gnd = barometer_pressure;

#ifdef USE_DEBUG_IO
  printf("altimeter_calibrate: ground temp & pres set %i, %li\r\n",
         barometer_temperature_gnd, barometer_pressure_gnd);
#endif
}

#if (BAROMETER_ALTITUDE == 1)
void udb_barometer_callback(long pressure, int temperature, char status) {
  barometer_temperature = temperature;
  barometer_pressure = pressure;
}
#endif

void estBaroAltitude(void) {
#if (SILSIM == 1)
  return;
#endif

#if (BAROMETER_ALTITUDE == 1)

  if (barometer_pressure_gnd != 0) {
    // relative altitude above ground in cm
    float temperature_kelvin = 0.1f * (float)(barometer_temperature) + 273.15f;
    barometer_altitude = (long)(15400.0f * temperature_kelvin *
                                (1 - pow((((float)barometer_pressure) /
                                          ((float)barometer_pressure_gnd)),
                                         (1 / 5.255f))));
#ifdef USE_DEBUG_IO
//		printf("estAltitude %li\r\n", barometer_altitude);
#endif
  }
#endif  // BAROMETER_ALTITUDE
}
