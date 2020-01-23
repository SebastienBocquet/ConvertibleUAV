/*
 * File:   sonarAltitude.c
 * Author: Seb
 *
 * Created on 28 décembre 2017, 23:45
 */

#include "defines.h"
#include "sonarAltitude.h"

#if (USE_SONAR == 1)

// USEABLE_SONAR_DISTANCE may well vary with type of ground cover (e.g. long
// grass may be less).
// Pete Hollands ran the code with #define SERIAL_OUTPUT SERIAL_UDB_SONAR while
// flying low
// over his landing area, which was a freshly cut straw field. Post flight, he
// anlaysed the CSV telemetry into a spreadsheet graph,
// and determined that all measurements below 4 meters were true, as long as
// there were at least 3 consecutive measurements,
// that were less than 4 meters (400 centimeters).
#define NO_READING_RECEIVED_DISTANCE \
  9999  // Distance denotes that no sonar reading was returned from sonar device
#define SONAR_SAMPLE_THRESHOLD \
  3  // Number of readings before code deems "certain" of a true reading.
#define UDB_SONAR_PWM_UNITS_TO_CENTIMETERS 278

#if (USE_SONAR_ON_PWM_INPUT_7 == 1)
uint16_t udb_pwm_sonar = 0;
#endif

int16_t sonar_distance;          // distance to target in centimeter
int16_t sonar_height_to_ground;  // calculated distance to ground in Earth's Z
                                 // Plane allowing for tilt
int16_t failure_sonar_distance = OUT_OF_RANGE_DISTANCE;
unsigned char sonar_good_sample_count =
    0;  // Tracks the number of consequtive good samples up until
        // SONAR_SAMPLE_THRESHOLD is reached.
unsigned char sonar_no_readings_count = 0;  // Tracks number of UDB frames since
                                            // last sonar reading was sent by
                                            // sonar device

void setFailureSonarDist(int16_t distance) {
  failure_sonar_distance = distance;
}

void calculate_sonar_height_above_ground(void) {
#if (SILSIM == 1)
  return;
#endif

  sonar_distance = OUT_OF_RANGE_DISTANCE;

  if (udb_flags._.sonar_updated == 1 || ANALOG_SONAR_INPUT_CHANNEL > 0) {
    sonar_no_readings_count = 0;

#if (USE_SONAR_ON_PWM_INPUT_7 == 1)
    // PWM input
    union longbbbb accum;
    accum.WW =
        __builtin_muluu(udb_pwm_sonar, UDB_SONAR_PWM_UNITS_TO_CENTIMETERS);
    sonar_distance = accum._.W1 << 1;
#endif

#if (ANALOG_SONAR_INPUT_CHANNEL > 0)
    // Analogic input
    sonar_distance = (int16_t)(
        udb_analogInputs[ANALOG_SONAR_INPUT_CHANNEL - 1].input / 73 + 440);
#endif

    if (sonar_distance > USEABLE_SONAR_DISTANCE ||
        sonar_distance < SONAR_MINIMUM_DISTANCE) {
      sonar_height_to_ground = failure_sonar_distance;
      sonar_good_sample_count = 0;
      udb_flags._.sonar_height_valid = 0;
#if (LED_RED_SONAR_CHECK == 1)
      LED_RED = LED_ON;
#endif
    } else {
      sonar_good_sample_count++;
      if (sonar_good_sample_count > SONAR_SAMPLE_THRESHOLD) {
        sonar_good_sample_count = SONAR_SAMPLE_THRESHOLD;
        sonar_height_to_ground = sonar_distance;
        udb_flags._.sonar_height_valid = 1;
#if (LED_RED_SONAR_CHECK == 1)
        LED_RED = LED_OFF;
#endif
      } else {
        sonar_height_to_ground = failure_sonar_distance;
        udb_flags._.sonar_height_valid = 0;
#if (LED_RED_SONAR_CHECK == 1)
        LED_RED = LED_ON;
#endif
      }
    }
    udb_flags._.sonar_updated = 0;
  } else {
    if (sonar_no_readings_count < 7)  // This assumes running at UDB frame rate
    {
      sonar_no_readings_count++;
    } else {
      sonar_height_to_ground = NO_READING_RECEIVED_DISTANCE;
      udb_flags._.sonar_height_valid = 0;
#if (LED_RED_SONAR_CHECK == 1)
      LED_RED = LED_ON;
#endif
    }
  }
  return;
}
#endif
