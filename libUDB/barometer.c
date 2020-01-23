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

#include "libUDB_internal.h"
#include "I2C.h"
#include "barometer.h"
#include "defines.h"

#if (BAROMETER_ALTITUDE == 1)

#if (USE_BARO_BMP085 == 1)

#define BMP085_ADDRESS 0xEE  // I2C address of BMP085
#define USE_BMP085_ON_I2C 2

// BMP085 oversampling can be set from 0 thru 3
//#define OSS 3
//#define OSS 2
//#define OSS 1
#define OSS 2

typedef union {
  struct {
    int ac1;
    int ac2;
    int ac3;
    unsigned int ac4;
    unsigned int ac5;
    unsigned int ac6;
    int b1;
    int b2;
    int mb;
    int mc;
    int md;
  };
  unsigned char buf[22];
} barCalib_union_t;

static barCalib_union_t bc;  // barometer calibration constants
static long b5;  // this is the important number for calculating the temperature

static const unsigned char bmp085read_barCalib[] = {
    0xAA};  // Address of the first register to read
static const unsigned char bmp085read_barData[] = {0xF6};

static const unsigned char bmp085write_index[] = {
    0xF4};  // Address of the command register to write
static unsigned char bmp085read_barTemp[] = {0x2E};
static unsigned char bmp085read_barPres[] = {0x34 + (OSS << 6)};

static unsigned char barData[3];

static int barMessage = 0;  // message type, state machine counter
static int barPause = 0;

void ReadBarTemp_callback(boolean I2CtrxOK);
void ReadBarPres_callback(boolean I2CtrxOK);
void ReadBarCalib_callback(boolean I2CtrxOK);

#if (USE_BMP085_ON_I2C == 1)
#define I2C_Normal I2C1_Normal
#define I2C_Read I2C1_Read
#define I2C_Write I2C1_Write
#define I2C_Reset I2C1_Reset
#elif(USE_BMP085_ON_I2C == 2)
#define I2C_Normal I2C2_Normal
#define I2C_Read I2C2_Read
#define I2C_Write I2C2_Write
#define I2C_Reset I2C2_Reset
#endif

barometer_callback_funcptr barometer_callback = NULL;

void rxBarometer(barometer_callback_funcptr callback)  // service the barometer
{
  barometer_callback = callback;

  if (I2C_Normal() == false)  // if I2C is not ok
  {
    barMessage = 0;  // start over again
    I2C_Reset();     // reset the I2C
    return;
  }

  if (barPause == 0) {
    barMessage++;
    if (barMessage > 17) {
      barMessage = 4;
    }

    switch (barMessage) {
      case 1:
        break;
      case 2:
        break;
      case 3:
        I2C_Read(BMP085_ADDRESS, bmp085read_barCalib, 1, bc.buf, 22,
                 &ReadBarCalib_callback, I2C_MODE_WRITE_ADDR_READ);
        break;

      case 4:
        barPause = 1;  // probably not required
        // at HEARTBEAT_Hz=80Hz, it gives 25ms to write on device. For BMP085,
        // it is ok for OSS=0, 1 or 2
        I2C_Write(BMP085_ADDRESS, bmp085write_index, 1, bmp085read_barPres, 1,
                  NULL);
        break;
      case 5:
        I2C_Read(BMP085_ADDRESS, bmp085read_barData, 1, barData, 3,
                 &ReadBarPres_callback, I2C_MODE_WRITE_ADDR_READ);
        break;
      case 6:
        barPause = 1;  // probably not required
        // at HEARTBEAT_Hz=80Hz, it gives 25ms to write on device. For BMP085,
        // it is ok for OSS=0, 1 or 2
        I2C_Write(BMP085_ADDRESS, bmp085write_index, 1, bmp085read_barPres, 1,
                  NULL);
        break;
      case 7:
        I2C_Read(BMP085_ADDRESS, bmp085read_barData, 1, barData, 3,
                 &ReadBarPres_callback, I2C_MODE_WRITE_ADDR_READ);
        break;
      case 8:
        barPause = 1;  // probably not required
        // at HEARTBEAT_Hz=80Hz, it gives 25ms to write on device. For BMP085,
        // it is ok for OSS=0, 1 or 2
        I2C_Write(BMP085_ADDRESS, bmp085write_index, 1, bmp085read_barPres, 1,
                  NULL);
        break;
      case 9:
        I2C_Read(BMP085_ADDRESS, bmp085read_barData, 1, barData, 3,
                 &ReadBarPres_callback, I2C_MODE_WRITE_ADDR_READ);
        break;
      case 10:
        barPause = 1;  // probably not required
        // at HEARTBEAT_Hz=80Hz, it gives 25ms to write on device. For BMP085,
        // it is ok for OSS=0, 1 or 2
        I2C_Write(BMP085_ADDRESS, bmp085write_index, 1, bmp085read_barPres, 1,
                  NULL);
        break;
      case 11:
        I2C_Read(BMP085_ADDRESS, bmp085read_barData, 1, barData, 3,
                 &ReadBarPres_callback, I2C_MODE_WRITE_ADDR_READ);
        break;
      case 12:
        barPause = 1;  // probably not required
        // at HEARTBEAT_Hz=80Hz, it gives 25ms to write on device. For BMP085,
        // it is ok for OSS=0, 1 or 2
        I2C_Write(BMP085_ADDRESS, bmp085write_index, 1, bmp085read_barPres, 1,
                  NULL);
        break;
      case 13:
        I2C_Read(BMP085_ADDRESS, bmp085read_barData, 1, barData, 3,
                 &ReadBarPres_callback, I2C_MODE_WRITE_ADDR_READ);
        break;
      case 14:
        barPause = 1;  // probably not required
        // at HEARTBEAT_Hz=80Hz, it gives 25ms to write on device. For BMP085,
        // it is ok for OSS=0, 1 or 2
        I2C_Write(BMP085_ADDRESS, bmp085write_index, 1, bmp085read_barPres, 1,
                  NULL);
        break;
      case 15:
        I2C_Read(BMP085_ADDRESS, bmp085read_barData, 1, barData, 3,
                 &ReadBarPres_callback, I2C_MODE_WRITE_ADDR_READ);
        break;

      case 16:
        barPause = 1;  // probably not required
        // at HEARTBEAT_Hz=80Hz, it gives 25ms to write on device. For BMP085,
        // it is ok for OSS=0, 1 or 2
        I2C_Write(BMP085_ADDRESS, bmp085write_index, 1, bmp085read_barTemp, 1,
                  NULL);
        break;
      case 17:
        I2C_Read(BMP085_ADDRESS, bmp085read_barData, 1, barData, 2,
                 &ReadBarTemp_callback, I2C_MODE_WRITE_ADDR_READ);
        break;

      default:
        barMessage = 0;
        break;
    }
  } else {
    barPause--;
  }
}

/*
#define BIG_ENDIAN      0
#define LITTLE_ENDIAN   1

int TestByteOrder()
{
        short int word = 0x0001;
        char *byte = (char *) &word;
        return(byte[0] ? LITTLE_ENDIAN : BIG_ENDIAN);
}
 */

static void byteswaparray(unsigned char* ary, int len) {
  int i;
  unsigned char tmp;

  for (i = 0; i < len; i += 2) {
    tmp = ary[i];
    ary[i] = ary[i + 1];
    ary[i + 1] = tmp;
  }
}

void ReadBarCalib_callback(boolean I2CtrxOK) {
  if (I2CtrxOK == true) {
    byteswaparray(bc.buf, 22);
#ifdef TEST_WITH_DATASHEET_VALUES
    bc.ac1 = 408;
    bc.ac2 = -72;
    bc.ac3 = -14383;
    bc.ac4 = 32741;
    bc.ac5 = 32757;
    bc.ac6 = 23153;
    bc.b1 = 6190;
    bc.b2 = 4;
    bc.mb = -32768;
    bc.mc = -8711;
    bc.md = 2868;
#endif
  }
}

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
static long bmp085CalcTemperature(unsigned int ut) {
  long x1, x2;

  x1 = (((long)ut - (long)bc.ac6) * (long)bc.ac5) >> 15;
  x2 = ((long)bc.mc << 11) / (x1 + bc.md);
  b5 = x1 + x2;

  return ((b5 + 8) >> 4);
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085CalcTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
static long bmp085CalcPressure(long up) {
  long x1, x2, x3, b3, b6, b7, p;
  unsigned long b4;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (bc.b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (bc.ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((bc.ac1 * 4 + x3) << OSS) + 2) >> 2;
  // Calculate B4
  x1 = (bc.ac3 * b6) >> 13;
  x2 = (bc.b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (bc.ac4 * (unsigned long)(x3 + 32768)) >> 15;
  b7 = ((unsigned long)up - b3) * (50000 >> OSS);
  if (b7 < 0x80000000) {
    p = (b7 << 1) / b4;
  } else {
    p = (b7 / b4) << 1;
  }
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p = p + ((x1 + x2 + 3791) >> 4);

  return p;
}

void ReadBarTemp_callback(boolean I2CtrxOK) {
  long temperature;
  unsigned int ut;

  if (I2CtrxOK == true) {
    udb_flags._.baro_valid = 1;
#if (LED_RED_BARO_CHECK == 1)
    LED_RED = LED_OFF;
#endif

    //		byteswaparray((unsigned char*)&barData, 2);
    ut = ((unsigned int)(unsigned int)barData[0] << 8 |
          (unsigned int)barData[1]);
#ifdef TEST_WITH_DATASHEET_VALUES
    ut = 27898;
#endif
    temperature = bmp085CalcTemperature(ut);
  } else {
    // the operation failed - we should probably do something about it...
    udb_flags._.baro_valid = 0;
    setTriggerParams(SENSOR_FAILURE_PULSE_PERIOD,
                     SENSOR_FAILURE_PULSE_DURATION);
    activateTrigger(SENSOR_FAILURE_PULSE_PERIOD);
#if (LED_RED_BARO_CHECK == 1)
    LED_RED = LED_ON;
#endif
  }
}

void ReadBarPres_callback(boolean I2CtrxOK) {
  long pressure;

  if (I2CtrxOK == true) {
    udb_flags._.baro_valid = 1;
    LED_RED = LED_OFF;

    pressure =
        ((long)barData[0] << 16 | (long)barData[1] << 8 | (long)barData[2]) >>
        (8 - OSS);
#ifdef TEST_WITH_DATASHEET_VALUES
    pressure = 23843;
#endif
    pressure = bmp085CalcPressure(pressure);
    if (barometer_callback != NULL) {
      barometer_callback(pressure, ((b5 + 8) >> 4), 0);  // Callback
    }
  } else {
    // the operation failed - we should probably do something about it...
    udb_flags._.baro_valid = 0;
    setTriggerParams(SENSOR_FAILURE_PULSE_PERIOD,
                     SENSOR_FAILURE_PULSE_DURATION);
    activateTrigger(SENSOR_FAILURE_PULSE_PERIOD);
    LED_RED = LED_ON;
  }
}

#endif

#if USE_BARO_MS5611

#define USE_MS5611_ON_I2C 2
#define MS5611_ADDRESS \
  0xEC  // Module address write mode 1110 1110 bit1=CSB bit0=0 write
        // Module address read mode 1110 1111
// bit0 is managed by software dependent on if it is writing or reading

static unsigned char ms5611reset[] = {0x1E};  // reset
static unsigned char ms5611read_barCalib[] = {0xA2};
static unsigned char ms5611read_barCalib1[] = {0xA0};
static unsigned char ms5611read_barCalib2[] = {0xA2};
static unsigned char ms5611read_barCalib3[] = {0xA4};
static unsigned char ms5611read_barCalib4[] = {0xA6};
static unsigned char ms5611read_barCalib5[] = {0xA8};
static unsigned char ms5611read_barCalib6[] = {0xAA};
static unsigned char ms5611read_barCalib7[] = {0xAC};
static unsigned char ms5611read_barCalib8[] = {0xAE};
static const unsigned char ms5611read_barData[] = {0x00};

static unsigned char ms5611read_barTemp[] = {0x48};
static unsigned char ms5611read_barPres[] = {0x58};

static unsigned char barData[3];

static int barMessage = 0;  // message type, state machine counter
static int barCalibPause = 0;

void ReadBarTemp_callback(boolean I2CtrxOK);
void ReadBarPres_callback(boolean I2CtrxOK);
void ReadBarCalib_callback(boolean I2CtrxOK);

#if (USE_MS5611_ON_I2C == 1)
#define I2C_Normal I2C1_Normal
#define I2C_Read I2C1_Read
#define I2C_Write I2C1_Write
#define I2C_Reset I2C1_Reset
#elif(USE_MS5611_ON_I2C == 2)
#define I2C_Normal I2C2_Normal
#define I2C_Read I2C2_Read
#define I2C_Write I2C2_Write
#define I2C_Reset I2C2_Reset
#endif

typedef union {
  struct {
    unsigned int c0;
  };
  unsigned char buf[2];
} barCalib_union_t;

static barCalib_union_t bc;  // barometer calibration constants

uint16_t C[8];  // coefficients table for pressure sensor

static uint32_t D1;  // ADC value of the pressure conversion
static uint32_t D2;  // ADC value of the temperature conversion

void MS5611_compute_PT(int32_t* pressure, int16_t* temperature);

barometer_callback_funcptr barometer_callback = NULL;

void rxBarometer(barometer_callback_funcptr callback)  // service the barometer
{
  barometer_callback = callback;
  uint8_t coef_num;
  boolean check_ACK;

  if (I2C_Normal() == false)  // if I2C is not ok
  {
    barMessage = 0;  // start over again
    I2C_Reset();     // reset the I2C
    return;
  }

  if (barCalibPause == 0) {
    barMessage++;
    if (barMessage > 14) {
      barMessage = 11;
    }
    switch (barMessage) {
      case 1:
        // I2C_Write(ADDR,  ms5611reset, 1, ms5611reset, 0, NULL);
        break;
      case 2:
        I2C_Read(MS5611_ADDRESS, ms5611read_barCalib1, 1, bc.buf, 2,
                 &ReadBarCalib_callback, I2C_MODE_WRITE_ADDR_READ);
        break;
      case 3:
        C[0] = bc.c0;
        I2C_Read(MS5611_ADDRESS, ms5611read_barCalib2, 1, bc.buf, 2,
                 &ReadBarCalib_callback, I2C_MODE_WRITE_ADDR_READ);

        break;
      case 4:
        C[1] = bc.c0;
        I2C_Read(MS5611_ADDRESS, ms5611read_barCalib3, 1, bc.buf, 2,
                 &ReadBarCalib_callback, I2C_MODE_WRITE_ADDR_READ);

        break;
      case 5:
        C[2] = bc.c0;
        I2C_Read(MS5611_ADDRESS, ms5611read_barCalib4, 1, bc.buf, 2,
                 &ReadBarCalib_callback, I2C_MODE_WRITE_ADDR_READ);
        break;
      case 6:
        C[3] = bc.c0;
        I2C_Read(MS5611_ADDRESS, ms5611read_barCalib5, 1, bc.buf, 2,
                 &ReadBarCalib_callback, I2C_MODE_WRITE_ADDR_READ);
        break;
      case 7:
        C[4] = bc.c0;
        I2C_Read(MS5611_ADDRESS, ms5611read_barCalib6, 1, bc.buf, 2,
                 &ReadBarCalib_callback, I2C_MODE_WRITE_ADDR_READ);
        break;
      case 8:
        C[5] = bc.c0;
        I2C_Read(MS5611_ADDRESS, ms5611read_barCalib7, 1, bc.buf, 2,
                 &ReadBarCalib_callback, I2C_MODE_WRITE_ADDR_READ);
        break;
      case 9:
        C[6] = bc.c0;
        I2C_Read(MS5611_ADDRESS, ms5611read_barCalib8, 1, bc.buf, 2,
                 &ReadBarCalib_callback, I2C_MODE_WRITE_ADDR_READ);
        break;
      case 10:
        C[7] = bc.c0;
        break;

      case 11:
        barCalibPause = 1;
        I2C_Write(MS5611_ADDRESS, ms5611read_barTemp, 1, NULL, 0, NULL);
        break;
      case 12:
        I2C_Read(MS5611_ADDRESS, ms5611read_barData, 1, barData, 3,
                 &ReadBarTemp_callback, I2C_MODE_WRITE_ADDR_READ);
        break;
      case 13:
        barCalibPause = 1;
        I2C_Write(MS5611_ADDRESS, ms5611read_barPres, 1, NULL, 0, NULL);
        break;
      case 14:
        I2C_Read(MS5611_ADDRESS, ms5611read_barData, 1, barData, 3,
                 &ReadBarPres_callback, I2C_MODE_WRITE_ADDR_READ);
        break;
      default:
        barMessage = 0;
        break;
    }
  } else {
    barCalibPause--;
  }
}

static void byteswaparray(unsigned char* ary, int len) {
  int i;
  unsigned char tmp;

  for (i = 0; i < len; i += 2) {
    tmp = ary[i];
    ary[i] = ary[i + 1];
    ary[i + 1] = tmp;
  }
}

void ReadBarCalib_callback(boolean I2CtrxOK) {
  if (I2CtrxOK == true) {
    byteswaparray(bc.buf, 2);
  }
}

void ReadBarTemp_callback(boolean I2CtrxOK) {
  if (I2CtrxOK == true) {
    udb_flags._.baro_valid = 1;

    D2 = ((uint32_t)barData[0] << 16 | (uint32_t)barData[1] << 8 |
          (uint32_t)barData[2]);
#ifdef TEST_WITH_DATASHEET_VALUES
    D2 = 8569150;
#endif
  } else {
    // the operation failed - we should probably do something about it...
    udb_flags._.baro_valid = 0;
  }
}

void ReadBarPres_callback(boolean I2CtrxOK) {
  int32_t pressure = 0;
  int16_t temperature = 0;

  if (I2CtrxOK == true) {
    udb_flags._.baro_valid = 1;

    D1 = ((uint32_t)barData[0] << 16 | (uint32_t)barData[1] << 8 |
          (uint32_t)barData[2]);
#ifdef TEST_WITH_DATASHEET_VALUES
    D1 = 9085466;
#endif
    if (barometer_callback != NULL) {
      MS5611_compute_PT(&pressure, &temperature);
#ifdef TEST_WITH_DATASHEET_VALUES
      temperature = 2007;
      pressure = 100009;
#endif
      barometer_callback(pressure, temperature, 0);  // Callback
    }
  } else {
    // the operation failed - we should probably do something about it...
    udb_flags._.baro_valid = 0;
  }
}

void MS5611_compute_PT(int32_t* pres, int16_t* temp) {
  // uint32_t D1; //ADC value of the pressure conversion
  // uint32_t D2; //ADC value of the temperature conversion
  int32_t dT;  // difference between actual and measured temp.
  int64_t OFF;  // offset at actual temperature
  int64_t SENS;  // sensitivity at actual temperature
  int32_t T2;
  int64_t OFF2;
  int64_t SENS2;
  int32_t TEMP;
  dT = D2 - C[5] * pow(2, 8);
  OFF = C[2] * pow(2, 16) + (dT * C[4]) / pow(2, 7);
  SENS = C[1] * pow(2, 15) + (dT * C[3]) / pow(2, 8);
  TEMP = (int32_t)(2000 + dT * C[6] / pow(2, 23));
  // SECOND ORDER TEMP COMPENSATION
  //    if(TEMP < 2000){
  //    T2 = (dT *dT) >> 31;
  //    OFF2 = 5 *  ((TEMP - 2000) * (TEMP - 2000)) / 2;
  //    SENS2 = OFF2 / 2;
  //        if(TEMP < -1500){
  //            OFF2 = OFF2 + 7 * (TEMP + 1500) * (TEMP + 1500);
  //            SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
  //        }
  //    TEMP -= T2;
  //    OFF -= OFF2;
  //    SENS -= SENS2;
  //    }
  *pres = (int32_t)((D1 * SENS / pow(2, 21) - OFF) / pow(2, 15));
  *temp = (int16_t)(TEMP / 10);
}

#endif  // MS5611

#endif  // BAROMETER_ALTITUDE
