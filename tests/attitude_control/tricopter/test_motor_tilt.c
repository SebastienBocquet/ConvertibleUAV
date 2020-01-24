#include "gtest/gtest.h"
#include "defines.h"
#include <stdio.h>
#include <math.h>


namespace 
{
    // The fixture for testing class Foo.
    class TricopterMotorTilt : public ::testing::Test
    {
      protected:

          // If the constructor and destructor are not enough for setting up
          // and cleaning up each test, you can define the following methods:
    
          //tricopter geometry
          const float cos_alpha = 0.6647579365354364;
          const float sin_alpha = 0.7470588235294118 ;
          const float R = 0.25;
          const float R_A = 0.425;
          const float R_B = 0.443;
          const float k_pitch = R / (2 * R_A * cos_alpha);
          const float k_roll = R / (R_A * sin_alpha);

          // PID gains
          const uint16_t tilt_ki = (uint16_t)(RMAX*0.0);
          const uint16_t tilt_kp = (uint16_t)(RMAX*0.5);
          const uint16_t tilt_rate_kp = (uint16_t)(RMAX*0.22);
          const uint16_t tilt_rate_kd = (uint16_t)(RMAX*0.0);
          const uint16_t yaw_ki = (uint16_t)(RMAX*0.0);
          const uint16_t yaw_kp = (uint16_t)(RMAX*3.0);
          const uint16_t yaw_rate_kp = (uint16_t)(RMAX*1.3);

          virtual void SetUp() 
          {
              // Code here will be called immediately after the constructor (right
              // before each test).
              printf("Entering set up\n");
              dcm_flags._.calib_finished = 1;
              dcm_flags._.yaw_init_finished = 1;
              current_orientation = F_HOVER;
              udb_pwIn[THROTTLE_HOVER_INPUT_CHANNEL] = 3000;
              flags._.integral_pid_term = 1;
              udb_flags._.radio_on = 1;

          }

          virtual void TearDown() 
          {
              // Code here will be called immediately after each test (right
              // before the destructor).
              printf("Entering tear down\n");
              reset_target_orientation();
              reset_derivative_terms();
              reset_integral_terms();
          }
          // Objects declared here can be used by all tests in the test case for Foo.
    };

    TEST_F(TricopterMotorTilt, neutralTilt)
    {
        udb_pwIn[INPUT_CHANNEL_AUX1] = 3000;
        yaw_quad_control = 0;
        motorTiltCntrl();       
        motorTiltServoMix();
        ASSERT_EQ(udb_pwOut[MOTOR_TILT_OUTPUT_CHANNEL], 3000);
    }

    TEST_F(TricopterMotorTilt, motorTiltManual)
    {
        udb_pwIn[INPUT_CHANNEL_AUX1] = 3500;
        yaw_quad_control = 0;
        motorTiltCntrl();       
        motorTiltServoMix();
        int motor_tilt_pwm = (120./90) * (udb_pwIn[INPUT_CHANNEL_AUX1] - 3000);
        ASSERT_EQ(udb_pwOut[MOTOR_TILT_OUTPUT_CHANNEL], 3000 + motor_tilt_pwm);
    }

    TEST_F(TricopterMotorTilt, motorTiltMax)
    {
        udb_pwIn[INPUT_CHANNEL_AUX1] = 4000;
        yaw_quad_control = 0;
        motorTiltCntrl();       
        motorTiltServoMix();
        ASSERT_EQ(udb_pwOut[MOTOR_TILT_OUTPUT_CHANNEL], 4000);
    }

    TEST_F(TricopterMotorTilt, motorTiltMin)
    {
        udb_pwIn[INPUT_CHANNEL_AUX1] = 2000;
        yaw_quad_control = 0;
        motorTiltCntrl();       
        motorTiltServoMix();
        ASSERT_EQ(udb_pwOut[MOTOR_TILT_OUTPUT_CHANNEL], 2000);
    }



}  // namespace
