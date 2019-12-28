#include "gtest/gtest.h"
#include "defines.h"
#include <stdio.h>


namespace 
{
    // The fixture for testing class Foo.
    class TricopterMotorMeanControl : public ::testing::Test
    {
      protected:

          // PID gains
          const uint16_t tilt_ki = (uint16_t)(RMAX*0.0);
          const uint16_t tilt_kp = (uint16_t)(RMAX*0.5);
          const uint16_t tilt_rate_kp = (uint16_t)(RMAX*0.22);
          const uint16_t tilt_rate_kd = (uint16_t)(RMAX*0.0);
          const uint16_t yaw_ki = (uint16_t)(RMAX*0.0);
          const uint16_t yaw_kp = (uint16_t)(RMAX*0.0);
          const uint16_t yaw_rate_kp = (uint16_t)(RMAX*0.0);

          // If the constructor and destructor are not enough for setting up
          // and cleaning up each test, you can define the following methods:
    
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

    TEST_F(TricopterMotorMeanControl, meanControlWithLimiters)
    {
        rmat[6] = RMAX;
        rmat[7] = 0;
        motorCntrl(tilt_kp, tilt_ki, tilt_rate_kp, tilt_rate_kd, yaw_ki, yaw_kp, yaw_rate_kp);
        
        const int mean_control = 0.3333333333333333 * (udb_pwOut[MOTOR_A_OUTPUT_CHANNEL] + udb_pwOut[MOTOR_B_OUTPUT_CHANNEL] + udb_pwOut[MOTOR_C_OUTPUT_CHANNEL]);
        ASSERT_EQ(mean_control, 3000);
    }

}  // namespace
