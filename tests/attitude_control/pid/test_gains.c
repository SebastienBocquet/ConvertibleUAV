#include "gtest/gtest.h"
#include "defines.h"
#include <stdio.h>

namespace 
{
    // The fixture for testing class Foo.
    class MotorCntrlPID : public ::testing::Test
    {
      protected:

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
              flags._.integral_pid_term = 0;
              udb_flags._.radio_on = 1;
              rmat[6] = 1000;
          }

          virtual void TearDown() 
          {
              // Code here will be called immediately after each test (right
              // before the destructor).
              printf("Entering tear down\n");
          }
          // Objects declared here can be used by all tests in the test case for Foo.
    };

    TEST_F(MotorCntrlPID, ComputesCorrectGains)
    {
        int output_pid_1;
        int output_pid_2;
        int motor_control;

        motorCntrl();
        // Simulate first PID controller
        // Kp gain must be 0.5 in options.h
        output_pid_1 = -0.5 * rmat[6];
        // Simulate second PID controller
        // Kp gain must be 0.22 in options.h
        // Kd gain must be 0.5 in options.h
        output_pid_2 = -output_pid_1 * 0.22 - output_pid_1 * 0.5;
        // Scale motor order for an X configuration quadcopter
        motor_control = -(3./4) * output_pid_2;
        motor_control = -267;
        ASSERT_EQ(udb_pwOut[MOTOR_A_OUTPUT_CHANNEL], 3000 - motor_control);
        ASSERT_EQ(udb_pwOut[MOTOR_B_OUTPUT_CHANNEL], 3000 - motor_control);
        ASSERT_EQ(udb_pwOut[MOTOR_C_OUTPUT_CHANNEL], 3000 + motor_control);
        ASSERT_EQ(udb_pwOut[MOTOR_D_OUTPUT_CHANNEL], 3000 + motor_control);

        rmat[6] = 2000;
        motorCntrl();
        output_pid_1 = -0.5 * rmat[6];
        output_pid_2 = -output_pid_1 * 0.22 - output_pid_1 * 0.5;
        motor_control = -(3./4) * output_pid_2;
        motor_control = -537;
        ASSERT_EQ(udb_pwOut[MOTOR_A_OUTPUT_CHANNEL], 3000 - motor_control);
        ASSERT_EQ(udb_pwOut[MOTOR_B_OUTPUT_CHANNEL], 3000 - motor_control);
        ASSERT_EQ(udb_pwOut[MOTOR_C_OUTPUT_CHANNEL], 3000 + motor_control);
        ASSERT_EQ(udb_pwOut[MOTOR_D_OUTPUT_CHANNEL], 3000 + motor_control);

        // Test if max throttle limiter activates
        rmat[6] = RMAX;
        motorCntrl();
        motor_control = (int)((1+0.95) * 2000);
        ASSERT_EQ(udb_pwOut[MOTOR_A_OUTPUT_CHANNEL], motor_control);
        ASSERT_EQ(udb_pwOut[MOTOR_B_OUTPUT_CHANNEL], motor_control);

        // Test if min throttle limiter activates
        rmat[6] = RMAX;
        motorCntrl();
        motor_control = (int)((1+0.2) * 2000);
        ASSERT_EQ(udb_pwOut[MOTOR_C_OUTPUT_CHANNEL], motor_control);
        ASSERT_EQ(udb_pwOut[MOTOR_D_OUTPUT_CHANNEL], motor_control);


    }

}  // namespace
