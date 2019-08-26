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
            throttle_hover_control = 0;
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
        motorCntrl();
        ASSERT_EQ(udb_pwOut[MOTOR_A_OUTPUT_CHANNEL], 3081);
        ASSERT_EQ(udb_pwOut[MOTOR_B_OUTPUT_CHANNEL], 3081);
        ASSERT_EQ(udb_pwOut[MOTOR_C_OUTPUT_CHANNEL], 2919);
        ASSERT_EQ(udb_pwOut[MOTOR_D_OUTPUT_CHANNEL], 2919);
    }

}  // namespace
