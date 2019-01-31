#include "gtest/gtest.h"
#include "defines.h"
#include <stdio.h>

namespace
{
    // The fixture for testing class Foo.
    class MotorCntrlIntegralTerms : public ::testing::Test
    {
      protected:

        // If the constructor and destructor are not enough for setting up
        // and cleaning up each test, you can define the following methods:

        virtual void SetUp()
        {
            // Code here will be called immediately after the constructor (right
            // before each test).
            printf("Entering set up\n");

            //Set-up corresponds to quadricopter motor control in the following conditions:
            //UAV is in hover flight mode,
            //manual control mode,
            //near ground (so integral gains are deactivated),
            //throttle is above the minimum value such that motor control is activated
            //Only roll axis is tested (TODO: we could test each axis within this fixture)

            rmat[0] = RMAX;
            rmat[1] = 0;
            rmat[2] = 0;
            rmat[3] = 0;
            rmat[4] = RMAX;
            rmat[5] = 0;
            rmat[6] = 0;
            rmat[7] = 0;
            rmat[8] = RMAX;
            dcm_flags._.calib_finished = 1;
            manual_to_auto_ramp = RMAX;
            yaw_control_ramp = RMAX;
            flags._.pitch_feedback = 0;
            current_orientation = F_HOVER;
            flags._.mag_failure = 0;
            flags._.invalid_mag_reading = 0;
            flags._.is_close_to_ground = 0;
            yaw_control = 0;
            roll_control = 0;
            pitch_control = 0;
            flags._.engines_off = 0;
            current_flight_phase = F_MANUAL_TAKE_OFF;
            throttle_hover_control = 0;
            udb_flags._.radio_on = 1;
            udb_pwIn[THROTTLE_HOVER_INPUT_CHANNEL] = 2000 + 1000;
        }

        virtual void TearDown()
        {
          // Code here will be called immediately after each test (right
          // before the destructor).
          printf("Entering tear down\n");
          reset_derivative_terms();
          reset_integral_terms();
          reset_target_orientation();
          rmat[6] = 0;
          rmat[7] = 0;
          rmat[8] = RMAX;
          udb_pwIn[RUDDER_INPUT_CHANNEL] = 0;
        }
        // Objects declared here can be used by all tests in the test case for Foo.
    };

    TEST_F(MotorCntrlIntegralTerms, ComputesCorrectRollGains)
    {
        rmat[6] = 1000;
        motorCntrl();
        ASSERT_EQ(udb_pwOut[MOTOR_A_OUTPUT_CHANNEL], 3270);
        ASSERT_EQ(udb_pwOut[MOTOR_B_OUTPUT_CHANNEL], 3270);
        ASSERT_EQ(udb_pwOut[MOTOR_C_OUTPUT_CHANNEL], 2730);
        ASSERT_EQ(udb_pwOut[MOTOR_D_OUTPUT_CHANNEL], 2730);
    }

    TEST_F(MotorCntrlIntegralTerms, ComputesCorrectPitchGains)
    {
        rmat[7] = -1000;
        motorCntrl();
        ASSERT_EQ(udb_pwOut[MOTOR_A_OUTPUT_CHANNEL], 2730);
        ASSERT_EQ(udb_pwOut[MOTOR_B_OUTPUT_CHANNEL], 3270);
        ASSERT_EQ(udb_pwOut[MOTOR_C_OUTPUT_CHANNEL], 3270);
        ASSERT_EQ(udb_pwOut[MOTOR_D_OUTPUT_CHANNEL], 2730);
    }

    TEST_F(MotorCntrlIntegralTerms, ComputesCorrectYawGains)
    {
        udb_pwIn[RUDDER_INPUT_CHANNEL] = 1000;
        motorCntrl();
        ASSERT_EQ(udb_pwOut[MOTOR_A_OUTPUT_CHANNEL], 2956);
        ASSERT_EQ(udb_pwOut[MOTOR_B_OUTPUT_CHANNEL], 3044);
        ASSERT_EQ(udb_pwOut[MOTOR_C_OUTPUT_CHANNEL], 2956);
        ASSERT_EQ(udb_pwOut[MOTOR_D_OUTPUT_CHANNEL], 3044);
    }

}  // namespace