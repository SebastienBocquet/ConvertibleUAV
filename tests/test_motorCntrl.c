#include "gtest/gtest.h"
#include "defines.h"
#include <stdio.h>

namespace 
{
    // The fixture for testing class Foo.
    class MotorCntrlTest : public ::testing::Test
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
            manual_to_auto_ramp = RMAX;
            yaw_control_ramp = RMAX;
            flags._.pitch_feedback = 0;
            current_orientation = F_HOVER;
            flags._.mag_failure = 0;
            flags._.invalid_mag_reading = 0;
            flags._.is_close_to_ground = 1;
            yaw_control = 0;
            roll_control = 0;
            pitch_control = 0;
            flags._.engines_off = 0;
            current_flight_phase = F_MANUAL_TAKE_OFF;
            throttle_hover_control = 0;
            udb_flags._.radio_on = 1;
            udb_pwIn[THROTTLE_HOVER_INPUT_CHANNEL] = 2000 + 1000;
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

    TEST_F(MotorCntrlTest, MotorCntrlComputesCorrectProportionalGain)
    {
        motorCntrl();
        ASSERT_EQ(udb_pwOut[MOTOR_B_OUTPUT_CHANNEL], 2000);
    }

}  // namespace

const int max_tilt = (int) (MAX_TILT*.7111) ;
int commanded_tilt_gain;
int mp_argc;
char **mp_argv;

int main(int argc, char **argv) 
{
#ifdef TestGains
    printf("toto");
#endif
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
