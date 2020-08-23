#include "gtest/gtest.h"
#include "defines.h"
#include <stdio.h>
#include <math.h>


namespace
{
    // The fixture for testing class Foo.
    class TricopterYawControl : public ::testing::Test
    {
      protected:

          // If the constructor and destructor are not enough for setting up
          // and cleaning up each test, you can define the following methods:

          const float TILT_THROW_RATIO = 0.5 * (TILT_THROW_RATIO1 + TILT_THROW_RATIO2);

          //tricopter geometry
          const float k_pitch = EQUIV_R / (2 * R_A * COS_ALPHA);
          const float k_roll = EQUIV_R / (R_A * SIN_ALPHA);
          const float k_tilt = (-2*KQ*K1/(T_EQ_A*R_A*SIN_ALPHA))*(180./M_PI)*(2000. * TILT_THROW_RATIO / (TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG));
          const int yaw_corr_tilt_pwm = BETA_EQ_DEG * 2000. * TILT_THROW_RATIO / (TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG);

          // PID gains
          const uint16_t tilt_ki = (uint16_t)(RMAX*0.0);
          const uint16_t tilt_kp = (uint16_t)(RMAX*TILT_KP);
          const uint16_t tilt_rate_kp = (uint16_t)(RMAX*TILT_RATE_KP);
          const uint16_t tilt_rate_kd = (uint16_t)(RMAX*0.0);
          const uint16_t yaw_ki = (uint16_t)(RMAX*0.0);
          const uint16_t yaw_kp = (uint16_t)(RMAX*YAW_KP);
          const uint16_t yaw_rate_kp = (uint16_t)(RMAX*YAW_RATE_KP);

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

    TEST_F(TricopterYawControl, yawKpGains)
    {
        rmat[1] = 0;
        rmat[6] = 0;
        rmat[7] = 0;
        // set maximal yaw control strength
        udb_pwIn[INPUT_CHANNEL_AUX2] = 4000;
        // initialize yaw control
        dcm_flags._.yaw_init_finished = 0;
        motorCntrl(tilt_kp, tilt_ki, tilt_rate_kp, tilt_rate_kd, yaw_ki, yaw_kp, yaw_rate_kp);
        //apply control
        dcm_flags._.yaw_init_finished = 1;
        rmat[1] = 1010;
        motorCntrl(tilt_kp, tilt_ki, tilt_rate_kp, tilt_rate_kd, yaw_ki, yaw_kp, yaw_rate_kp);

        // Simulate first PID controller
        int yaw_error = 0.25 * rmat[1];
        int desired_yaw = -YAW_KP * yaw_error;
        // Simulate second PID controller
        int yaw_rate_error = -desired_yaw;
        int expected_yaw_quad_control = -YAW_RATE_KP * yaw_rate_error;
        printf("expected yaw quad control %d \n", expected_yaw_quad_control);
        ASSERT_NEAR(yaw_quad_control, expected_yaw_quad_control, 1);
    }

    TEST_F(TricopterYawControl, yawKpGainsLimited)
    {
        rmat[1] = 0;
        rmat[6] = 0;
        rmat[7] = 0;
        // cancel yaw control (minimal value of control strength)
        udb_pwIn[INPUT_CHANNEL_AUX2] = 2000;
        // initialize yaw control
        dcm_flags._.yaw_init_finished = 0;
        motorCntrl(tilt_kp, tilt_ki, tilt_rate_kp, tilt_rate_kd, yaw_ki, yaw_kp, yaw_rate_kp);
        //apply control
        dcm_flags._.yaw_init_finished = 1;
        rmat[1] = 1010;
        motorCntrl(tilt_kp, tilt_ki, tilt_rate_kp, tilt_rate_kd, yaw_ki, yaw_kp, yaw_rate_kp);

        int expected_yaw_quad_control = 0;
        printf("expected yaw quad control %d \n", expected_yaw_quad_control);
        ASSERT_NEAR(yaw_quad_control, expected_yaw_quad_control, 1);
    }

    /* Test front motor tilt for yaw control
     */
    TEST_F(TricopterYawControl, yawControl)
    {
        // impose minimum tilt control to make sure we are in hovering mode
        udb_pwIn[INPUT_CHANNEL_AUX1] = 2000;
        yaw_quad_control = 1000;
        motorTiltCntrl();
        ASSERT_TRUE(motorsInHoveringPos());
        const int tilt_pwm = k_tilt * yaw_quad_control + yaw_corr_tilt_pwm;
        ASSERT_NEAR(yawCntrlByTilt(), tilt_pwm, 1);
    }
}  // namespace
