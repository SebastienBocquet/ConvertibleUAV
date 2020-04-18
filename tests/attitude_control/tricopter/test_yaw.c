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

          //tricopter geometry
          const float k_pitch = EQUIV_R / (2 * R_A * COS_ALPHA);
          const float k_roll = EQUIV_R / (R_A * SIN_ALPHA);
          const float t_eq_a = 4.492450879924699;
          const float k_tilt = (-2*KQ*K1/(t_eq_a*R_A*SIN_ALPHA))*(180./M_PI)*(2000. * TILT_THROW_RATIO / (TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG));
          const int tilt_pwm_eq = BETA_EQ_DEG * (180./M_PI) * 2000. * TILT_THROW_RATIO / (TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG);

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

    TEST_F(TricopterYawControl, yawKpGains)
    {
        rmat[1] = 0;
        rmat[6] = 0;
        rmat[7] = 0;
        // set maximal yaw control
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
        int desired_yaw = -3 * yaw_error;
        // Simulate second PID controller
        int yaw_rate_error = -desired_yaw;
        int expected_yaw_quad_control = -1.3 * yaw_rate_error;
        // correct for round-off error in the attitude control
        /* expected_yaw_quad_control += 1; */
        printf("expected yaw quad control %d \n", expected_yaw_quad_control);
        ASSERT_EQ(yaw_quad_control, expected_yaw_quad_control);
    }

    /* Test front motor tilt in neutral position
     */
    TEST_F(TricopterYawControl, noControl)
    {
        udb_pwIn[INPUT_CHANNEL_AUX1] = 3000;
        yaw_quad_control = 0;
        motorTiltCntrl();
        motorTiltServoMix1();
        motorTiltServoMix2();
        ASSERT_EQ(udb_pwOut[MOTOR_TILT_OUTPUT_CHANNEL1], 3000 + tilt_pwm_eq);
        ASSERT_EQ(udb_pwOut[MOTOR_TILT_OUTPUT_CHANNEL2], 3000 - tilt_pwm_eq);
    }

    /* Test front motor tilt for yaw control
     */
    TEST_F(TricopterYawControl, yawControl)
    {
        udb_pwIn[INPUT_CHANNEL_AUX1] = 3000;
        yaw_quad_control = 1000;
        motorTiltCntrl();
        motorTiltServoMix1();
        motorTiltServoMix2();
        const int tilt_pwm = k_tilt * yaw_quad_control + tilt_pwm_eq;
        printf("expected yaw motor tilt pwm %d \n", tilt_pwm);
        ASSERT_EQ(udb_pwOut[MOTOR_TILT_OUTPUT_CHANNEL1], 3000 + tilt_pwm);
        ASSERT_EQ(udb_pwOut[MOTOR_TILT_OUTPUT_CHANNEL2], 3000 - tilt_pwm);
    }
}  // namespace
