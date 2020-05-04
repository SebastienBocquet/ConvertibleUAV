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
          const int tilt_pwm_eq = BETA_EQ_DEG * (2000.*TILT_THROW_RATIO/(TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG));

          // PID gains
          const uint16_t tilt_ki = (uint16_t)(RMAX*0.0);
          const uint16_t tilt_kp = (uint16_t)(RMAX*TILT_KP);
          const uint16_t tilt_rate_kp = (uint16_t)(RMAX*TILT_RATE_KP);
          const uint16_t tilt_rate_kd = (uint16_t)(RMAX*0.0);
          const uint16_t yaw_ki = (uint16_t)(RMAX*YAW_KI);
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

    TEST_F(TricopterMotorTilt, neutralTilt)
    {
        udb_pwIn[INPUT_CHANNEL_AUX1] = 3000;
        yaw_quad_control = 0;
        motorTiltCntrl();
        ASSERT_NEAR(motor_tilt_servo_pwm_delta, 0, 1);
        ASSERT_NEAR(yawCntrlByTilt(), tilt_pwm_eq, 1);
        motorTiltServoMix1();
        motorTiltServoMix2();
        ASSERT_NEAR(udb_pwOut[MOTOR_TILT_OUTPUT_CHANNEL1], 3000 + REVERSE_IF_NEEDED(MOTOR_TILT_YAW_REVERSED, tilt_pwm_eq), 1);
        ASSERT_NEAR(udb_pwOut[MOTOR_TILT_OUTPUT_CHANNEL2], 3000 - REVERSE_IF_NEEDED(MOTOR_TILT_YAW_REVERSED, tilt_pwm_eq), 1);
    }

    TEST_F(TricopterMotorTilt, motorTiltManual)
    {
        udb_pwIn[INPUT_CHANNEL_AUX1] = 2500;
        yaw_quad_control = 0;
        ASSERT_NEAR(yawCntrlByTilt(), tilt_pwm_eq, 1);
        motorTiltCntrl();
        motorTiltServoMix1();
        motorTiltServoMix2();
        int tilt_pwm = TILT_THROW_RATIO * (udb_pwIn[INPUT_CHANNEL_AUX1] - 3000);
        ASSERT_NEAR(motor_tilt_servo_pwm_delta, tilt_pwm, 1);
    }

    TEST_F(TricopterMotorTilt, motorTiltPwmOutput)
    {
        udb_pwIn[INPUT_CHANNEL_AUX1] = 2500;
        yaw_quad_control = 0;
        motorTiltCntrl();
        motorTiltServoMix1();
        motorTiltServoMix2();
        int tilt_pwm = TILT_THROW_RATIO * (udb_pwIn[INPUT_CHANNEL_AUX1] - 3000);
        ASSERT_NEAR(udb_pwOut[MOTOR_TILT_OUTPUT_CHANNEL1], 3000 + tilt_pwm + REVERSE_IF_NEEDED(MOTOR_TILT_YAW_REVERSED, tilt_pwm_eq), 1);
        ASSERT_NEAR(udb_pwOut[MOTOR_TILT_OUTPUT_CHANNEL2], 3000 + tilt_pwm - REVERSE_IF_NEEDED(MOTOR_TILT_YAW_REVERSED, tilt_pwm_eq), 1);
    }

    TEST_F(TricopterMotorTilt, isInHoveringPos)
    {
        // this input value leads to the minimal tilt pwm
        udb_pwIn[INPUT_CHANNEL_AUX1] = 2000;
        yaw_quad_control = 0;
        motorTiltCntrl();
        ASSERT_TRUE(motorsInHoveringPos());
    }

    TEST_F(TricopterMotorTilt, zeroDifferentialTilt)
    {
        // impose max tilt
        udb_pwIn[INPUT_CHANNEL_AUX1] = 4000;
        yaw_quad_control = 0;
        motorTiltCntrl();
        motorTiltServoMix1();
        motorTiltServoMix2();
        ASSERT_NEAR(udb_pwOut[MOTOR_TILT_OUTPUT_CHANNEL1], udb_pwOut[MOTOR_TILT_OUTPUT_CHANNEL2], 1);
    }

}  // namespace
