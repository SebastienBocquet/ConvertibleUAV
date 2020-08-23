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

          //yaw_corr_tilt_pwm is a shift around the manual tilt_pwm value.
          const float TILT_THROW_RATIO = 0.5 * (TILT_THROW_RATIO1 + TILT_THROW_RATIO2);
          const int yaw_corr_tilt_pwm = BETA_EQ_DEG * (2000.*TILT_THROW_RATIO/(TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG));
          //tilt angle in forward flight mode.
          const int BETA_FORWARD_FLIGHT = 90;

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

    TEST_F(TricopterMotorTilt, motorTiltManualHovering)
    {
        // small input value to be in hovering mode
        udb_pwIn[INPUT_CHANNEL_AUX1] = 2200;
        yaw_quad_control = 0;
        motorTiltCntrl();
        motorTiltServoMix1();
        motorTiltServoMix2();

        // we assume we are in hovering mode
        ASSERT_TRUE(motorsInHoveringPos());
        // if yaw_quad_control is zero, yawCntrlByTilt() == yaw_corr_tilt_pwm
        ASSERT_NEAR(yawCntrlByTilt(), yaw_corr_tilt_pwm, 1);
        int tilt_pwm1 = TILT_THROW_RATIO1 * (udb_pwIn[INPUT_CHANNEL_AUX1] - 3000);
        ASSERT_NEAR(motor_tilt_servo_pwm_delta1, tilt_pwm1, 1);
        int tilt_pwm2 = TILT_THROW_RATIO2 * (udb_pwIn[INPUT_CHANNEL_AUX1] - 3000);
        ASSERT_NEAR(motor_tilt_servo_pwm_delta2, tilt_pwm2, 1);
    }

    TEST_F(TricopterMotorTilt, motorTiltManualForwardFlight)
    {
        // must be larger than TILT_PWM_TRANSITON / TILT_THROW_RATIO + 3000 
        udb_pwIn[INPUT_CHANNEL_AUX1] = 3500;
        motorTiltCntrl();
        motorTiltServoMix1();
        motorTiltServoMix2();
        const int tilt_pwm_forward_flight = (1000*TILT_THROW_RATIO/(TILT_MAX_ANGLE_DEG - TILT_MIN_ANGLE_DEG)) * (2 * BETA_FORWARD_FLIGHT - TILT_MIN_ANGLE_DEG - TILT_MAX_ANGLE_DEG);
        ASSERT_NEAR(motor_tilt_servo_pwm_delta1, tilt_pwm_forward_flight, 1);
        ASSERT_NEAR(motor_tilt_servo_pwm_delta2, tilt_pwm_forward_flight, 1);
        //check that the two front motors have equal tilt angle.
        ASSERT_NEAR(udb_pwOut[MOTOR_TILT_OUTPUT_CHANNEL1], udb_pwOut[MOTOR_TILT_OUTPUT_CHANNEL2], 1);
    }

    TEST_F(TricopterMotorTilt, motorTiltPwmOutput)
    {
        udb_pwIn[INPUT_CHANNEL_AUX1] = 2500;
        // if yaw_quad_control is zero, yawCntrlByTilt() == yaw_corr_tilt_pwm
        yaw_quad_control = 0;
        motorTiltCntrl();
        motorTiltServoMix1();
        motorTiltServoMix2();
        int tilt_pwm1 = TILT_THROW_RATIO1 * (udb_pwIn[INPUT_CHANNEL_AUX1] - 3000);
        ASSERT_NEAR(udb_pwOut[MOTOR_TILT_OUTPUT_CHANNEL1], 3000 + tilt_pwm1 + REVERSE_IF_NEEDED(MOTOR_TILT_YAW_REVERSED, yawCntrlByTilt()), 1);
        int tilt_pwm2 = TILT_THROW_RATIO2 * (udb_pwIn[INPUT_CHANNEL_AUX1] - 3000);
        ASSERT_NEAR(udb_pwOut[MOTOR_TILT_OUTPUT_CHANNEL2], 3000 + tilt_pwm2 - REVERSE_IF_NEEDED(MOTOR_TILT_YAW_REVERSED, yawCntrlByTilt()), 1);
    }

    TEST_F(TricopterMotorTilt, isInHoveringPos)
    {
        // this input value leads to the minimal tilt pwm
        udb_pwIn[INPUT_CHANNEL_AUX1] = 2000;
        yaw_quad_control = 0;
        motorTiltCntrl();
        ASSERT_TRUE(motorsInHoveringPos());
    }

    TEST_F(TricopterMotorTilt, noYawControlInPlaneMode)
    {
        // large input value to be in plane mode
        udb_pwIn[INPUT_CHANNEL_AUX1] = 3900;
        yaw_quad_control = 1000;

        motorTiltCntrl();
        ASSERT_TRUE(!motorsInHoveringPos()); 
        ASSERT_NEAR(yawCntrlByTilt(), 0, 1);
    }


}  // namespace
