#include "gtest/gtest.h"
#include "defines.h"
#include <stdio.h>
#include <math.h>


namespace 
{
    // The fixture for testing class Foo.
    class TricopterMotorCntrlTilt : public ::testing::Test
    {
      protected:

          // If the constructor and destructor are not enough for setting up
          // and cleaning up each test, you can define the following methods:
    
          //tricopter geometry
          const float alpha = 60.*M_PI/180;
          const float R = 0.25;
          const float R_A = 0.25;
          const float R_B = 0.25;
          const float k_pitch = R / (2 * R_A * cos(alpha));
          const float k_roll = R / (R_A * sin(alpha));

          // PID gains
          const uint16_t tilt_ki = (uint16_t)(RMAX*0.0);
          const uint16_t tilt_kp = (uint16_t)(RMAX*0.5);
          const uint16_t tilt_rate_kp = (uint16_t)(RMAX*0.22);
          const uint16_t tilt_rate_kd = (uint16_t)(RMAX*0.0);
          const uint16_t yaw_ki = (uint16_t)(RMAX*0.0);
          const uint16_t yaw_kp = (uint16_t)(RMAX*0.0);
          const uint16_t yaw_rate_kp = (uint16_t)(RMAX*0.0);

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

    TEST_F(TricopterMotorCntrlTilt, noControl)
    {
        rmat[6] = 0;
        rmat[7] = 0;
        motorCntrl(tilt_kp, tilt_ki, tilt_rate_kp, tilt_rate_kd, yaw_ki, yaw_kp, yaw_rate_kp);
        ASSERT_EQ(udb_pwOut[MOTOR_A_OUTPUT_CHANNEL], 3000);
        ASSERT_EQ(udb_pwOut[MOTOR_B_OUTPUT_CHANNEL], 3000);
        ASSERT_EQ(udb_pwOut[MOTOR_C_OUTPUT_CHANNEL], 3000);
    }

    TEST_F(TricopterMotorCntrlTilt, rollKpGains)
    {
        rmat[6] = 1000;
        rmat[7] = 0;
        motorCntrl(tilt_kp, tilt_ki, tilt_rate_kp, tilt_rate_kd, yaw_ki, yaw_kp, yaw_rate_kp);
       
        // roll control corresponding to this tricopter geometry and these rmat values
        const int roll_quad_control = -109;
        // Scale motor order for a tricopter
        const int throttle_A = -k_roll * roll_quad_control;
        const int throttle_B = 0.;
        const int throttle_C = k_roll * roll_quad_control;
        printf("expected motor control A %d \n", throttle_A);
        printf("expected motor control B %d \n", throttle_B);
        printf("expected motor control C %d \n", throttle_C);
        ASSERT_EQ(udb_pwOut[MOTOR_A_OUTPUT_CHANNEL], 3000 + throttle_A);
        ASSERT_EQ(udb_pwOut[MOTOR_B_OUTPUT_CHANNEL], 3000 + throttle_B);
        ASSERT_EQ(udb_pwOut[MOTOR_C_OUTPUT_CHANNEL], 3000 + throttle_C);
    }

    TEST_F(TricopterMotorCntrlTilt, pitchKpGains)
    {
        rmat[6] = 0;
        rmat[7] = 1000;
        motorCntrl(tilt_kp, tilt_ki, tilt_rate_kp, tilt_rate_kd, yaw_ki, yaw_kp, yaw_rate_kp);
        
        // Simulate first PID controller
        const int pitch_error = -rmat[7];
        const int desired_pitch = -0.5 * pitch_error;
        // Simulate second PID controller
        const int pitch_rate_error = -desired_pitch;
        const int pitch_quad_control = -0.22 * pitch_rate_error;
       
        // Scale motor order for a tricopter
        const int throttle_A = k_pitch * pitch_quad_control;
        const int throttle_B = -2 * throttle_A;
        const int throttle_C = k_pitch * pitch_quad_control;
        printf("expected motor control A %d \n", throttle_A);
        printf("expected motor control B %d \n", throttle_B);
        printf("expected motor control C %d \n", throttle_C);
        ASSERT_EQ(udb_pwOut[MOTOR_A_OUTPUT_CHANNEL], 3000 + throttle_A);
        ASSERT_EQ(udb_pwOut[MOTOR_B_OUTPUT_CHANNEL], 3000 + throttle_B);
        ASSERT_EQ(udb_pwOut[MOTOR_C_OUTPUT_CHANNEL], 3000 + throttle_C);
    }
}  // namespace
