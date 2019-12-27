#include "gtest/gtest.h"
#include "defines.h"
#include <stdio.h>


namespace 
{
    // The fixture for testing class Foo.
    class MotorCntrlYaw : public ::testing::Test
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
              flags._.integral_pid_term = 1;
              udb_flags._.radio_on = 1;
              rmat[6] = 0;
              rmat[7] = 0;
          }

          virtual void TearDown() 
          {
              // Code here will be called immediately after each test (right
              // before the destructor).
              printf("Entering tear down\n");
              rmat[1] = 0;
              reset_target_orientation();
              reset_derivative_terms();
              reset_integral_terms();
          }
          // Objects declared here can be used by all tests in the test case for Foo.
    };

    TEST_F(MotorCntrlYaw, noHeadingChange)
    {
        const uint16_t tilt_ki = (uint16_t)(RMAX*0.25);
        const uint16_t tilt_kp = (uint16_t)(RMAX*0.5);
        const uint16_t tilt_rate_kp = (uint16_t)(RMAX*0.22);
        const uint16_t tilt_rate_kd = (uint16_t)(RMAX*0.5);
        const uint16_t yaw_ki = (uint16_t)(RMAX*0.);
        const uint16_t yaw_kp = (uint16_t)(RMAX*0.45);
        const uint16_t yaw_rate_kp = (uint16_t)(RMAX*0.2);
        // maximal yaw control
        udb_pwIn[INPUT_CHANNEL_AUX2] = 4000;

        // initialize yaw control
        dcm_flags._.yaw_init_finished = 0;
        motorCntrl(tilt_kp, tilt_ki, tilt_rate_kp, tilt_rate_kd, yaw_ki, yaw_kp, yaw_rate_kp);
        //apply control
        dcm_flags._.yaw_init_finished = 1;
        motorCntrl(tilt_kp, tilt_ki, tilt_rate_kp, tilt_rate_kd, yaw_ki, yaw_kp, yaw_rate_kp);
        
        /* for (int i=0; i<9; i++){ */
        /*     printf("rmat[%d] %d \n", i, rmat[i]); */
        /* } */
        ASSERT_EQ(udb_pwOut[MOTOR_A_OUTPUT_CHANNEL], 3000);
        ASSERT_EQ(udb_pwOut[MOTOR_B_OUTPUT_CHANNEL], 3000);
        ASSERT_EQ(udb_pwOut[MOTOR_C_OUTPUT_CHANNEL], 3000);
        ASSERT_EQ(udb_pwOut[MOTOR_D_OUTPUT_CHANNEL], 3000);
    }

    TEST_F(MotorCntrlYaw, noYawControl)
    {
        const uint16_t tilt_ki = (uint16_t)(RMAX*0.25);
        const uint16_t tilt_kp = (uint16_t)(RMAX*0.5);
        const uint16_t tilt_rate_kp = (uint16_t)(RMAX*0.22);
        const uint16_t tilt_rate_kd = (uint16_t)(RMAX*0.5);
        const uint16_t yaw_ki = (uint16_t)(RMAX*0.);
        const uint16_t yaw_kp = (uint16_t)(RMAX*0.45);
        const uint16_t yaw_rate_kp = (uint16_t)(RMAX*0.2);
        // maximal yaw control
        udb_pwIn[INPUT_CHANNEL_AUX2] = 2000;

        // initialize yaw control
        dcm_flags._.yaw_init_finished = 0;
        motorCntrl(tilt_kp, tilt_ki, tilt_rate_kp, tilt_rate_kd, yaw_ki, yaw_kp, yaw_rate_kp);
        //apply control
        dcm_flags._.yaw_init_finished = 1;
        rmat[1] = 1000;
        motorCntrl(tilt_kp, tilt_ki, tilt_rate_kp, tilt_rate_kd, yaw_ki, yaw_kp, yaw_rate_kp);
  
        ASSERT_EQ(udb_pwOut[MOTOR_A_OUTPUT_CHANNEL], 3000);
        ASSERT_EQ(udb_pwOut[MOTOR_B_OUTPUT_CHANNEL], 3000);
        ASSERT_EQ(udb_pwOut[MOTOR_C_OUTPUT_CHANNEL], 3000);
        ASSERT_EQ(udb_pwOut[MOTOR_D_OUTPUT_CHANNEL], 3000);
    }

    TEST_F(MotorCntrlYaw, yawKp)
    {
        const uint16_t tilt_ki = (uint16_t)(RMAX*0.25);
        const uint16_t tilt_kp = (uint16_t)(RMAX*0.5);
        const uint16_t tilt_rate_kp = (uint16_t)(RMAX*0.22);
        const uint16_t tilt_rate_kd = (uint16_t)(RMAX*0.5);
        const uint16_t yaw_ki = (uint16_t)(RMAX*0.);
        const uint16_t yaw_kp = (uint16_t)(RMAX*0.45);
        const uint16_t yaw_rate_kp = (uint16_t)(RMAX*0.2);
        // maximal yaw control
        udb_pwIn[INPUT_CHANNEL_AUX2] = 4000;

        // initialize yaw control
        dcm_flags._.yaw_init_finished = 0;
        motorCntrl(tilt_kp, tilt_ki, tilt_rate_kp, tilt_rate_kd, yaw_ki, yaw_kp, yaw_rate_kp);
        //apply control
        dcm_flags._.yaw_init_finished = 1;
        rmat[1] = 1000;
        motorCntrl(tilt_kp, tilt_ki, tilt_rate_kp, tilt_rate_kd, yaw_ki, yaw_kp, yaw_rate_kp);
 
        // Simulate first PID controller
        int yaw_error = 0.25 * rmat[1];
        int desired_yaw = -0.45 * yaw_error;
        // Simulate second PID controller
        int yaw_rate_error = -desired_yaw;
        int yaw_quad_control = -0.2 * yaw_rate_error;
        printf("computed expected motor control %d \n", yaw_quad_control);
        
        ASSERT_EQ(udb_pwOut[MOTOR_A_OUTPUT_CHANNEL], 3000+yaw_quad_control);
        ASSERT_EQ(udb_pwOut[MOTOR_B_OUTPUT_CHANNEL], 3000-yaw_quad_control);
        ASSERT_EQ(udb_pwOut[MOTOR_C_OUTPUT_CHANNEL], 3000+yaw_quad_control);
        ASSERT_EQ(udb_pwOut[MOTOR_D_OUTPUT_CHANNEL], 3000-yaw_quad_control);
    }
}  // namespace
