#include "gtest/gtest.h"
#include "defines.h"
#include <stdio.h>

TEST(MotorControlTest,test1){
  dcm_flags._.calib_finished = 1;
  rmat[4] = 50;
  motorCntrl();
  ASSERT_EQ(udb_pwOut[MOTOR_A_OUTPUT_CHANNEL], 0);
};

const int max_tilt = (int) (MAX_TILT*.7111) ;
int commanded_tilt_gain;
int mp_argc;
char **mp_argv;

int main(int argc,char**argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
