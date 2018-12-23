#include "gtest/gtest.h"
#include "defines.h"
#include <stdio.h>

TEST(MotorControlTest,test1){
motorCntrl();
};

const int max_tilt = (int) (MAX_TILT*.7111) ;
int commanded_tilt_gain;
int mp_argc;
char **mp_argv;

int main(int argc,char**argv)
{
  mp_argc = argc;
  mp_argv = argv;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
