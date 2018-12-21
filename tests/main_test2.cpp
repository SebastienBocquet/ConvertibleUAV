#include "../../googletest/googletest/include/gtest/gtest.h"
#include "../MatrixPilot/defines.h"
#include "../libSTM/libSTM.h"
#include <stdio.h>

TEST(MotorControlTest,test1){
motorCntrl();
};

int main(int argc,char**argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
