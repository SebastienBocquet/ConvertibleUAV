#include "../../googletest/googletest/include/gtest/gtest.h"
#include "../MatrixPilot/defines.h"
#include <stdio.h>

TEST(ExponentialFilterTest,filteredEqualInstantaneous){
int16_t x=1;
float x_filtered = 1.;
float invdeltafilter = 80.;
EXPECT_EQ(1.5,exponential_filter(x, &x_filtered, invdeltafilter));
};

int main(int argc,char**argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
