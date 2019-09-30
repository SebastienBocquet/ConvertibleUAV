#include "gtest/gtest.h"
#include "defines.h"
#include <stdio.h>

namespace 
{
    // The fixture for testing class Foo.
    class IntScaling : public ::testing::Test
    {
      protected:

        // If the constructor and destructor are not enough for setting up
        // and cleaning up each test, you can define the following methods:
    
        virtual void SetUp() 
        {
            // Code here will be called immediately after the constructor (right
            // before each test).
            printf("Entering set up\n");
        }

        virtual void TearDown() 
        {
          // Code here will be called immediately after each test (right
          // before the destructor).
          printf("Entering tear down\n");
        }
        // Objects declared here can be used by all tests in the test case for Foo.
    };

    TEST_F(IntScaling, ComputeIntScaling)
    {
        int16_t arg1 = 0;
        int16_t arg2 = 1;

        int16_t output = int_scale(arg1, arg2);
        ASSERT_EQ(output, 0);

        arg1 = 1;
        arg2 = RMAX;
        output = int_scale(arg1, arg2);
        ASSERT_EQ(output, 1);

        arg1 = RMAX;
        arg2 = RMAX;
        output = int_scale(arg1, arg2);
        ASSERT_EQ(output, RMAX);
    }

}  // namespace
