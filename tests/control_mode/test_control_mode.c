#include "gtest/gtest.h"
#include "defines.h"
#include "mode_switch.h"
#include <stdio.h>

namespace {
// The fixture for testing class Foo.
class ControlMode : public ::testing::Test {
 protected:
  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
    printf("Entering set up\n");
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
    printf("Entering tear down\n");
  }
  // Objects declared here can be used by all tests in the test case for Foo.
};

TEST_F(ControlMode, IsInManualMode) {
  flight_mode_switch_check_set();
  ASSERT_EQ(flags._.man_req, 1);
  ASSERT_EQ(flags._.auto_req, 0);
  ASSERT_EQ(flags._.home_req, 0);
}

}  // namespace
