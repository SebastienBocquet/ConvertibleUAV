#include "gtest/gtest.h"
#include "defines.h"
#include <stdio.h>

namespace {
// The fixture for testing class Foo.
class TxLinearControl : public ::testing::Test {
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

TEST_F(TxLinearControl, ComputeTxLinearControl) {
  int16_t tx_order = 2000;
  int16_t output_min = 0;
  int16_t output_max = 100;
  int16_t reverse_control = 0;

  int16_t output_control = compute_tx_linear_control(
      tx_order, output_min, output_max, reverse_control);
  ASSERT_EQ(output_control, 0);

  tx_order = 4000;
  output_control = compute_tx_linear_control(tx_order, output_min, output_max,
                                             reverse_control);
  ASSERT_EQ(output_control, 100);

  tx_order = 3000;
  output_control = compute_tx_linear_control(tx_order, output_min, output_max,
                                             reverse_control);
  ASSERT_EQ(output_control, 50);

  // Check that min order is recovered even if Tx order is below 2000
  tx_order = 1900;
  output_control = compute_tx_linear_control(tx_order, output_min, output_max,
                                             reverse_control);
  ASSERT_EQ(output_control, 0);

  // Check that max order is recovered even if Tx order is above 4000
  tx_order = 4100;
  output_control = compute_tx_linear_control(tx_order, output_min, output_max,
                                             reverse_control);
  ASSERT_EQ(output_control, 100);
}

}  // namespace
