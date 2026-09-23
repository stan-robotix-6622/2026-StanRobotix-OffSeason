#include <iostream>
#include <gtest/gtest.h>
#include <frc/MathUtil.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>

TEST(SanityTest, BasicAssertion) {
  EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
