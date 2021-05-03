#include "walnut/double.h"

#include "gtest/gtest.h"

namespace walnut {

TEST(double, DecomposeOne) {
  int exp;
  int64_t mantissa = Decompose(1.0, &exp);
  EXPECT_EQ(mantissa, 1);
  EXPECT_EQ(exp, 0);
}

TEST(double, DecomposeOneHalf) {
  int exp;
  int64_t mantissa = Decompose(0.5, &exp);
  EXPECT_EQ(mantissa, 1);
  EXPECT_EQ(exp, -1);
}

TEST(double, DecomposeTwo) {
  int exp;
  int64_t mantissa = Decompose(2, &exp);
  EXPECT_EQ(mantissa, 1);
  EXPECT_EQ(exp, 1);
}

TEST(double, DecomposeAllOnes) {
  double value = 0;
  for (int i = 0; i >= -80; --i) {
    value += ldexp(1, i);
  }
  int exp;
  int64_t mantissa = Decompose(value, &exp);
  EXPECT_EQ(ldexp(mantissa, exp), value);
}

}  // walnut
