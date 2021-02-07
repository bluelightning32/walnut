#include "walnut/rational.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

TEST(rational, RoundUpPosPosPos) {
  BigInt<64> num1(5);
  BigInt<64> denom1(3);
  BigInt<64> denom2(2);

  BigInt<64> num2 = rational::RoundUp(num1, denom1, denom2);
  // 5/3 ~= 1.7 ~= 4/2
  EXPECT_EQ(num2, 4);
}

TEST(rational, RoundUpNegNegNeg) {
  BigInt<64> num1(-5);
  BigInt<64> denom1(-3);
  BigInt<64> denom2(-2);

  BigInt<64> num2 = rational::RoundUp(num1, denom1, denom2);
  // -5/-3 = 5/3 ~= 1.7 ~= -4/-2
  EXPECT_EQ(num2, -4);
}

TEST(rational, RoundUpPosNegNeg) {
  BigInt<64> num1(5);
  BigInt<64> denom1(-3);
  BigInt<64> denom2(-2);

  BigInt<64> num2 = rational::RoundUp(num1, denom1, denom2);
  // 5/-3 = -5/3 ~= -1.6 ~= 3/-2
  EXPECT_EQ(num2, 3);
}

TEST(rational, RoundUpPosPosNeg) {
  BigInt<64> num1(5);
  BigInt<64> denom1(3);
  BigInt<64> denom2(-2);

  BigInt<64> num2 = rational::RoundUp(num1, denom1, denom2);
  // 5/3 ~= 1.7 ~= -4/-2
  EXPECT_EQ(num2, -4);
}

TEST(rational, RoundDownPosPosPos) {
  BigInt<64> num1(5);
  BigInt<64> denom1(3);
  BigInt<64> denom2(2);

  BigInt<64> num2 = rational::RoundDown(num1, denom1, denom2);
  // 5/3 ~= 1.7 ~= 3/2
  EXPECT_EQ(num2, 3);
}

TEST(rational, RoundDownNegNegNeg) {
  BigInt<64> num1(-5);
  BigInt<64> denom1(-3);
  BigInt<64> denom2(-2);

  BigInt<64> num2 = rational::RoundDown(num1, denom1, denom2);
  // -5/-3 = 5/3 ~= 1.7 ~= -3/-2
  EXPECT_EQ(num2, -3);
}

TEST(rational, RoundDownPosNegNeg) {
  BigInt<64> num1(5);
  BigInt<64> denom1(-3);
  BigInt<64> denom2(-2);

  BigInt<64> num2 = rational::RoundDown(num1, denom1, denom2);
  // 5/-3 = -5/3 ~= -1.6 ~= 4/-2
  EXPECT_EQ(num2, 4);
}

TEST(rational, RoundDownPosPosNeg) {
  BigInt<64> num1(5);
  BigInt<64> denom1(3);
  BigInt<64> denom2(-2);

  BigInt<64> num2 = rational::RoundDown(num1, denom1, denom2);
  // 5/3 ~= 1.7 ~= -3/-2
  EXPECT_EQ(num2, -3);
}

}  // walnut
