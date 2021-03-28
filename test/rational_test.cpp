#include "walnut/rational.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

TEST(rational, RoundUpPosPosPos) {
  BigInt num1(5);
  BigInt denom1(3);
  BigInt denom2(2);

  BigInt num2 = rational::RoundUp(num1, denom1, denom2);
  // 5/3 ~= 1.7 ~= 4/2
  EXPECT_EQ(num2, 4);
}

TEST(rational, RoundUpNegNegNeg) {
  BigInt num1(-5);
  BigInt denom1(-3);
  BigInt denom2(-2);

  BigInt num2 = rational::RoundUp(num1, denom1, denom2);
  // -5/-3 = 5/3 ~= 1.7 ~= -4/-2
  EXPECT_EQ(num2, -4);
}

TEST(rational, RoundUpPosNegNeg) {
  BigInt num1(5);
  BigInt denom1(-3);
  BigInt denom2(-2);

  BigInt num2 = rational::RoundUp(num1, denom1, denom2);
  // 5/-3 = -5/3 ~= -1.6 ~= 3/-2
  EXPECT_EQ(num2, 3);
}

TEST(rational, RoundUpPosPosNeg) {
  BigInt num1(5);
  BigInt denom1(3);
  BigInt denom2(-2);

  BigInt num2 = rational::RoundUp(num1, denom1, denom2);
  // 5/3 ~= 1.7 ~= -4/-2
  EXPECT_EQ(num2, -4);
}

TEST(rational, RoundDownPosPosPos) {
  BigInt num1(5);
  BigInt denom1(3);
  BigInt denom2(2);

  BigInt num2 = rational::RoundDown(num1, denom1, denom2);
  // 5/3 ~= 1.7 ~= 3/2
  EXPECT_EQ(num2, 3);
}

TEST(rational, RoundDownNegNegNeg) {
  BigInt num1(-5);
  BigInt denom1(-3);
  BigInt denom2(-2);

  BigInt num2 = rational::RoundDown(num1, denom1, denom2);
  // -5/-3 = 5/3 ~= 1.7 ~= -3/-2
  EXPECT_EQ(num2, -3);
}

TEST(rational, RoundDownPosNegNeg) {
  BigInt num1(5);
  BigInt denom1(-3);
  BigInt denom2(-2);

  BigInt num2 = rational::RoundDown(num1, denom1, denom2);
  // 5/-3 = -5/3 ~= -1.6 ~= 4/-2
  EXPECT_EQ(num2, 4);
}

TEST(rational, RoundDownPosPosNeg) {
  BigInt num1(5);
  BigInt denom1(3);
  BigInt denom2(-2);

  BigInt num2 = rational::RoundDown(num1, denom1, denom2);
  // 5/3 ~= 1.7 ~= -3/-2
  EXPECT_EQ(num2, -3);
}

TEST(rational, IsLessThanPosDenomPosDenom) {
  // 1/3 < 1/2
  EXPECT_TRUE(rational::IsLessThan(/*num1=*/BigInt{1},
                                   /*denom1=*/BigInt{3},
                                   /*num2=*/BigInt{1},
                                   /*denom2=*/BigInt{2}));

  // !( 1/2 < 1/3 )
  EXPECT_FALSE(rational::IsLessThan(/*num1=*/BigInt{1},
                                    /*denom1=*/BigInt{2},
                                    /*num2=*/BigInt{1},
                                    /*denom2=*/BigInt{3}));

  // 3/2 < 5/2
  EXPECT_TRUE(rational::IsLessThan(/*num1=*/BigInt{3},
                                   /*denom1=*/BigInt{2},
                                   /*num2=*/BigInt{5},
                                   /*denom2=*/BigInt{2}));
}

TEST(rational, IsLessThanPosDenomNegDenom) {
  // 1/1 < -5/-1
  EXPECT_TRUE(rational::IsLessThan(/*num1=*/BigInt{1},
                                   /*denom1=*/BigInt{1},
                                   /*num2=*/BigInt{-5},
                                   /*denom2=*/BigInt{-1}));

  // -1/1 < -1/-1
  EXPECT_TRUE(rational::IsLessThan(/*num1=*/BigInt{-1},
                                   /*denom1=*/BigInt{1},
                                   /*num2=*/BigInt{-1},
                                   /*denom2=*/BigInt{-1}));

  // !( 1/1 < 1/-1 )
  EXPECT_FALSE(rational::IsLessThan(/*num1=*/BigInt{1},
                                    /*denom1=*/BigInt{1},
                                    /*num2=*/BigInt{1},
                                    /*denom2=*/BigInt{-1}));
}

TEST(rational, IsLessThanNegDenomNegDenom) {
  // 1/-1 < -5/-1
  EXPECT_TRUE(rational::IsLessThan(/*num1=*/BigInt{1},
                                   /*denom1=*/BigInt{-1},
                                   /*num2=*/BigInt{-5},
                                   /*denom2=*/BigInt{-1}));

  // !( -1/-1 < 1/-1 )
  EXPECT_FALSE(rational::IsLessThan(/*num1=*/BigInt{-1},
                                    /*denom1=*/BigInt{-1},
                                    /*num2=*/BigInt{1},
                                    /*denom2=*/BigInt{-1}));

  // !( 1/-1 < 1/-1 )
  EXPECT_FALSE(rational::IsLessThan(/*num1=*/BigInt{1},
                                    /*denom1=*/BigInt{-1},
                                    /*num2=*/BigInt{1},
                                    /*denom2=*/BigInt{-1}));
}

}  // walnut
