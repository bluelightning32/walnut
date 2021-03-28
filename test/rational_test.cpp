#include "walnut/rational.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

TEST(rational, RoundUpPosPosPos) {
  BigIntImpl num1(5);
  BigIntImpl denom1(3);
  BigIntImpl denom2(2);

  BigIntImpl num2 = rational::RoundUp(num1, denom1, denom2);
  // 5/3 ~= 1.7 ~= 4/2
  EXPECT_EQ(num2, 4);
}

TEST(rational, RoundUpNegNegNeg) {
  BigIntImpl num1(-5);
  BigIntImpl denom1(-3);
  BigIntImpl denom2(-2);

  BigIntImpl num2 = rational::RoundUp(num1, denom1, denom2);
  // -5/-3 = 5/3 ~= 1.7 ~= -4/-2
  EXPECT_EQ(num2, -4);
}

TEST(rational, RoundUpPosNegNeg) {
  BigIntImpl num1(5);
  BigIntImpl denom1(-3);
  BigIntImpl denom2(-2);

  BigIntImpl num2 = rational::RoundUp(num1, denom1, denom2);
  // 5/-3 = -5/3 ~= -1.6 ~= 3/-2
  EXPECT_EQ(num2, 3);
}

TEST(rational, RoundUpPosPosNeg) {
  BigIntImpl num1(5);
  BigIntImpl denom1(3);
  BigIntImpl denom2(-2);

  BigIntImpl num2 = rational::RoundUp(num1, denom1, denom2);
  // 5/3 ~= 1.7 ~= -4/-2
  EXPECT_EQ(num2, -4);
}

TEST(rational, RoundDownPosPosPos) {
  BigIntImpl num1(5);
  BigIntImpl denom1(3);
  BigIntImpl denom2(2);

  BigIntImpl num2 = rational::RoundDown(num1, denom1, denom2);
  // 5/3 ~= 1.7 ~= 3/2
  EXPECT_EQ(num2, 3);
}

TEST(rational, RoundDownNegNegNeg) {
  BigIntImpl num1(-5);
  BigIntImpl denom1(-3);
  BigIntImpl denom2(-2);

  BigIntImpl num2 = rational::RoundDown(num1, denom1, denom2);
  // -5/-3 = 5/3 ~= 1.7 ~= -3/-2
  EXPECT_EQ(num2, -3);
}

TEST(rational, RoundDownPosNegNeg) {
  BigIntImpl num1(5);
  BigIntImpl denom1(-3);
  BigIntImpl denom2(-2);

  BigIntImpl num2 = rational::RoundDown(num1, denom1, denom2);
  // 5/-3 = -5/3 ~= -1.6 ~= 4/-2
  EXPECT_EQ(num2, 4);
}

TEST(rational, RoundDownPosPosNeg) {
  BigIntImpl num1(5);
  BigIntImpl denom1(3);
  BigIntImpl denom2(-2);

  BigIntImpl num2 = rational::RoundDown(num1, denom1, denom2);
  // 5/3 ~= 1.7 ~= -3/-2
  EXPECT_EQ(num2, -3);
}

TEST(rational, IsLessThanPosDenomPosDenom) {
  // 1/3 < 1/2
  EXPECT_TRUE(rational::IsLessThan(/*num1=*/BigIntImpl{1},
                                   /*denom1=*/BigIntImpl{3},
                                   /*num2=*/BigIntImpl{1},
                                   /*denom2=*/BigIntImpl{2}));

  // !( 1/2 < 1/3 )
  EXPECT_FALSE(rational::IsLessThan(/*num1=*/BigIntImpl{1},
                                    /*denom1=*/BigIntImpl{2},
                                    /*num2=*/BigIntImpl{1},
                                    /*denom2=*/BigIntImpl{3}));

  // 3/2 < 5/2
  EXPECT_TRUE(rational::IsLessThan(/*num1=*/BigIntImpl{3},
                                   /*denom1=*/BigIntImpl{2},
                                   /*num2=*/BigIntImpl{5},
                                   /*denom2=*/BigIntImpl{2}));
}

TEST(rational, IsLessThanPosDenomNegDenom) {
  // 1/1 < -5/-1
  EXPECT_TRUE(rational::IsLessThan(/*num1=*/BigIntImpl{1},
                                   /*denom1=*/BigIntImpl{1},
                                   /*num2=*/BigIntImpl{-5},
                                   /*denom2=*/BigIntImpl{-1}));

  // -1/1 < -1/-1
  EXPECT_TRUE(rational::IsLessThan(/*num1=*/BigIntImpl{-1},
                                   /*denom1=*/BigIntImpl{1},
                                   /*num2=*/BigIntImpl{-1},
                                   /*denom2=*/BigIntImpl{-1}));

  // !( 1/1 < 1/-1 )
  EXPECT_FALSE(rational::IsLessThan(/*num1=*/BigIntImpl{1},
                                    /*denom1=*/BigIntImpl{1},
                                    /*num2=*/BigIntImpl{1},
                                    /*denom2=*/BigIntImpl{-1}));
}

TEST(rational, IsLessThanNegDenomNegDenom) {
  // 1/-1 < -5/-1
  EXPECT_TRUE(rational::IsLessThan(/*num1=*/BigIntImpl{1},
                                   /*denom1=*/BigIntImpl{-1},
                                   /*num2=*/BigIntImpl{-5},
                                   /*denom2=*/BigIntImpl{-1}));

  // !( -1/-1 < 1/-1 )
  EXPECT_FALSE(rational::IsLessThan(/*num1=*/BigIntImpl{-1},
                                    /*denom1=*/BigIntImpl{-1},
                                    /*num2=*/BigIntImpl{1},
                                    /*denom2=*/BigIntImpl{-1}));

  // !( 1/-1 < 1/-1 )
  EXPECT_FALSE(rational::IsLessThan(/*num1=*/BigIntImpl{1},
                                    /*denom1=*/BigIntImpl{-1},
                                    /*num2=*/BigIntImpl{1},
                                    /*denom2=*/BigIntImpl{-1}));
}

}  // walnut
