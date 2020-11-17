#include "walnut/homo_point3.h"

#include "gtest/gtest.h"

namespace walnut {

TEST(HomoPoint3, EqualityOperator) {
  HomoPoint3<> p(0, 1, 2, 7);
  EXPECT_EQ(p, p);

  // Test an equivalent point with double the w.
  {
    HomoPoint3<> compare(0, 2, 4, 14);
    EXPECT_EQ(p, compare);
    EXPECT_EQ(compare, p);
  }

  // Test an equivalent point with opposite w.
  {
    HomoPoint3<> compare(0, -1, -2, -7);
    EXPECT_EQ(p, compare);
    EXPECT_EQ(compare, p);
  }

  // Test a different point
  {
    HomoPoint3<> compare(0, 1, 1, 7);
    EXPECT_NE(p, compare);
    EXPECT_NE(compare, p);
  }
}

TEST(HomoPoint3, LexicographicallyLtBasic) {
  HomoPoint3<> p1(0, 0, 1, 1);
  HomoPoint3<> p2(0, 0, 2, 1);
  EXPECT_TRUE(HomoPoint3<>::LexicographicallyLt(p1, p2));
  EXPECT_FALSE(HomoPoint3<>::LexicographicallyLt(p2, p1));
}

TEST(HomoPoint3, LexicographicallyLtDifferentDist) {
  HomoPoint3<> p1(0, 0, 3, 5);
  HomoPoint3<> p2(0, 0, 3, 2);
  EXPECT_TRUE(HomoPoint3<>::LexicographicallyLt(p1, p2));
  EXPECT_FALSE(HomoPoint3<>::LexicographicallyLt(p2, p1));
}

TEST(HomoPoint3, LexicographicallyLtDifferentDistSign) {
  HomoPoint3<> p1(0, 0, 1, 1);
  HomoPoint3<> p2(0, 0, -2, -1);
  EXPECT_TRUE(HomoPoint3<>::LexicographicallyLt(p1, p2));
  EXPECT_FALSE(HomoPoint3<>::LexicographicallyLt(p2, p1));
}

}  // walnut