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

}  // walnut
