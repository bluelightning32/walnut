#include "walnut/point2.h"

#include "gtest/gtest.h"

namespace walnut {

TEST(Point2, GetTwistDirCollinear) {
  Point2 a(0, 0);
  Point2 center(1, 0);
  Point2 b(2, 0);

  EXPECT_EQ(center.GetTwistDir(a, b), 0);
  EXPECT_EQ(center.GetTwistDir(b, a), 0);
}

TEST(Point2, GetTwistDir) {
  //
  // b
  // ^
  // |
  // c ---> a
  //
  Point2 a(1, 0);
  Point2 center(0, 0);
  Point2 b(0, 1);

  // a to b is counter-clockwise
  EXPECT_GT(center.GetTwistDir(a, b), 0);
  // b to a is clockwise
  EXPECT_LT(center.GetTwistDir(b, a), 0);

  // collinear tests
  EXPECT_EQ(center.GetTwistDir(a, a), 0);
  EXPECT_EQ(center.GetTwistDir(b, b), 0);
  EXPECT_EQ(center.GetTwistDir(a, center), 0);
  EXPECT_EQ(center.GetTwistDir(center, a), 0);
}

}  // walnut
