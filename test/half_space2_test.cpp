#include "walnut/half_space2.h"

#include "gtest/gtest.h"

#include "walnut/point2.h"

namespace walnut {

TEST(HalfSpace2, ComparePoint2) {
  // Anything with x>5 is included in the half space.
  HalfSpace2 half_space(/*normal=*/Vector2(/*x=*/2, /*y=*/0), /*dist=*/BigInt<32>(10));

  EXPECT_TRUE(half_space.normal().IsSameDir(Vector2(1, 0)));

  // excluded
  EXPECT_LT(half_space.Compare(Point2(/*x=*/1, /*y=*/100)), 0);
  // coincident
  EXPECT_EQ(half_space.Compare(Point2(/*x=*/5, /*y=*/100)), 0);
  // included
  EXPECT_GT(half_space.Compare(Point2(/*x=*/6, /*y=*/100)), 0);
}

TEST(HalfSpace3, CompareHomoPoint2) {
  // Anything with x>5 is included in the half space.
  HalfSpace2 half_space(/*normal=*/Vector2(/*x=*/2, /*y=*/0), /*dist=*/BigInt<32>(10));

  // excluded
  EXPECT_LT(half_space.Compare(HomoPoint2(/*x=*/1, /*y=*/100, /*w=*/1)), 0);
  EXPECT_LT(half_space.Compare(HomoPoint2(/*x=*/9, /*y=*/100, /*w=*/2)), 0);
  // coincident
  EXPECT_EQ(half_space.Compare(HomoPoint2(/*x=*/5, /*y=*/100, /*w=*/1)), 0);
  EXPECT_EQ(half_space.Compare(HomoPoint2(/*x=*/10, /*y=*/100, /*w=*/2)), 0);
  // included
  EXPECT_GT(half_space.Compare(HomoPoint2(/*x=*/6, /*y=*/100, /*w=*/1)), 0);
  EXPECT_GT(half_space.Compare(HomoPoint2(/*x=*/11, /*y=*/100, /*w=*/2)), 0);

  // excluded
  EXPECT_LT(half_space.Compare(HomoPoint2(/*x=*/-1, /*y=*/100, /*w=*/-1)), 0);
}

TEST(HalfSpace2, BuildFromPoints) {
  // Build from the line:
  // [5, 0], [6, -1]
  //
  // Anything with z<5 is included in the half space.
  HalfSpace2 half_space(/*p1=*/Point2(6, -1),
                          /*p2=*/Point2(5, 0));

  EXPECT_TRUE(half_space.normal().IsSameDir(Vector2(-1, -1)));

  // excluded
  EXPECT_LT(half_space.Compare(Point2(/*x=*/3, /*y=*/3)), 0);
  // coincident
  EXPECT_EQ(half_space.Compare(Point2(/*x=*/2, /*y=*/3)), 0);
  // included
  EXPECT_GT(half_space.Compare(Point2(/*x=*/1, /*y=*/1)), 0);
}

TEST(HalfSpace2, HalfSpacesDistinct) {
  EXPECT_NE(HalfSpace2(/*x=*/0, /*y=*/0, /*dist=*/10),
            HalfSpace2(/*x=*/0, /*y=*/0, /*dist=*/-10));
}

}  // walnut
