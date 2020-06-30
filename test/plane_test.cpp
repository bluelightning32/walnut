#include "walnut/plane.h"

#include "gtest/gtest.h"

namespace walnut {

TEST(Plane, CompareVertex3) {
  // Anything with x<5 is included in the half space.
  Plane<> plane(/*normal=*/Vector<>(/*x=*/2, /*y=*/0, /*z=*/0), /*dist=*/BigInt<32>(10));

  EXPECT_TRUE(plane.normal().IsSameDir(Vector<>(1, 0, 0)));

  // included
  EXPECT_GT(plane.Compare(Vertex3<>(/*x=*/1, /*y=*/100, /*z=*/100)), 0);
  // coincident
  EXPECT_EQ(plane.Compare(Vertex3<>(/*x=*/5, /*y=*/100, /*z=*/100)), 0);
  // excluded
  EXPECT_LT(plane.Compare(Vertex3<>(/*x=*/6, /*y=*/100, /*z=*/100)), 0);
}

TEST(Plane, CompareVertex4) {
  // Anything with x<5 is included in the half space.
  Plane<> plane(/*normal=*/Vector<>(/*x=*/2, /*y=*/0, /*z=*/0), /*dist=*/BigInt<32>(10));

  // included
  EXPECT_GT(plane.Compare(Vertex4<>(/*x=*/1, /*y=*/100, /*z=*/100, /*w=*/1)), 0);
  EXPECT_GT(plane.Compare(Vertex4<>(/*x=*/9, /*y=*/100, /*z=*/100, /*w=*/2)), 0);
  // coincident
  EXPECT_EQ(plane.Compare(Vertex4<>(/*x=*/5, /*y=*/100, /*z=*/100, /*w=*/1)), 0);
  EXPECT_EQ(plane.Compare(Vertex4<>(/*x=*/10, /*y=*/100, /*z=*/100, /*w=*/2)), 0);
  // excluded
  EXPECT_LT(plane.Compare(Vertex4<>(/*x=*/6, /*y=*/100, /*z=*/100, /*w=*/1)), 0);
  EXPECT_LT(plane.Compare(Vertex4<>(/*x=*/11, /*y=*/100, /*z=*/100, /*w=*/2)), 0);
}

TEST(Plane, BuildFromVertexes) {
  // Build from the triangle:
  // [0, 0, 5], [1, 0, 5], [0, 1, 5]
  //
  // Anything with z<5 is included in the half space.
  Plane<> plane(/*p1=*/Vertex3<>(0, 0, 5),
                /*p2=*/Vertex3<>(1, 0, 5),
                /*p3=*/Vertex3<>(0, 1, 5));

  EXPECT_TRUE(plane.normal().IsSameDir(Vector<>(0, 0, 1)));

  // included
  EXPECT_GT(plane.Compare(Vertex3<>(/*x=*/100, /*y=*/100, /*z=*/1)), 0);
  // coincident
  EXPECT_EQ(plane.Compare(Vertex3<>(/*x=*/500, /*y=*/100, /*z=*/5)), 0);
  // excluded
  EXPECT_LT(plane.Compare(Vertex3<>(/*x=*/600, /*y=*/100, /*z=*/6)), 0);
}

}  // walnut
