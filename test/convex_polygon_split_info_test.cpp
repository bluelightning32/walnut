#include "walnut/convex_polygon_split_info.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

TEST(ConvexPolygonSplitInfo, SplitSquareAtVertices) {
  // 
  // p0 ----> p1
  // ^ \_ pos |
  // |   \__  |
  // | neg  \ v
  // p3 <---- p2

  ConvexPolygonSplitInfo info;
  // The postive-only range starts at p1, because p0 is shared
  info.ranges.pos_range.first = 1;
  // The postive-only range ends before p2, because p2 is shared
  info.ranges.pos_range.second = 2;
  info.ranges.neg_range.first = 3;
  info.ranges.neg_range.second = 4;
  info.has_new_shared_point1 = false;
  info.has_new_shared_point2 = false;

  EXPECT_TRUE(info.ShouldEmitNegativeChild());
  EXPECT_TRUE(info.ShouldEmitPositiveChild());
  EXPECT_TRUE(info.IsValid(/*vertex_count=*/4));
}

}  // walnut
