#include "walnut/connected_polygon.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/aabb_convex_polygon.h"
#include "walnut/point3.h"

namespace walnut {

TEST(ConnectedPolygon, ConstructEmpty) {
  ConnectedPolygon<> polygon;

  EXPECT_TRUE(polygon.IsValidState());
}

ConvexPolygon<> MakeRectangle() {
  // p[3] <----- p[2]
  //  |           ^
  //  v           |
  // p[0] -----> p[1]

  std::vector<Point3<32>> p = {
    Point3<32>(0, 0, 10),
    Point3<32>(2, 0, 10),
    Point3<32>(2, 1, 10),
    Point3<32>(0, 1, 10),
  };
  ConvexPolygon<>::HalfSpace3Rep plane(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10);
  return ConvexPolygon<>(plane, /*drop_dimension=*/2, p);
}

TEST(ConnectedPolygon, ConstructOnTopAABB) {
  ConnectedPolygon<AABBConvexPolygon<>> polygon(MakeRectangle());
  EXPECT_TRUE(polygon.IsValidState());
  EXPECT_EQ(polygon.aabb(), AABBConvexPolygon<>(MakeRectangle()).aabb());
}

TEST(ConnectedPolygon, ConstructUnderneathAABB) {
  AABBConvexPolygon<ConnectedPolygon<>> polygon(MakeRectangle());
  EXPECT_TRUE(polygon.IsValidState());
  EXPECT_EQ(polygon.aabb(), AABBConvexPolygon<>(MakeRectangle()).aabb());
}

TEST(ConnectedPolygon, CopyConstruct) {
  ConnectedPolygon<> polygon(MakeRectangle());

  ConnectedPolygon<> polygon2(polygon);
  EXPECT_EQ(polygon2, polygon);
}

TEST(ConnectedPolygon, SplitInMiddle) {
  //       split
  //         |->
  //         |
  // p[3] <----- p[2]
  //  |      |    ^
  //  v      |    |
  // p[0] -----> p[1]
  //         |

  ConnectedPolygon<> polygon(MakeRectangle());

  HalfSpace3<> split(/*x=*/1, /*y=*/0, /*z=*/0, /*dist=*/1);
  ConnectedPolygon<>::SplitInfoRep split_info = polygon.GetSplitInfo(split);
  EXPECT_TRUE(split_info.IsValid(polygon.vertex_count()));
  EXPECT_TRUE(split_info.ShouldEmitNegativeChild());
  EXPECT_TRUE(split_info.ShouldEmitPositiveChild());

  std::pair<ConnectedPolygon<>, ConnectedPolygon<>> children =
    polygon.CreateSplitChildren(split_info);

  EXPECT_EQ(children.first.vertex_count(), 4);
  EXPECT_EQ(children.second.vertex_count(), 4);
  EXPECT_TRUE(children.first.IsValidState());
  EXPECT_TRUE(children.second.IsValidState());
}

TEST(ConnectedPolygon, edge_index) {
  AABBConvexPolygon<ConnectedPolygon<>> polygon(MakeRectangle());

  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    EXPECT_EQ(&polygon.edge(i).polygon(), &polygon);
    EXPECT_EQ(polygon.edge(i).edge_index(), i);
  }
}

}  // walnut
