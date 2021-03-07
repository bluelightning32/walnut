#include "walnut/aabb_convex_polygon.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/convex_polygon_factory.h"
#include "walnut/homo_point3.h"

namespace walnut {

TEST(AABBConvexPolygon, ConstructEmpty) {
  AABBConvexPolygon<> polygon;

  EXPECT_TRUE(polygon.IsValidState());
  EXPECT_EQ(polygon.aabb(), AABB<>());
}

TEST(AABBConvexPolygon, SquareBounds) {
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
  AABBConvexPolygon<>::HalfSpace3Rep plane(/*x=*/0, /*y=*/0, /*z=*/1,
                                           /*dist=*/10);
  AABBConvexPolygon<> polygon(plane, /*drop_dimension=*/2, p);
  EXPECT_TRUE(polygon.IsValidState());
  EXPECT_EQ(polygon.aabb(), AABB<>(/*min_x=*/0, /*min_y=*/0, /*min_z=*/10,
                                   /*max_x=*/2, /*max_y=*/1, /*max_z=*/10,
                                   /*denom=*/1));
}

TEST(AABBConvexPolygon, CopyConstruct) {
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
  AABBConvexPolygon<>::HalfSpace3Rep plane(/*x=*/0, /*y=*/0, /*z=*/1,
                                           /*dist=*/10);
  AABBConvexPolygon<> polygon(plane, /*drop_dimension=*/2, p);

  AABBConvexPolygon<> polygon2(polygon);
  EXPECT_EQ(polygon2, polygon);
  EXPECT_EQ(polygon2.aabb(), polygon.aabb());
}

TEST(AABBConvexPolygon, MoveConstruct) {
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
  AABBConvexPolygon<>::HalfSpace3Rep plane(/*x=*/0, /*y=*/0, /*z=*/1,
                                           /*dist=*/10);
  AABBConvexPolygon<> polygon(plane, /*drop_dimension=*/2, p);

  AABBConvexPolygon<> polygon2(polygon);
  AABBConvexPolygon<> polygon3(std::move(polygon2));

  EXPECT_EQ(polygon3, polygon);
  EXPECT_EQ(polygon3.aabb(), polygon.aabb());
}

TEST(AABBConvexPolygon, SplitOnNegativeSide) {
  // split    p[3] <----- p[2]
  //   |       |           ^
  // <-|       v           |
  //   |      p[0] -----> p[1]

  std::vector<Point3<32>> p = {
    Point3<32>(0, 0, 10),
    Point3<32>(2, 0, 10),
    Point3<32>(2, 1, 10),
    Point3<32>(0, 1, 10),
  };
  AABBConvexPolygon<>::HalfSpace3Rep plane(/*x=*/0, /*y=*/0, /*z=*/1,
                                           /*dist=*/10);
  AABBConvexPolygon<> polygon(plane, /*drop_dimension=*/2, p);

  HalfSpace3<> split(/*x=*/-1, /*y=*/0, /*z=*/0, /*dist=*/1);
  AABBConvexPolygon<>::SplitInfoRep split_info = polygon.GetSplitInfo(split);
  EXPECT_TRUE(split_info.IsValid(p.size()));
  EXPECT_TRUE(split_info.ShouldEmitNegativeChild());
  EXPECT_FALSE(split_info.ShouldEmitPositiveChild());
}

TEST(AABBConvexPolygon, SplitOnPositiveSide) {
  // split    p[3] <----- p[2]
  //   |       |           ^
  //   |->     v           |
  //   |      p[0] -----> p[1]

  std::vector<Point3<32>> p = {
    Point3<32>(0, 0, 10),
    Point3<32>(2, 0, 10),
    Point3<32>(2, 1, 10),
    Point3<32>(0, 1, 10),
  };
  AABBConvexPolygon<>::HalfSpace3Rep plane(/*x=*/0, /*y=*/0, /*z=*/1,
                                           /*dist=*/10);
  AABBConvexPolygon<> polygon(plane, /*drop_dimension=*/2, p);

  HalfSpace3<> split(/*x=*/1, /*y=*/0, /*z=*/0, /*dist=*/-1);
  AABBConvexPolygon<>::SplitInfoRep split_info = polygon.GetSplitInfo(split);
  EXPECT_TRUE(split_info.IsValid(p.size()));
  EXPECT_FALSE(split_info.ShouldEmitNegativeChild());
  EXPECT_TRUE(split_info.ShouldEmitPositiveChild());
}

TEST(AABBConvexPolygon, SplitInMiddle) {
  //       split
  //         |->
  //         |
  // p[3] <----- p[2]
  //  |      |    ^
  //  v      |    |
  // p[0] -----> p[1]
  //         |

  std::vector<Point3<32>> p = {
    Point3<32>(0, 0, 10),
    Point3<32>(2, 0, 10),
    Point3<32>(2, 1, 10),
    Point3<32>(0, 1, 10),
  };
  AABBConvexPolygon<>::HalfSpace3Rep plane(/*x=*/0, /*y=*/0, /*z=*/1,
                                           /*dist=*/10);
  AABBConvexPolygon<> polygon(plane, /*drop_dimension=*/2, p);

  HalfSpace3<> split(/*x=*/1, /*y=*/0, /*z=*/0, /*dist=*/1);
  AABBConvexPolygon<>::SplitInfoRep split_info = polygon.GetSplitInfo(split);
  EXPECT_TRUE(split_info.IsValid(p.size()));
  EXPECT_TRUE(split_info.ShouldEmitNegativeChild());
  EXPECT_TRUE(split_info.ShouldEmitPositiveChild());

  std::pair<AABBConvexPolygon<>, AABBConvexPolygon<>> children =
    polygon.CreateSplitChildren(split_info);

  EXPECT_EQ(children.first.vertex_count(), 4);
  EXPECT_EQ(children.second.vertex_count(), 4);
  EXPECT_TRUE(children.first.IsValidState());
  EXPECT_TRUE(children.second.IsValidState());

  EXPECT_EQ(children.first.aabb(),
            AABB<>(/*min_x=*/0, /*min_y=*/0, /*min_z=*/10,
                   /*max_x=*/1, /*max_y=*/1, /*max_z=*/10,
                   /*denom=*/1));
  EXPECT_EQ(children.second.aabb(),
            AABB<>(/*min_x=*/1, /*min_y=*/0, /*min_z=*/10,
                   /*max_x=*/2, /*max_y=*/1, /*max_z=*/10,
                   /*denom=*/1));
}

}  // walnut
