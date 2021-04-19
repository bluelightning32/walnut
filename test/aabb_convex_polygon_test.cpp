#include "walnut/aabb_convex_polygon.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/point3.h"

namespace walnut {

TEST(AABBConvexPolygon, ConstructEmpty) {
  AABBConvexPolygon<> polygon;

  EXPECT_TRUE(polygon.IsValidState());
  EXPECT_EQ(polygon.aabb(), AABB());
}

TEST(AABBConvexPolygon, SquareBounds) {
  // p[3] <----- p[2]
  //  |           ^
  //  v           |
  // p[0] -----> p[1]

  std::vector<Point3> p = {
    Point3(0, 0, 10),
    Point3(2, 0, 10),
    Point3(2, 1, 10),
    Point3(0, 1, 10),
  };
  HalfSpace3 plane(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10);
  AABBConvexPolygon<> polygon(plane, /*drop_dimension=*/2, p);
  EXPECT_TRUE(polygon.IsValidState());
  EXPECT_EQ(polygon.aabb(), AABB(/*min_x=*/0, /*min_y=*/0, /*min_z=*/10,
                                   /*max_x=*/2, /*max_y=*/1, /*max_z=*/10,
                                   /*denom=*/1));
}

TEST(AABBConvexPolygon, CopyConstruct) {
  // p[3] <----- p[2]
  //  |           ^
  //  v           |
  // p[0] -----> p[1]

  std::vector<Point3> p = {
    Point3(0, 0, 10),
    Point3(2, 0, 10),
    Point3(2, 1, 10),
    Point3(0, 1, 10),
  };
  HalfSpace3 plane(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10);
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

  std::vector<Point3> p = {
    Point3(0, 0, 10),
    Point3(2, 0, 10),
    Point3(2, 1, 10),
    Point3(0, 1, 10),
  };
  HalfSpace3 plane(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10);
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

  std::vector<Point3> p = {
    Point3(0, 0, 10),
    Point3(2, 0, 10),
    Point3(2, 1, 10),
    Point3(0, 1, 10),
  };
  HalfSpace3 plane(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10);
  AABBConvexPolygon<> polygon(plane, /*drop_dimension=*/2, p);

  HalfSpace3 split(/*x=*/-1, /*y=*/0, /*z=*/0, /*dist=*/1);
  ConvexPolygonSplitInfo split_info = polygon.GetSplitInfo(split);
  EXPECT_TRUE(split_info.IsValid(p.size()));
  EXPECT_TRUE(split_info.ShouldEmitNegativeChild());
  EXPECT_FALSE(split_info.ShouldEmitPositiveChild());
}

TEST(AABBConvexPolygon, SplitOnPositiveSide) {
  // split    p[3] <----- p[2]
  //   |       |           ^
  //   |->     v           |
  //   |      p[0] -----> p[1]

  std::vector<Point3> p = {
    Point3(0, 0, 10),
    Point3(2, 0, 10),
    Point3(2, 1, 10),
    Point3(0, 1, 10),
  };
  HalfSpace3 plane(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10);
  AABBConvexPolygon<> polygon(plane, /*drop_dimension=*/2, p);

  HalfSpace3 split(/*x=*/1, /*y=*/0, /*z=*/0, /*dist=*/-1);
  ConvexPolygonSplitInfo split_info = polygon.GetSplitInfo(split);
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

  std::vector<Point3> p = {
    Point3(0, 0, 10),
    Point3(2, 0, 10),
    Point3(2, 1, 10),
    Point3(0, 1, 10),
  };
  HalfSpace3 plane(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10);
  AABBConvexPolygon<> polygon(plane, /*drop_dimension=*/2, p);

  HalfSpace3 split(/*x=*/1, /*y=*/0, /*z=*/0, /*dist=*/1);
  ConvexPolygonSplitInfo split_info = polygon.GetSplitInfo(split);
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
            AABB(/*min_x=*/0, /*min_y=*/0, /*min_z=*/10,
                   /*max_x=*/1, /*max_y=*/1, /*max_z=*/10,
                   /*denom=*/1));
  EXPECT_EQ(children.second.aabb(),
            AABB(/*min_x=*/1, /*min_y=*/0, /*min_z=*/10,
                   /*max_x=*/2, /*max_y=*/1, /*max_z=*/10,
                   /*denom=*/1));
}

TEST(AABBConvexPolygon, MergeConvexVertex) {
  //                              |
  //      p[3]       p[2]         |
  //      +-----------+           |
  //      | polygon1   \          |
  //      |             \         |
  // p[0] |______________\ p[1]   |
  // q[3] |              / q[2]   |
  //      |             /         |
  //      | polygon2   /          |
  //      +-----------+           |
  //      q[0]       q[1]         |
  //                              |
  std::vector<Point3> p = {
    Point3(0, 2, 10),
    Point3(4, 2, 10),
    Point3(3, 4, 10),
    Point3(0, 4, 10),
  };
  HalfSpace3 plane(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10);
  AABBConvexPolygon<> polygon1a(plane, /*drop_dimension=*/2, p);
  AABBConvexPolygon<> polygon1b(plane, /*drop_dimension=*/2, p);
  EXPECT_EQ(polygon1a.vertex(1), p[0]);

  std::vector<Point3> q = {
    Point3(0, 0, 10),
    Point3(3, 0, 10),
    p[1],
    p[0],
  };
  AABBConvexPolygon<> polygon2a(plane, /*drop_dimension=*/2, q);
  AABBConvexPolygon<> polygon2b(plane, /*drop_dimension=*/2, q);
  EXPECT_EQ(polygon2a.vertex(1), q[0]);

  std::vector<Point3> merged_points = {
    q[0],
    q[1],
    q[2],
    p[2],
    p[3],
  };
  AABBConvexPolygon<> expected_merged(plane, /*drop_dimension=*/2, merged_points);

  EXPECT_TRUE(polygon1a.TryMergePolygon(/*nonzero_edge_dimension=*/0,
                                        /*my_edge_index=*/1,
                                        /*other=*/polygon2a,
                                        /*other_edge_index=*/3));
  EXPECT_EQ(polygon1a.aabb(), expected_merged.aabb());
  EXPECT_TRUE(polygon2b.TryMergePolygon(/*nonzero_edge_dimension=*/0,
                                        /*my_edge_index=*/3,
                                        /*other=*/polygon1b,
                                        /*other_edge_index=*/1));
  EXPECT_EQ(polygon2b.aabb(), expected_merged.aabb());
}

}  // walnut
