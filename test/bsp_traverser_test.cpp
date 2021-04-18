#include "walnut/plane_partitioner.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/bsp_tree.h"

namespace walnut {

using testing::IsEmpty;
using testing::UnorderedPointwise;
using testing::Eq;

TEST(BSPTraverser, EmptyInput) {
  BSPTree<> tree;

  using OutputPolygon = BSPTree<>::OutputPolygon;
  CollectorVisitor<OutputPolygon, PolygonFilter> visitor(PolygonFilter(0));

  tree.Traverse(visitor);

  EXPECT_THAT(visitor.polygons(), IsEmpty());
}

std::vector<ConvexPolygon<>> MakeCuboid(int min_x, int min_y, int min_z,
                                        int max_x, int max_y, int max_z) {
  AABB aabb(min_x, min_y, min_z, max_x, max_y, max_z, /*denom=*/1);
  return aabb.GetWalls();
}

TEST(PlanePartitioner, AcceptSingleCube) {
  BSPTree<> tree;
  BSPContentId id = tree.AllocateId();
  std::vector<ConvexPolygon<>> cube = MakeCuboid(/*min_x=*/0, /*min_y=*/0,
                                                 /*min_z=*/0, /*max_x=*/1,
                                                 /*max_y=*/1, /*max_z=*/1);
  tree.AddContents(id, cube);

  using OutputPolygon = BSPTree<>::OutputPolygon;
  CollectorVisitor<OutputPolygon, PolygonFilter> visitor((PolygonFilter(id)));

  tree.Traverse(visitor);

  EXPECT_THAT(visitor.polygons(), UnorderedPointwise(Eq(), cube));
}

TEST(PlanePartitioner, AcceptOneOfTwoCubes) {
  BSPTree<> tree;
  std::vector<ConvexPolygon<>> cube = MakeCuboid(/*min_x=*/0, /*min_y=*/0,
                                                 /*min_z=*/0, /*max_x=*/1,
                                                 /*max_y=*/1, /*max_z=*/1);
  BSPContentId id = tree.AllocateId();
  tree.AddContents(id, cube);
  // Filter out this cube.
  tree.AddContents(tree.AllocateId(), MakeCuboid(/*min_x=*/5, /*min_y=*/5,
                                                 /*min_z=*/5, /*max_x=*/6,
                                                 /*max_y=*/6, /*max_z=*/6));

  using OutputPolygon = BSPTree<>::OutputPolygon;
  CollectorVisitor<OutputPolygon, PolygonFilter> visitor((PolygonFilter(id)));

  tree.Traverse(visitor);

  EXPECT_THAT(visitor.polygons(), UnorderedPointwise(Eq(), cube));
}

TEST(PlanePartitioner, IntersectExactlyMatchingCubes) {
  // Two cubes are added with the exact same walls. The intersection should
  // return only 1.
  BSPTree<> tree;
  std::vector<ConvexPolygon<>> cube = MakeCuboid(/*min_x=*/0, /*min_y=*/0,
                                                 /*min_z=*/0, /*max_x=*/1,
                                                 /*max_y=*/1, /*max_z=*/1);
  BSPContentId id1 = tree.AllocateId();
  tree.AddContents(id1, cube);
  BSPContentId id2 = tree.AllocateId();
  tree.AddContents(id2, cube);

  using OutputPolygon = BSPTree<>::OutputPolygon;
  auto filter = MakeIntersectionFilter(PolygonFilter(id1), PolygonFilter(id2));
  CollectorVisitor<OutputPolygon, decltype(filter)> visitor(filter);

  tree.Traverse(visitor);

  EXPECT_THAT(visitor.polygons(), UnorderedPointwise(Eq(), cube));
}

TEST(PlanePartitioner, IntersectCubesWithCornerOverlap) {
  // Intersect two cubes that only overlap in their corners. The two cubes do
  // not share any walls.
  BSPTree<> tree;
  std::vector<ConvexPolygon<>> cube1 = MakeCuboid(/*min_x=*/-3, /*min_y=*/-3,
                                                  /*min_z=*/-3, /*max_x=*/1,
                                                  /*max_y=*/1, /*max_z=*/1);
  BSPContentId id1 = tree.AllocateId();
  tree.AddContents(id1, cube1);
  std::vector<ConvexPolygon<>> cube2 = MakeCuboid(/*min_x=*/0, /*min_y=*/0,
                                                  /*min_z=*/0, /*max_x=*/3,
                                                  /*max_y=*/3, /*max_z=*/3);
  BSPContentId id2 = tree.AllocateId();
  tree.AddContents(id2, cube2);

  using OutputPolygon = BSPTree<>::OutputPolygon;
  auto filter = MakeIntersectionFilter(PolygonFilter(id1), PolygonFilter(id2));
  CollectorVisitor<OutputPolygon, decltype(filter)> visitor(filter);

  tree.Traverse(visitor);

  std::vector<ConvexPolygon<>> expected = MakeCuboid(/*min_x=*/0, /*min_y=*/0,
                                                     /*min_z=*/0, /*max_x=*/1,
                                                     /*max_y=*/1, /*max_z=*/1);
  EXPECT_THAT(visitor.polygons(), UnorderedPointwise(Eq(), expected));
}

TEST(PlanePartitioner, IntersectCubesWithWallOverlap) {
  // Intersect two cubes that have some walls that overlap.
  BSPTree<> tree;
  std::vector<ConvexPolygon<>> cube1 = MakeCuboid(/*min_x=*/0, /*min_y=*/0,
                                                  /*min_z=*/0, /*max_x=*/2,
                                                  /*max_y=*/1, /*max_z=*/1);
  BSPContentId id1 = tree.AllocateId();
  tree.AddContents(id1, cube1);
  std::vector<ConvexPolygon<>> cube2 = MakeCuboid(/*min_x=*/1, /*min_y=*/0,
                                                  /*min_z=*/0, /*max_x=*/4,
                                                  /*max_y=*/1, /*max_z=*/1);
  BSPContentId id2 = tree.AllocateId();
  tree.AddContents(id2, cube2);

  using OutputPolygon = BSPTree<>::OutputPolygon;
  auto filter = MakeIntersectionFilter(PolygonFilter(id1), PolygonFilter(id2));
  CollectorVisitor<OutputPolygon, decltype(filter)> visitor(filter);

  tree.Traverse(visitor);

  std::vector<ConvexPolygon<>> expected = MakeCuboid(/*min_x=*/1, /*min_y=*/0,
                                                     /*min_z=*/0, /*max_x=*/2,
                                                     /*max_y=*/1, /*max_z=*/1);
  EXPECT_THAT(visitor.polygons(), UnorderedPointwise(Eq(), expected));
}

}  // walnut
