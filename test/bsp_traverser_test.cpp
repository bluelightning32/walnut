#include "walnut/plane_partitioner.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/bsp_tree.h"
#include "walnut/connecting_visitor.h"

namespace walnut {

using testing::AnyOf;
using testing::Eq;
using testing::IsEmpty;
using testing::UnorderedPointwise;
using testing::SizeIs;

std::vector<MutableConvexPolygon<>> MakeCuboid(int min_x, int min_y, int min_z,
                                               int max_x, int max_y,
                                               int max_z) {
  AABB aabb(min_x, min_y, min_z, max_x, max_y, max_z, /*denom=*/1);
  return aabb.GetWalls();
}

// Return a cubiod where every facet is split into 2 triangle, instead of a
// square facets.
std::vector<MutableConvexPolygon<>> MakeTriangulatedCuboid(int min_x,
                                                           int min_y,
                                                           int min_z,
                                                           int max_x,
                                                           int max_y,
                                                           int max_z) {
  std::vector<MutableConvexPolygon<>> triangles;
  for (const MutableConvexPolygon<>& square : MakeCuboid(min_x, min_y, min_z,
                                                         max_x, max_y,
                                                         max_z)) {
    assert(square.vertex_count() == 4);
    ConvexPolygonSplitInfo split_info;
    // All vertices except index 1 are in the positive child.
    split_info.ranges.neg_range.first = 1;
    split_info.ranges.neg_range.second = 2;
    // All vertices except index 3 are in the negative child.
    split_info.ranges.pos_range.first = 3;
    split_info.ranges.pos_range.second = 4;
    split_info.new_line = PluckerLine(square.vertex(0), square.vertex(2));
    assert(split_info.ShouldEmitNegativeChild());
    assert(split_info.ShouldEmitPositiveChild());

    std::pair<MutableConvexPolygon<>, MutableConvexPolygon<>> children =
      square.CreateSplitChildren(split_info);
    triangles.push_back(std::move(children.first));
    triangles.push_back(std::move(children.second));
  }
  return triangles;
}

template <typename Polygon>
std::map<HalfSpace3, std::vector<Polygon>, HalfSpace3ReduceCompare>
GroupByPlane(const std::vector<Polygon>& polygons) {
  std::map<HalfSpace3, std::vector<Polygon>, HalfSpace3ReduceCompare> result;
  for (const Polygon& polygon : polygons) {
    result[polygon.plane()].push_back(polygon);
  }
  return result;
}


TEST(BSPTraverser, EmptyInput) {
  BSPTree<> tree;

  using OutputPolygon = BSPTree<>::OutputPolygon;
  CollectorVisitor<OutputPolygon, PolygonFilter> visitor(PolygonFilter(0));

  tree.Traverse(visitor);

  EXPECT_THAT(visitor.polygons(), IsEmpty());
}

TEST(BSPTraverser, AcceptSingleCube) {
  BSPTree<> tree;
  BSPContentId id = tree.AllocateId();
  std::vector<MutableConvexPolygon<>> cube = MakeCuboid(/*min_x=*/0,
                                                        /*min_y=*/0,
                                                        /*min_z=*/0,
                                                        /*max_x=*/1,
                                                        /*max_y=*/1,
                                                        /*max_z=*/1);
  tree.AddContents(id, cube);

  using OutputPolygon = BSPTree<>::OutputPolygon;
  CollectorVisitor<OutputPolygon, PolygonFilter> visitor((PolygonFilter(id)));

  tree.Traverse(visitor);

  EXPECT_THAT(visitor.polygons(), UnorderedPointwise(Eq(), cube));
}

TEST(BSPTraverser, AcceptSingleTriangulatedCube) {
  BSPTree<> tree;
  BSPContentId id = tree.AllocateId();
  std::vector<MutableConvexPolygon<>> triangulated_cube =
    MakeTriangulatedCuboid(/*min_x=*/0, /*min_y=*/0, /*min_z=*/0, /*max_x=*/1,
                           /*max_y=*/1, /*max_z=*/1);
  EXPECT_THAT(triangulated_cube, SizeIs(12));
  tree.AddContents(id, triangulated_cube);

  using OutputPolygon = BSPTree<>::OutputPolygon;
  CollectorVisitor<OutputPolygon, PolygonFilter> visitor((PolygonFilter(id)));

  tree.Traverse(visitor);

  EXPECT_THAT(visitor.polygons(), UnorderedPointwise(Eq(), triangulated_cube));
}

TEST(BSPTraverser, AcceptOneOfTwoCubes) {
  BSPTree<> tree;
  std::vector<MutableConvexPolygon<>> cube = MakeCuboid(/*min_x=*/0,
                                                        /*min_y=*/0,
                                                        /*min_z=*/0,
                                                        /*max_x=*/1,
                                                        /*max_y=*/1,
                                                        /*max_z=*/1);
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

TEST(BSPTraverser, IntersectExactlyMatchingCubes) {
  // Two cubes are added with the exact same walls. The intersection should
  // return only 1.
  BSPTree<> tree;
  std::vector<MutableConvexPolygon<>> cube = MakeCuboid(/*min_x=*/0,
                                                        /*min_y=*/0,
                                                        /*min_z=*/0,
                                                        /*max_x=*/1,
                                                        /*max_y=*/1,
                                                        /*max_z=*/1);
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

TEST(BSPTraverser, UnionExactlyMatchingCubes) {
  // Two cubes are added with the exact same walls. The intersection should
  // return only 1.
  BSPTree<> tree;
  std::vector<MutableConvexPolygon<>> cube = MakeCuboid(/*min_x=*/0,
                                                        /*min_y=*/0,
                                                        /*min_z=*/0,
                                                        /*max_x=*/1,
                                                        /*max_y=*/1,
                                                        /*max_z=*/1);
  BSPContentId id1 = tree.AllocateId();
  tree.AddContents(id1, cube);
  BSPContentId id2 = tree.AllocateId();
  tree.AddContents(id2, cube);

  using OutputPolygon = BSPTree<>::OutputPolygon;
  auto filter = MakeUnionFilter(PolygonFilter(id1), PolygonFilter(id2));
  CollectorVisitor<OutputPolygon, decltype(filter)> visitor(filter);

  tree.Traverse(visitor);

  EXPECT_THAT(visitor.polygons(), UnorderedPointwise(Eq(), cube));
}

TEST(BSPTraverser, IntersectCubesWithCornerOverlap) {
  // Intersect two cubes that only overlap in their corners. The two cubes do
  // not share any walls.
  BSPTree<> tree;
  std::vector<MutableConvexPolygon<>> cube1 = MakeCuboid(/*min_x=*/-3,
                                                         /*min_y=*/-3,
                                                         /*min_z=*/-3,
                                                         /*max_x=*/1,
                                                         /*max_y=*/1,
                                                         /*max_z=*/1);
  BSPContentId id1 = tree.AllocateId();
  tree.AddContents(id1, cube1);
  std::vector<MutableConvexPolygon<>> cube2 = MakeCuboid(/*min_x=*/0,
                                                         /*min_y=*/0,
                                                         /*min_z=*/0,
                                                         /*max_x=*/3,
                                                         /*max_y=*/3,
                                                         /*max_z=*/3);
  BSPContentId id2 = tree.AllocateId();
  tree.AddContents(id2, cube2);

  using OutputPolygon = BSPTree<>::OutputPolygon;
  auto filter = MakeIntersectionFilter(PolygonFilter(id1), PolygonFilter(id2));
  CollectorVisitor<OutputPolygon, decltype(filter)> visitor(filter);

  tree.Traverse(visitor);

  std::vector<MutableConvexPolygon<>> expected = MakeCuboid(/*min_x=*/0,
                                                            /*min_y=*/0,
                                                            /*min_z=*/0,
                                                            /*max_x=*/1,
                                                            /*max_y=*/1,
                                                            /*max_z=*/1);
  EXPECT_THAT(visitor.polygons(), UnorderedPointwise(Eq(), expected));
}

TEST(BSPTraverser, IntersectCubesWithWallOverlap) {
  // Intersect two cubes that have some walls that overlap.
  BSPTree<> tree;
  std::vector<MutableConvexPolygon<>> cube1 = MakeCuboid(/*min_x=*/0,
                                                         /*min_y=*/0,
                                                         /*min_z=*/0,
                                                         /*max_x=*/2,
                                                         /*max_y=*/1,
                                                         /*max_z=*/1);
  BSPContentId id1 = tree.AllocateId();
  tree.AddContents(id1, cube1);
  std::vector<MutableConvexPolygon<>> cube2 = MakeCuboid(/*min_x=*/1,
                                                         /*min_y=*/0,
                                                         /*min_z=*/0,
                                                         /*max_x=*/4,
                                                         /*max_y=*/1,
                                                         /*max_z=*/1);
  BSPContentId id2 = tree.AllocateId();
  tree.AddContents(id2, cube2);

  using OutputPolygon = BSPTree<>::OutputPolygon;
  auto filter = MakeIntersectionFilter(PolygonFilter(id1), PolygonFilter(id2));
  CollectorVisitor<OutputPolygon, decltype(filter)> visitor(filter);

  tree.Traverse(visitor);

  std::vector<MutableConvexPolygon<>> expected = MakeCuboid(/*min_x=*/1,
                                                            /*min_y=*/0,
                                                            /*min_z=*/0,
                                                            /*max_x=*/2,
                                                            /*max_y=*/1,
                                                            /*max_z=*/1);
  EXPECT_THAT(visitor.polygons(), UnorderedPointwise(Eq(), expected));
}

TEST(BSPTraverser, ConnectIntersectCubesWithCornerOverlap) {
  // Intersect two cubes that only overlap in their corners. The two cubes do
  // not share any walls.
  BSPTree<> tree;
  std::vector<MutableConvexPolygon<>> cube1 = MakeCuboid(/*min_x=*/-3,
                                                         /*min_y=*/-3,
                                                         /*min_z=*/-3,
                                                         /*max_x=*/1,
                                                         /*max_y=*/1,
                                                         /*max_z=*/1);
  BSPContentId id1 = tree.AllocateId();
  tree.AddContents(id1, cube1);
  std::vector<MutableConvexPolygon<>> cube2 = MakeCuboid(/*min_x=*/0,
                                                         /*min_y=*/0,
                                                         /*min_z=*/0,
                                                         /*max_x=*/3,
                                                         /*max_y=*/3,
                                                         /*max_z=*/3);
  BSPContentId id2 = tree.AllocateId();
  tree.AddContents(id2, cube2);

  auto filter = MakeIntersectionFilter(PolygonFilter(id1), PolygonFilter(id2));
  bool errored = false;
  auto error_log = [&errored](const std::string& error) {
    errored = true;
    std::cout << error << std::endl;
  };
  ConnectingVisitor<decltype(filter)> visitor(filter, error_log);
  tree.Traverse(visitor);

  EXPECT_FALSE(errored);
  std::vector<MutableConvexPolygon<>> expected = MakeCuboid(/*min_x=*/0,
                                                            /*min_y=*/0,
                                                            /*min_z=*/0,
                                                            /*max_x=*/1,
                                                            /*max_y=*/1,
                                                            /*max_z=*/1);
  EXPECT_THAT(visitor.polygons(), UnorderedPointwise(Eq(), expected));
}

TEST(BSPTraverser, ConnectUnionCubesWithCornerOverlap) {
  // Union two cubes that only overlap in their corners. The two cubes do not
  // share any walls.
  BSPTree<> tree;
  std::vector<MutableConvexPolygon<>> cube1 = MakeCuboid(/*min_x=*/-3,
                                                         /*min_y=*/-3,
                                                         /*min_z=*/-3,
                                                         /*max_x=*/1,
                                                         /*max_y=*/1,
                                                         /*max_z=*/1);
  BSPContentId id1 = tree.AllocateId();
  tree.AddContents(id1, cube1);
  std::vector<MutableConvexPolygon<>> cube2 = MakeCuboid(/*min_x=*/0,
                                                         /*min_y=*/0,
                                                         /*min_z=*/0,
                                                         /*max_x=*/3,
                                                         /*max_y=*/3,
                                                         /*max_z=*/3);
  BSPContentId id2 = tree.AllocateId();
  tree.AddContents(id2, cube2);

  auto filter = MakeUnionFilter(PolygonFilter(id1), PolygonFilter(id2));
  bool errored = false;
  auto error_log = [&errored](const std::string& error) {
    errored = true;
    std::cout << error << std::endl;
  };
  using Visitor = ConnectingVisitor<decltype(filter)>;
  Visitor visitor(filter, error_log);
  tree.Traverse(visitor);

  EXPECT_FALSE(errored);

  visitor.FilterEmptyPolygons();
  std::map<HalfSpace3,
           std::vector<Visitor::PolygonRep>,
           HalfSpace3ReduceCompare> planes_to_polygons =
             GroupByPlane(visitor.polygons());
  for (const std::pair<const HalfSpace3,
                       std::vector<Visitor::PolygonRep>>& plane_info
                       : planes_to_polygons) {
    int non_zero_dims = 0;
    BigInt non_zero;
    if (!plane_info.first.x().IsZero()) {
      ++non_zero_dims;
      non_zero = plane_info.first.x();
    }
    if (!plane_info.first.y().IsZero()) {
      ++non_zero_dims;
      non_zero = plane_info.first.y();
    }
    if (!plane_info.first.z().IsZero()) {
      ++non_zero_dims;
      non_zero = plane_info.first.z();
    }
    EXPECT_EQ(non_zero_dims, 1);
    EXPECT_EQ(non_zero.abs(), 1);
    if (non_zero * plane_info.first.d() == -3) {
      EXPECT_THAT(plane_info.second, SizeIs(1));
    } else if (non_zero * plane_info.first.d() == 1) {
      EXPECT_THAT(plane_info.second, SizeIs(2));
    } else if (non_zero * plane_info.first.d() == 3) {
      EXPECT_THAT(plane_info.second, SizeIs(1));
    } else if (non_zero * plane_info.first.d() == 0) {
      EXPECT_THAT(plane_info.second, SizeIs(2));
    } else {
      EXPECT_THAT(plane_info.second, IsEmpty());
    }
  }
}

TEST(BSPTraverser, ConnectUnionCubesIntoCubiod) {
  // Union two cubes that together form a rectangular prism.
  BSPTree<> tree;
  std::vector<MutableConvexPolygon<>> cube1 = MakeCuboid(/*min_x=*/0,
                                                         /*min_y=*/0,
                                                         /*min_z=*/0,
                                                         /*max_x=*/3,
                                                         /*max_y=*/3,
                                                         /*max_z=*/3);
  BSPContentId id1 = tree.AllocateId();
  tree.AddContents(id1, cube1);
  std::vector<MutableConvexPolygon<>> cube2 = MakeCuboid(/*min_x=*/1,
                                                         /*min_y=*/0,
                                                         /*min_z=*/0,
                                                         /*max_x=*/4,
                                                         /*max_y=*/3,
                                                         /*max_z=*/3);
  BSPContentId id2 = tree.AllocateId();
  tree.AddContents(id2, cube2);

  auto filter = MakeUnionFilter(PolygonFilter(id1), PolygonFilter(id2));
  bool errored = false;
  auto error_log = [&errored](const std::string& error) {
    errored = true;
    std::cout << error << std::endl;
  };
  using Visitor = ConnectingVisitor<decltype(filter)>;
  Visitor visitor(filter, error_log);
  tree.Traverse(visitor);

  EXPECT_FALSE(errored);

  std::vector<MutableConvexPolygon<>> expected = MakeCuboid(/*min_x=*/0,
                                                            /*min_y=*/0,
                                                            /*min_z=*/0,
                                                            /*max_x=*/4,
                                                            /*max_y=*/3,
                                                            /*max_z=*/3);
  visitor.FilterEmptyPolygons();
  EXPECT_THAT(visitor.polygons(), UnorderedPointwise(Eq(), expected));
}

TEST(BSPTraverser, ConnectUnionCubesWithPlaneCornerOverlap) {
  // Union two cubes whose corners overlap, and both cubes have the same start
  // and height in the z component.
  BSPTree<> tree;
  std::vector<MutableConvexPolygon<>> cube1 = MakeCuboid(/*min_x=*/0,
                                                         /*min_y=*/0,
                                                         /*min_z=*/0,
                                                         /*max_x=*/2,
                                                         /*max_y=*/2,
                                                         /*max_z=*/1);
  BSPContentId id1 = tree.AllocateId();
  tree.AddContents(id1, cube1);
  std::vector<MutableConvexPolygon<>> cube2 = MakeCuboid(/*min_x=*/1,
                                                         /*min_y=*/1,
                                                         /*min_z=*/0,
                                                         /*max_x=*/3,
                                                         /*max_y=*/3,
                                                         /*max_z=*/1);
  BSPContentId id2 = tree.AllocateId();
  tree.AddContents(id2, cube2);

  auto filter = MakeUnionFilter(PolygonFilter(id1), PolygonFilter(id2));
  bool errored = false;
  auto error_log = [&errored](const std::string& error) {
    errored = true;
    std::cout << error << std::endl;
  };
  using Visitor = ConnectingVisitor<decltype(filter)>;
  Visitor visitor(filter, error_log);
  tree.Traverse(visitor);

  EXPECT_FALSE(errored);

  visitor.FilterEmptyPolygons();
  EXPECT_THAT(visitor.polygons(), SizeIs(14));
  std::map<HalfSpace3,
           std::vector<Visitor::PolygonRep>,
           HalfSpace3ReduceCompare> planes_to_polygons =
             GroupByPlane(visitor.polygons());
  for (const std::pair<const HalfSpace3,
                       std::vector<Visitor::PolygonRep>>& plane_info
                       : planes_to_polygons) {
    int non_zero_dims = 0;
    int non_zero_dim = -1;
    BigInt non_zero;
    if (!plane_info.first.x().IsZero()) {
      ++non_zero_dims;
      non_zero = plane_info.first.x();
      non_zero_dim = 0;
    }
    if (!plane_info.first.y().IsZero()) {
      ++non_zero_dims;
      non_zero = plane_info.first.y();
      non_zero_dim = 1;
    }
    if (!plane_info.first.z().IsZero()) {
      ++non_zero_dims;
      non_zero = plane_info.first.z();
      non_zero_dim = 2;
    }
    EXPECT_EQ(non_zero_dims, 1);
    EXPECT_EQ(non_zero.abs(), 1);
    if (non_zero_dim == 2) {
      EXPECT_THAT(non_zero * plane_info.first.d(), AnyOf(Eq(0), Eq(1)));
      EXPECT_THAT(plane_info.second, SizeIs(3));
    } else {
      EXPECT_THAT(plane_info.second, SizeIs(1));
    }
  }
}

}  // walnut
