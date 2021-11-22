#include "walnut/polygon_event_point.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::AnyOf;
using testing::Eq;
using testing::IsEmpty;
using testing::SizeIs;

BSPPolygon<AABBConvexPolygon<>> MakeTriangleForInterval(
    BSPContentId id, const HomoPoint3& start, const HomoPoint3& end) {
  std::vector<HomoPoint3> vertices;
  vertices.push_back(start);
  // Find 2 components that are different in `start` and `end`, and copy one
  // from `start` and one from `end`. The remaining component is copied from
  // either.
  BigInt middle_coords[3];
  size_t component = 0;
  for (; component < 3; ++component) {
    middle_coords[component] =
      start.vector_from_origin().components()[component] * end.w();
    if (!start.IsEquivalentComponent(component, end)) break;
  }
  for (++component; component < 3; ++component) {
    middle_coords[component] =
      end.vector_from_origin().components()[component] * start.w();
  }
  vertices.emplace_back(middle_coords[0], middle_coords[1], middle_coords[2],
                        start.w() * end.w());
  vertices.push_back(end);
  HalfSpace3 plane(vertices[0], vertices[1], vertices[2]);
  int drop_dimension = plane.normal().GetFirstNonzeroDimension();
  return BSPPolygon<AABBConvexPolygon<>>(
      id, /*on_node_plane=*/nullptr, /*pos_side=*/false,
      AABBConvexPolygon<>(std::move(plane), drop_dimension, std::move(vertices)));
}

TEST(MakeEventPoints, ConstructEmpty) {
  MakeEventPoints(/*dimension=*/0,
                  /*polygons=*/std::vector<BSPPolygon<AABBConvexPolygon<>>>(),
                  /*event_points=*/nullptr);
}

void CheckSorted(size_t dimension,
                 const std::vector<BSPPolygon<AABBConvexPolygon<>>>& polygons,
                 const PolygonEventPoint* event_points) {
  std::map<const BSPPolygon<AABBConvexPolygon<>>*, size_t> seen;
  const HomoPoint3* prev = nullptr;
  bool seen_start_points = false;
  for (size_t i = 0; i < polygons.size()*2; ++i) {
    size_t& seen_polygon =
      seen[&event_points[i].GetPolygon(event_points, polygons)];
    EXPECT_EQ(event_points[i].start, seen_polygon == 0);
    ++seen_polygon;
    EXPECT_LE(seen_polygon, 2);
    const HomoPoint3& location =
      event_points[i].GetLocation(dimension, event_points, polygons);
    if (i > 0) {
      EXPECT_NE(event_points[i].new_location,
                prev->IsEquivalentComponent(dimension, location));
      EXPECT_LE(prev->CompareComponent(dimension, location), 0);
    } else {
      EXPECT_TRUE(event_points[0].new_location);
    }
    if (event_points[i].new_location) seen_start_points = false;
    if (event_points[i].start) {
      seen_start_points = true;
    } else {
      EXPECT_FALSE(seen_start_points);
    }
    prev = &location;
  }
  for (size_t i = 0; i < polygons.size(); ++i) {
    EXPECT_EQ(seen[&polygons[i]], 2);
  }
}

TEST(MakeEventPoints, ThreeOverlaps) {
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  polygons.push_back(MakeTriangleForInterval(0, Point3(0, 0, 0),
                                             Point3(5, 5, 5)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(1, 1, 1),
                                             Point3(4, 4, 4)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(2, 2, 2),
                                             Point3(3, 3, 3)));

  for (size_t dimension = 0; dimension < 3; ++dimension) {
    PolygonEventPoint event_points[6];
    MakeEventPoints(dimension, polygons, event_points);
    CheckSorted(dimension, polygons, event_points);
  }
}

TEST(MakeEventPoints, EndBeforeStart) {
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  polygons.push_back(MakeTriangleForInterval(0, Point3(0, 0, 0),
                                             Point3(1, 1, 1)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(1, 1, 1),
                                             Point3(2, 2, 2)));

  for (size_t dimension = 0; dimension < 3; ++dimension) {
    PolygonEventPoint event_points[6];
    MakeEventPoints(dimension, polygons, event_points);
    CheckSorted(dimension, polygons, event_points);
  }
}

// Tests two sets of intervals, where the two sets do not touch in the middle.
TEST(MakeEventPoints, Discontinuity) {
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  polygons.push_back(MakeTriangleForInterval(0, Point3(0, 0, 0),
                                             Point3(2, 2, 2)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(1, 1, 1),
                                             Point3(2, 2, 2)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(3, 3, 3),
                                             Point3(5, 5, 5)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(4, 4, 4),
                                             Point3(5, 5, 5)));

  for (size_t dimension = 0; dimension < 3; ++dimension) {
    PolygonEventPoint event_points[8];
    MakeEventPoints(dimension, polygons, event_points);
    CheckSorted(dimension, polygons, event_points);
  }
}

// The polygons are sorted in different orders in different dimensions.
TEST(MakeEventPoints, DifferentDimensionSort) {
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  polygons.push_back(MakeTriangleForInterval(0, Point3(0, 0, 0),
                                             Point3(1, 1, 1)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(1, 2, 2),
                                             Point3(2, 3, 3)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(2, 1, 1),
                                             Point3(3, 2, 2)));

  for (size_t dimension = 0; dimension < 3; ++dimension) {
    PolygonEventPoint event_points[8];
    MakeEventPoints(dimension, polygons, event_points);
    CheckSorted(dimension, polygons, event_points);
  }
}

TEST(GetLowestCost, SplitMiddleNoExclude) {
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  for (size_t i = 0; i < 10; ++i) {
    polygons.push_back(MakeTriangleForInterval(0, Point3(i, i, i),
                                               Point3(i + 1, i + 1, i + 1)));
  }
  PolygonEventPoint event_points[20];
  MakeEventPoints(/*dimension=*/0, polygons, event_points);
  CheckSorted(/*dimension=*/0, polygons, event_points);

  PolygonEventPointPartition best = GetLowestCost(/*exclude_id=*/-1,
                                                  /*exclude_count=*/0,
                                                  polygons,
                                                  event_points);
  ASSERT_LT(best.split_index, polygons.size() * 2);
  EXPECT_EQ(best.neg_poly_count, 5);
  EXPECT_EQ(best.pos_poly_count, 5);
  EXPECT_EQ(event_points[best.split_index].GetLocation(/*dimension=*/0,
                                                       event_points,
                                                       polygons),
            Point3(5, 5, 5));

  EXPECT_EQ(best.cost, GetSplitCost(/*neg_total=*/5, /*neg_exclude=*/0,
                                    /*pos_total=*/5, /*pos_exclude=*/0));
}

TEST(GetLowestCost, AvoidOverlapNoExclude) {
  /*  0 1 2 3 4 5 6
   *  |-|-| |-|-|-|
   *      |-----|
   *
   *      ^
   *      |
   */

  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  for (size_t i = 0; i < 6; ++i) {
    if (i == 2) continue;
    polygons.push_back(MakeTriangleForInterval(0, Point3(i, i, i),
                                               Point3(i + 1, i + 1, i + 1)));
  }
  polygons.push_back(MakeTriangleForInterval(0, Point3(2, 2, 2),
                                             Point3(5, 5, 5)));
  PolygonEventPoint event_points[12];
  MakeEventPoints(/*dimension=*/0, polygons, event_points);
  CheckSorted(/*dimension=*/0, polygons, event_points);

  PolygonEventPointPartition best = GetLowestCost(/*exclude_id=*/-1,
                                                  /*exclude_count=*/0,
                                                  polygons,
                                                  event_points);
  ASSERT_LT(best.split_index, polygons.size() * 2);
  EXPECT_EQ(event_points[best.split_index].GetLocation(/*dimension=*/0,
                                                       event_points,
                                                       polygons),
            Point3(2, 2, 2));
  EXPECT_EQ(best.neg_poly_count, 2);
  EXPECT_EQ(best.pos_poly_count, 4);

  EXPECT_EQ(best.cost, GetSplitCost(/*neg_total=*/2, /*neg_exclude=*/0,
                                    /*pos_total=*/4, /*pos_exclude=*/0));
}

TEST(GetLowestCost, OverlapCompromiseNoExclude) {
  /*  0 1 2 3 4 5 6 7 8 9
   *  |---|---|---|---|
   *    |---|---|---|---|
   *
   *          ^ ^
   *          | |
   *    either is best
   */

  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  for (size_t i = 0; i < 8; ++i) {
    polygons.push_back(MakeTriangleForInterval(0, Point3(i, i, i),
                                               Point3(i + 2, i + 2, i + 2)));
  }
  PolygonEventPoint event_points[16];
  MakeEventPoints(/*dimension=*/0, polygons, event_points);
  CheckSorted(/*dimension=*/0, polygons, event_points);

  PolygonEventPointPartition best = GetLowestCost(/*exclude_id=*/-1,
                                                  /*exclude_count=*/0,
                                                  polygons,
                                                  event_points);
  ASSERT_LT(best.split_index, polygons.size() * 2);
  EXPECT_THAT(event_points[best.split_index].GetLocation(/*dimension=*/0,
                                                         event_points,
                                                         polygons),
              AnyOf(Eq(Point3(4, 4, 4)), Eq(Point3(5, 5, 5))));

  EXPECT_EQ(best.neg_poly_count + best.pos_poly_count, 9);
  EXPECT_THAT(best.neg_poly_count, AnyOf(Eq(4), Eq(5)));

  EXPECT_EQ(best.cost, GetSplitCost(/*neg_total=*/4, /*neg_exclude=*/0,
                                    /*pos_total=*/5, /*pos_exclude=*/0));
}

TEST(GetLowestCost, PartitionsExcludeId) {
  /*  0 1 2 3 4 5 6 7 8 9 0 1 2
   *  |-|-|-|-|-|-|-|-|
   *  mesh0
   *
   *                  |-|-|-|-|
   *                  mesh1
   *
   *                  ^
   *                  |
   *                best
   */

  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  for (size_t i = 0; i < 8; ++i) {
    polygons.push_back(MakeTriangleForInterval(0, Point3(i, i, i),
                                               Point3(i + 1, i + 1, i + 1)));
  }
  for (size_t i = 8; i < 12; ++i) {
    polygons.push_back(MakeTriangleForInterval(1, Point3(i, i, i),
                                               Point3(i + 1, i + 1, i + 1)));
  }
  std::vector<PolygonEventPoint> event_points(polygons.size()*2);
  MakeEventPoints(/*dimension=*/0, polygons, event_points.data());
  CheckSorted(/*dimension=*/0, polygons, event_points.data());

  PolygonEventPointPartition best = GetLowestCost(/*exclude_id=*/0,
                                                  /*exclude_count=*/8,
                                                  polygons,
                                                  event_points.data());
  ASSERT_LT(best.split_index, polygons.size() * 2);
  EXPECT_EQ(event_points[best.split_index].GetLocation(/*dimension=*/0,
                                                       event_points.data(),
                                                       polygons),
            Point3(8, 8, 8));

  EXPECT_EQ(best.neg_poly_count, 8);
  EXPECT_EQ(best.pos_poly_count, 4);

  EXPECT_EQ(best.cost, GetSplitCost(/*neg_total=*/8, /*neg_exclude=*/8,
                                    /*pos_total=*/4, /*pos_exclude=*/0));
}

TEST(PolygonEventPointPartition, ApplyPrimaryNoOverlap) {
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  for (size_t i = 0; i < 10; ++i) {
    polygons.push_back(MakeTriangleForInterval(0, Point3(i, i, i),
                                               Point3(i + 1, i + 1, i + 1)));
  }
  PolygonEventPoint event_points[20];
  MakeEventPoints(/*dimension=*/0, polygons, event_points);
  CheckSorted(/*dimension=*/0, polygons, event_points);

  // `polygons` will have 10 entries. Put the first 4 in the negative child.
  PolygonEventPointPartition partition;
  partition.split_index = 7;
  partition.neg_poly_count = 4;
  partition.pos_poly_count = 6;

  // `ApplyPrimary` will clear `polygons`.
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons_copy(polygons);
  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<size_t> polygon_index_map(10);
  PolygonEventPoint neg_event_points[8];
  partition.ApplyPrimary(/*dimension=*/0, event_points, polygons,
                         polygon_index_map.data(), neg_event_points,
                         neg_polygons, pos_polygons);

  EXPECT_THAT(polygons, IsEmpty());
  ASSERT_THAT(neg_polygons, SizeIs(4));
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(neg_polygons[i], polygons_copy[i]);
  }
  ASSERT_THAT(pos_polygons, SizeIs(6));
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_EQ(pos_polygons[i], polygons_copy[i + 4]);
  }

  CheckSorted(/*dimension=*/0, neg_polygons, neg_event_points);
  CheckSorted(/*dimension=*/0, pos_polygons, event_points);
}

TEST(PolygonEventPointPartition, ApplyPrimarySingleOverlap) {
  /*  0 1 2
   *  |-|
   *  |---|
   *
   *    ^
   *    |
   */
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  polygons.push_back(MakeTriangleForInterval(0, Point3(0, 0, 0),
                                             Point3(1, 1, 1)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(0, 0, 0),
                                             Point3(2, 2, 2)));
  PolygonEventPoint event_points[4];
  MakeEventPoints(/*dimension=*/0, polygons, event_points);
  CheckSorted(/*dimension=*/0, polygons, event_points);

  PolygonEventPointPartition partition;
  partition.split_index = 2;
  partition.neg_poly_count = 2;
  partition.pos_poly_count = 1;

  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<size_t> polygon_index_map(2);
  PolygonEventPoint neg_event_points[4];
  partition.ApplyPrimary(/*dimension=*/0, event_points, polygons,
                         polygon_index_map.data(), neg_event_points,
                         neg_polygons, pos_polygons);

  EXPECT_THAT(polygons, IsEmpty());
  EXPECT_THAT(neg_polygons, SizeIs(2));
  for (const BSPPolygon<AABBConvexPolygon<>>& polygon : neg_polygons) {
    ASSERT_GT(polygon.vertex_count(), 0);
    EXPECT_EQ(polygon.min_vertex(/*dimension=*/0), Point3(0, 0, 0));
    EXPECT_THAT(polygon.max_vertex(/*dimension=*/1),
                AnyOf(Eq(Point3(1, 1, 1)), Eq(Point3(1, 2, 2))));
  }
  EXPECT_THAT(pos_polygons, SizeIs(1));
  for (const BSPPolygon<AABBConvexPolygon<>>& polygon : pos_polygons) {
    ASSERT_GT(polygon.vertex_count(), 0);
    EXPECT_EQ(polygon.min_vertex(/*dimension=*/1), Point3(1, 1, 1));
    EXPECT_EQ(polygon.max_vertex(/*dimension=*/1), Point3(2, 2, 2));
  }

  CheckSorted(/*dimension=*/0, neg_polygons, neg_event_points);
  CheckSorted(/*dimension=*/0, pos_polygons, event_points);
}

TEST(PolygonEventPointPartition, ApplyPrimaryTwoOverlaps) {
  /*  0 1 2 3
   *  |-|
   *  |---|
   *  |-----|
   *
   *    ^
   *    |
   */
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  polygons.push_back(MakeTriangleForInterval(0, Point3(0, 0, 0),
                                             Point3(1, 1, 1)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(0, 0, 0),
                                             Point3(2, 2, 2)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(0, 0, 0),
                                             Point3(3, 3, 3)));
  PolygonEventPoint event_points[6];
  MakeEventPoints(/*dimension=*/0, polygons, event_points);
  CheckSorted(/*dimension=*/0, polygons, event_points);

  PolygonEventPointPartition partition;
  partition.split_index = 3;
  partition.neg_poly_count = 3;
  partition.pos_poly_count = 2;

  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<size_t> polygon_index_map(3);
  PolygonEventPoint neg_event_points[6];
  partition.ApplyPrimary(/*dimension=*/0, event_points, polygons,
                         polygon_index_map.data(), neg_event_points,
                         neg_polygons, pos_polygons);

  EXPECT_THAT(polygons, IsEmpty());
  EXPECT_THAT(neg_polygons, SizeIs(3));
  EXPECT_THAT(pos_polygons, SizeIs(2));

  CheckSorted(/*dimension=*/0, neg_polygons, neg_event_points);
  CheckSorted(/*dimension=*/0, pos_polygons, event_points);
}

TEST(PolygonEventPointPartition, ApplyPrimaryGapAtSplit) {
  /*  0 1 2 3 4
   *  |-|
   *      |-|-|
   *
   *    ^
   *    |
   */
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  polygons.push_back(MakeTriangleForInterval(0, Point3(0, 0, 0),
                                             Point3(1, 1, 1)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(2, 2, 2),
                                             Point3(3, 3, 3)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(3, 3, 3),
                                             Point3(4, 4, 4)));
  PolygonEventPoint event_points[6];
  MakeEventPoints(/*dimension=*/0, polygons, event_points);
  CheckSorted(/*dimension=*/0, polygons, event_points);

  PolygonEventPointPartition partition;
  partition.split_index = 1;
  partition.neg_poly_count = 1;
  partition.pos_poly_count = 2;

  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<size_t> polygon_index_map(3);
  PolygonEventPoint neg_event_points[2];
  partition.ApplyPrimary(/*dimension=*/0, event_points, polygons,
                         polygon_index_map.data(), neg_event_points,
                         neg_polygons, pos_polygons);

  EXPECT_THAT(polygons, IsEmpty());
  EXPECT_THAT(neg_polygons, SizeIs(1));
  EXPECT_THAT(pos_polygons, SizeIs(2));

  CheckSorted(/*dimension=*/0, neg_polygons, neg_event_points);
  CheckSorted(/*dimension=*/0, pos_polygons, event_points);
}

}  // walnut
