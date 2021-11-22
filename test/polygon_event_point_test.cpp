#include "walnut/polygon_event_point.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::AnyOf;
using testing::Eq;

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

}  // walnut
