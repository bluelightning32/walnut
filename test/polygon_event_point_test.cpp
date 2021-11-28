#include "walnut/polygon_event_point.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::AnyOf;
using testing::Eq;
using testing::IsEmpty;
using testing::SizeIs;

BSPPolygon<AABBConvexPolygon<>> MakeTriangle(
    BSPContentId id, const HomoPoint3& start, const HomoPoint3& middle,
    const HomoPoint3& end) {
  std::vector<HomoPoint3> vertices;
  vertices.push_back(start);
  vertices.push_back(middle);
  vertices.push_back(end);
  HalfSpace3 plane(vertices[0], vertices[1], vertices[2]);
  int drop_dimension = plane.normal().GetFirstNonzeroDimension();
  return BSPPolygon<AABBConvexPolygon<>>(
      id, /*on_node_plane=*/nullptr, /*pos_side=*/false,
      AABBConvexPolygon<>(std::move(plane), drop_dimension, std::move(vertices)));
}

BSPPolygon<AABBConvexPolygon<>> MakeTriangleForInterval(
    BSPContentId id, const HomoPoint3& start, const HomoPoint3& end) {
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
  return MakeTriangle(id, start,
                      HomoPoint3(middle_coords[0], middle_coords[1],
                                 middle_coords[2], start.w() * end.w()),
                      end);
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
  std::set<size_t> open_polygons_at_location;
  for (size_t i = 0; i < polygons.size()*2; ++i) {
    size_t& seen_polygon =
      seen[&event_points[i].GetPolygon(event_points, polygons)];
    EXPECT_EQ(event_points[i].start, seen_polygon == 0)
      << "i=" << i
      << " polygon=" << &event_points[i].GetPolygon(event_points, polygons) -
                        polygons.data();
    ++seen_polygon;
    EXPECT_LE(seen_polygon, 2);
    const HomoPoint3& location =
      event_points[i].GetLocation(dimension, event_points, polygons);
    if (i > 0) {
      EXPECT_NE(event_points[i].new_location,
                prev->IsEquivalentComponent(dimension, location));
      EXPECT_LE(prev->CompareComponent(dimension, location), 0)
        << "i=" << i << " prev=" << *prev << " location=" << location;
    } else {
      EXPECT_TRUE(event_points[0].new_location);
    }
    if (event_points[i].new_location) open_polygons_at_location.clear();
    if (event_points[i].start) {
      EXPECT_TRUE(
          open_polygons_at_location.insert(
            event_points[event_points[i].index.partner].index.content).second);
    } else {
      auto open_it =
        open_polygons_at_location.find(event_points[i].index.content);
      if (open_it != open_polygons_at_location.end()) {
        open_polygons_at_location.erase(open_it);
      } else {
        EXPECT_TRUE(open_polygons_at_location.empty());
      }
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

void CheckCoincidentVerticesAndEdges(
    const HalfSpace3* split_plane,
    bool pos_side,
    const std::vector<BSPPolygon<AABBConvexPolygon<>>>& polygons) {
  for (const BSPPolygon<AABBConvexPolygon<>>& polygon : polygons) {
    EXPECT_EQ(polygon.on_node_plane, SplitSide{});
    for (const auto& edge : polygon.edges()) {
      if (edge.line().IsCoincident(*split_plane)) {
        EXPECT_TRUE(split_plane->IsCoincident(edge.vertex()))
          << " split_plane=" << split_plane << std::endl;
        EXPECT_EQ(edge.edge_first_coincident, (SplitSide{split_plane,
                                                         pos_side}));
        EXPECT_EQ(edge.edge_last_coincident, (SplitSide{split_plane,
                                                        pos_side}));
      } else {
        EXPECT_EQ(edge.edge_first_coincident, SplitSide{});
        EXPECT_EQ(edge.edge_last_coincident, SplitSide{});
      }
      if (split_plane->IsCoincident(edge.vertex())) {
        EXPECT_EQ(edge.vertex_last_coincident, (SplitSide{split_plane,
                                                          pos_side}));
      } else {
        EXPECT_EQ(edge.vertex_last_coincident, SplitSide{});
      }
    }
  }
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
  partition.DiscountBorderPolygons(event_points, polygons.size());
  ASSERT_EQ(partition.neg_poly_count, 4);
  ASSERT_EQ(partition.pos_poly_count, 6);

  // `ApplyPrimary` will clear `polygons`.
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons_copy(polygons);
  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_border_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_border_polygons;
  std::vector<size_t> polygon_index_map(10);
  PolygonEventPoint neg_event_points[8];
  HalfSpace3 split_plane;
  partition.ApplyPrimary(/*dimension=*/0, event_points, &split_plane, polygons,
                         polygon_index_map.data(), neg_event_points,
                         neg_polygons, pos_polygons, neg_border_polygons,
                         pos_border_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, false, neg_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, true, pos_polygons);

  EXPECT_THAT(polygons, IsEmpty());
  ASSERT_THAT(neg_polygons, SizeIs(4));
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(neg_polygons[i], polygons_copy[i]);
  }
  ASSERT_THAT(pos_polygons, SizeIs(6));
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_EQ(pos_polygons[i], polygons_copy[i + 4]);
  }
  for (size_t i = 0; i < 10; ++i) {
    EXPECT_EQ(polygon_index_map[i], i);
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
  partition.DiscountBorderPolygons(event_points, polygons.size());
  ASSERT_EQ(partition.neg_poly_count, 2);
  ASSERT_EQ(partition.pos_poly_count, 1);

  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_border_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_border_polygons;
  std::vector<size_t> polygon_index_map(2);
  PolygonEventPoint neg_event_points[4];
  HalfSpace3 split_plane;
  partition.ApplyPrimary(/*dimension=*/0, event_points, &split_plane, polygons,
                         polygon_index_map.data(), neg_event_points,
                         neg_polygons, pos_polygons, neg_border_polygons,
                         pos_border_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, false, neg_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, true, pos_polygons);

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
  partition.DiscountBorderPolygons(event_points, polygons.size());

  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_border_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_border_polygons;
  std::vector<size_t> polygon_index_map(3);
  PolygonEventPoint neg_event_points[6];
  HalfSpace3 split_plane;
  partition.ApplyPrimary(/*dimension=*/0, event_points, &split_plane, polygons,
                         polygon_index_map.data(), neg_event_points,
                         neg_polygons, pos_polygons, neg_border_polygons,
                         pos_border_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, false, neg_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, true, pos_polygons);

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
  partition.DiscountBorderPolygons(event_points, polygons.size());

  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_border_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_border_polygons;
  std::vector<size_t> polygon_index_map(3);
  PolygonEventPoint neg_event_points[2];
  HalfSpace3 split_plane;
  partition.ApplyPrimary(/*dimension=*/0, event_points, &split_plane, polygons,
                         polygon_index_map.data(), neg_event_points,
                         neg_polygons, pos_polygons, neg_border_polygons,
                         pos_border_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, false, neg_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, true, pos_polygons);

  EXPECT_THAT(polygons, IsEmpty());
  EXPECT_THAT(neg_polygons, SizeIs(1));
  EXPECT_THAT(pos_polygons, SizeIs(2));

  CheckSorted(/*dimension=*/0, neg_polygons, neg_event_points);
  CheckSorted(/*dimension=*/0, pos_polygons, event_points);
}

TEST(PolygonEventPointPartition, ApplyBorderPolygons) {
  /*  x-view:
   *
   *  0 1 2 3 4
   *  |-|-|-|-|
   *
   *        ^
   *        |
   *
   *  y-view:
   *  2  -
   *     |
   *  1---  <- 0 height interval
   *   |
   *  0-
   *
   *
   */
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  polygons.push_back(MakeTriangle(0, Point3(0, 0, 0),
                                     Point3(1, 1, 0),
                                     Point3(1, 1, 1)));
  polygons.push_back(MakeTriangle(0, Point3(1, 1, 1),
                                     Point3(2, 1, 1),
                                     Point3(2, 1, 2)));
  polygons.push_back(MakeTriangle(0, Point3(3, 1, 3),
                                     Point3(3, 1, 2),
                                     Point3(2, 1, 2)));
  polygons.push_back(MakeTriangle(0, Point3(3, 1, 3),
                                     Point3(4, 2, 3),
                                     Point3(4, 2, 4)));
  const size_t parent_polygon_count = polygons.size();
  PolygonEventPoint primary_event_points[8];
  MakeEventPoints(/*dimension=*/1, polygons, primary_event_points);
  CheckSorted(/*dimension=*/1, polygons, primary_event_points);
  PolygonEventPoint secondary_event_points[8];
  MakeEventPoints(/*dimension=*/0, polygons, secondary_event_points);
  CheckSorted(/*dimension=*/0, polygons, secondary_event_points);

  PolygonEventPointPartition partition;
  partition.split_index = 5;
  partition.neg_poly_count = 3;
  partition.pos_poly_count = 1;
  partition.DiscountBorderPolygons(primary_event_points, polygons.size());
  ASSERT_EQ(partition.neg_poly_count, 1);
  ASSERT_EQ(partition.pos_poly_count, 1);

  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_border_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_border_polygons;
  std::vector<size_t> polygon_index_map(4);
  PolygonEventPoint primary_neg_event_points[2];
  HalfSpace3 split_plane;
  partition.ApplyPrimary(/*dimension=*/1, primary_event_points, &split_plane,
                         polygons, polygon_index_map.data(),
                         primary_neg_event_points, neg_polygons, pos_polygons,
                         neg_border_polygons, pos_border_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, false, neg_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, true, pos_polygons);

  EXPECT_THAT(neg_polygons, SizeIs(1));
  EXPECT_THAT(pos_polygons, SizeIs(1));
  EXPECT_THAT(neg_border_polygons, SizeIs(1));
  EXPECT_THAT(pos_border_polygons, SizeIs(1));
  for (const BSPPolygon<AABBConvexPolygon<>>& polygon : neg_polygons) {
    EXPECT_FALSE(polygon.plane().IsSameOrOpposite(split_plane));
  }
  for (const BSPPolygon<AABBConvexPolygon<>>& polygon : pos_polygons) {
    EXPECT_FALSE(polygon.plane().IsSameOrOpposite(split_plane));
  }
  for (const BSPPolygon<AABBConvexPolygon<>>& polygon : neg_border_polygons) {
    EXPECT_EQ(polygon.plane(), split_plane);
    EXPECT_EQ(polygon.on_node_plane.split, &split_plane);
    EXPECT_EQ(polygon.on_node_plane.pos_side, false);
    for (const auto& edge : polygon.edges()) {
      EXPECT_EQ(edge.edge_first_coincident, (SplitSide{&split_plane, false}));
      EXPECT_EQ(edge.edge_last_coincident, (SplitSide{&split_plane, false}));
      EXPECT_EQ(edge.vertex_last_coincident, (SplitSide{&split_plane, false}));
    }
  }
  for (const BSPPolygon<AABBConvexPolygon<>>& polygon : pos_border_polygons) {
    EXPECT_EQ(polygon.plane(), -split_plane);
    EXPECT_EQ(polygon.on_node_plane.split, &split_plane);
    EXPECT_EQ(polygon.on_node_plane.pos_side, true);
    for (const auto& edge : polygon.edges()) {
      EXPECT_EQ(edge.edge_first_coincident, (SplitSide{&split_plane, true}));
      EXPECT_EQ(edge.edge_last_coincident, (SplitSide{&split_plane, true}));
      EXPECT_EQ(edge.vertex_last_coincident, (SplitSide{&split_plane, true}));
    }
  }
  CheckCoincidentVerticesAndEdges(&split_plane, false, neg_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, true, pos_polygons);
  CheckSorted(/*dimension=*/1, neg_polygons, primary_neg_event_points);
  CheckSorted(/*dimension=*/1, pos_polygons, primary_event_points);

  PolygonEventPoint secondary_neg_event_points[2];
  PolygonEventPoint secondary_pos_event_points[2];
  std::vector<PolygonMergeEvent> merge_heap;
  partition.ApplySecondary(/*dimension=*/0, parent_polygon_count, neg_polygons,
                           pos_polygons, polygon_index_map.data(),
                           secondary_event_points, secondary_neg_event_points,
                           secondary_pos_event_points, merge_heap);
  ASSERT_TRUE(secondary_neg_event_points[0].start);
  ASSERT_TRUE(secondary_pos_event_points[0].start);
  CheckSorted(/*dimension=*/0, neg_polygons, primary_neg_event_points);
  CheckSorted(/*dimension=*/0, pos_polygons, primary_event_points);
}

TEST(PolygonEventPointPartition, ApplySecondaryNoOverlap) {
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  for (size_t i = 0; i < 10; ++i) {
    polygons.push_back(MakeTriangleForInterval(0, Point3(i, i, i),
                                               Point3(i + 1, i + 1, i + 1)));
  }
  PolygonEventPoint primary_event_points[20];
  MakeEventPoints(/*dimension=*/0, polygons, primary_event_points);
  PolygonEventPoint secondary_event_points[20];
  MakeEventPoints(/*dimension=*/1, polygons, secondary_event_points);
  CheckSorted(/*dimension=*/1, polygons, secondary_event_points);

  // `polygons` will have 10 entries. Put the first 4 in the negative child.
  PolygonEventPointPartition partition;
  partition.split_index = 7;
  partition.neg_poly_count = 4;
  partition.pos_poly_count = 6;
  partition.DiscountBorderPolygons(primary_event_points, polygons.size());

  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_border_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_border_polygons;
  std::vector<size_t> polygon_index_map(10);
  PolygonEventPoint primary_neg_event_points[8];
  const size_t parent_polygon_count = polygons.size();
  HalfSpace3 split_plane;
  partition.ApplyPrimary(/*dimension=*/0, primary_event_points, &split_plane,
                         polygons, polygon_index_map.data(),
                         primary_neg_event_points, neg_polygons, pos_polygons,
                         neg_border_polygons, pos_border_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, false, neg_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, true, pos_polygons);

  PolygonEventPoint secondary_neg_event_points[8];
  PolygonEventPoint secondary_pos_event_points[12];
  std::vector<PolygonMergeEvent> merge_heap;
  partition.ApplySecondary(/*dimension=*/1, parent_polygon_count, neg_polygons,
                           pos_polygons, polygon_index_map.data(),
                           secondary_event_points, secondary_neg_event_points,
                           secondary_pos_event_points, merge_heap);
  ASSERT_TRUE(secondary_neg_event_points[0].start);
  ASSERT_TRUE(secondary_pos_event_points[0].start);

  CheckSorted(/*dimension=*/1, neg_polygons, secondary_neg_event_points);
  CheckSorted(/*dimension=*/1, pos_polygons, secondary_pos_event_points);
}

TEST(PolygonEventPointPartition, ApplySecondaryReverseNoOverlap) {
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  for (size_t i = 0; i < 10; ++i) {
    polygons.push_back(MakeTriangleForInterval(0, Point3(i, i + 1, i),
                                               Point3(i + 1, i, i + 1)));
  }
  PolygonEventPoint primary_event_points[20];
  MakeEventPoints(/*dimension=*/0, polygons, primary_event_points);
  PolygonEventPoint secondary_event_points[20];
  MakeEventPoints(/*dimension=*/1, polygons, secondary_event_points);
  CheckSorted(/*dimension=*/1, polygons, secondary_event_points);

  // `polygons` will have 10 entries. Put the first 4 in the negative child.
  PolygonEventPointPartition partition;
  partition.split_index = 7;
  partition.neg_poly_count = 4;
  partition.pos_poly_count = 6;
  partition.DiscountBorderPolygons(primary_event_points, polygons.size());

  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_border_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_border_polygons;
  std::vector<size_t> polygon_index_map(10);
  PolygonEventPoint primary_neg_event_points[8];
  const size_t parent_polygon_count = polygons.size();
  HalfSpace3 split_plane;
  partition.ApplyPrimary(/*dimension=*/0, primary_event_points, &split_plane,
                         polygons, polygon_index_map.data(),
                         primary_neg_event_points, neg_polygons, pos_polygons,
                         neg_border_polygons, pos_border_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, false, neg_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, true, pos_polygons);

  PolygonEventPoint secondary_neg_event_points[8];
  PolygonEventPoint secondary_pos_event_points[12];
  std::vector<PolygonMergeEvent> merge_heap;
  partition.ApplySecondary(/*dimension=*/1, parent_polygon_count, neg_polygons,
                           pos_polygons, polygon_index_map.data(),
                           secondary_event_points, secondary_neg_event_points,
                           secondary_pos_event_points, merge_heap);
  ASSERT_TRUE(secondary_neg_event_points[0].start);
  ASSERT_TRUE(secondary_pos_event_points[0].start);

  CheckSorted(/*dimension=*/1, neg_polygons, secondary_neg_event_points);
  CheckSorted(/*dimension=*/1, pos_polygons, secondary_pos_event_points);
}

TEST(PolygonEventPointPartition, ApplySecondaryTwoOverlaps) {
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
  PolygonEventPoint secondary_event_points[6];
  MakeEventPoints(/*dimension=*/1, polygons, secondary_event_points);
  CheckSorted(/*dimension=*/1, polygons, secondary_event_points);

  PolygonEventPointPartition partition;
  partition.split_index = 3;
  partition.neg_poly_count = 3;
  partition.pos_poly_count = 2;
  partition.DiscountBorderPolygons(event_points, polygons.size());

  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_border_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_border_polygons;
  std::vector<size_t> polygon_index_map(3);
  PolygonEventPoint neg_event_points[6];
  HalfSpace3 split_plane;
  partition.ApplyPrimary(/*dimension=*/0, event_points, &split_plane,
                         polygons, polygon_index_map.data(), neg_event_points,
                         neg_polygons, pos_polygons, neg_border_polygons,
                         pos_border_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, false, neg_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, true, pos_polygons);

  EXPECT_THAT(neg_polygons, SizeIs(3));
  EXPECT_THAT(pos_polygons, SizeIs(2));

  PolygonEventPoint secondary_neg_event_points[6];
  PolygonEventPoint secondary_pos_event_points[4];
  std::vector<PolygonMergeEvent> merge_heap;
  partition.ApplySecondary(/*dimension=*/1, polygon_index_map.size(),
                           neg_polygons, pos_polygons,
                           polygon_index_map.data(), secondary_event_points,
                           secondary_neg_event_points,
                           secondary_pos_event_points, merge_heap);
  ASSERT_TRUE(secondary_neg_event_points[0].start);
  ASSERT_TRUE(secondary_pos_event_points[0].start);

  CheckSorted(/*dimension=*/1, neg_polygons, secondary_neg_event_points);
  CheckSorted(/*dimension=*/1, pos_polygons, secondary_pos_event_points);
}

TEST(PolygonEventPointPartition, ApplySecondaryTwoOverlapsRightAngleDown) {
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
                                             Point3(1, -1, -1)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(0, 0, 0),
                                             Point3(2, -2, -2)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(0, 0, 0),
                                             Point3(3, -3, -3)));
  PolygonEventPoint event_points[6];
  MakeEventPoints(/*dimension=*/0, polygons, event_points);
  CheckSorted(/*dimension=*/0, polygons, event_points);
  PolygonEventPoint secondary_event_points[6];
  MakeEventPoints(/*dimension=*/1, polygons, secondary_event_points);
  CheckSorted(/*dimension=*/1, polygons, secondary_event_points);

  PolygonEventPointPartition partition;
  partition.split_index = 3;
  partition.neg_poly_count = 3;
  partition.pos_poly_count = 2;
  partition.DiscountBorderPolygons(event_points, polygons.size());

  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_border_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_border_polygons;
  std::vector<size_t> polygon_index_map(3);
  PolygonEventPoint neg_event_points[6];
  HalfSpace3 split_plane;
  partition.ApplyPrimary(/*dimension=*/0, event_points, &split_plane, polygons,
                         polygon_index_map.data(), neg_event_points,
                         neg_polygons, pos_polygons, neg_border_polygons,
                         pos_border_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, false, neg_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, true, pos_polygons);

  EXPECT_THAT(neg_polygons, SizeIs(3));
  EXPECT_THAT(pos_polygons, SizeIs(2));

  PolygonEventPoint secondary_neg_event_points[6];
  PolygonEventPoint secondary_pos_event_points[4];
  std::vector<PolygonMergeEvent> merge_heap;
  partition.ApplySecondary(/*dimension=*/1, polygon_index_map.size(),
                           neg_polygons, pos_polygons,
                           polygon_index_map.data(), secondary_event_points,
                           secondary_neg_event_points,
                           secondary_pos_event_points, merge_heap);
  ASSERT_TRUE(secondary_neg_event_points[0].start);
  ASSERT_TRUE(secondary_pos_event_points[0].start);

  CheckSorted(/*dimension=*/1, neg_polygons, secondary_neg_event_points);
  CheckSorted(/*dimension=*/1, pos_polygons, secondary_pos_event_points);
}

TEST(PolygonEventPointPartition, ApplySecondaryTwoOverlapsSkewedDown) {
  /*  0 1 2 3
   *  |-|
   *  |---|
   *  |-----|
   *
   *    ^
   *    |
   *
   * 5 +\
   * 4 | 0-\
   * 3 |/ 1 \
   * 2 ||/   2
   * 1 |/_--/
   * 0 +/
   *   0 1 2 3
   *
   *     ^
   *     |
   */
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  polygons.push_back(MakeTriangle(0,
                                  Point3(0, 0, 0),
                                  Point3(0, 5, 5),
                                  Point3(1, 4, 4)));
  polygons.push_back(MakeTriangle(0,
                                  Point3(0, 0, 0),
                                  Point3(0, 5, 5),
                                  Point3(2, 3, 3)));
  polygons.push_back(MakeTriangle(0,
                                  Point3(0, 0, 0),
                                  Point3(0, 5, 5),
                                  Point3(3, 2, 2)));
  PolygonEventPoint event_points[6];
  MakeEventPoints(/*dimension=*/0, polygons, event_points);
  CheckSorted(/*dimension=*/0, polygons, event_points);
  PolygonEventPoint secondary_event_points[6];
  MakeEventPoints(/*dimension=*/1, polygons, secondary_event_points);
  CheckSorted(/*dimension=*/1, polygons, secondary_event_points);

  PolygonEventPointPartition partition;
  partition.split_index = 3;
  partition.neg_poly_count = 3;
  partition.pos_poly_count = 2;
  partition.DiscountBorderPolygons(event_points, polygons.size());

  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_border_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_border_polygons;
  std::vector<size_t> polygon_index_map(3);
  PolygonEventPoint neg_event_points[6];
  HalfSpace3 split_plane;
  partition.ApplyPrimary(/*dimension=*/0, event_points, &split_plane, polygons,
                         polygon_index_map.data(), neg_event_points,
                         neg_polygons, pos_polygons, neg_border_polygons,
                         pos_border_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, false, neg_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, true, pos_polygons);

  EXPECT_THAT(neg_polygons, SizeIs(3));
  EXPECT_THAT(pos_polygons, SizeIs(2));

  PolygonEventPoint secondary_neg_event_points[6];
  PolygonEventPoint secondary_pos_event_points[4];
  std::vector<PolygonMergeEvent> merge_heap;
  partition.ApplySecondary(/*dimension=*/1, polygon_index_map.size(),
                           neg_polygons, pos_polygons,
                           polygon_index_map.data(), secondary_event_points,
                           secondary_neg_event_points,
                           secondary_pos_event_points, merge_heap);
  ASSERT_TRUE(secondary_neg_event_points[0].start);
  ASSERT_TRUE(secondary_pos_event_points[0].start);

  CheckSorted(/*dimension=*/1, neg_polygons, secondary_neg_event_points);
  CheckSorted(/*dimension=*/1, pos_polygons, secondary_pos_event_points);
}

TEST(PolygonEventPointPartition, ApplySecondaryTwoOverlapsHorzFlipped) {
  /*  0 1 2 3
   *  |-|
   *  |---|
   *  |-----|
   *
   *    ^
   *    |
   *
   */
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  polygons.push_back(MakeTriangle(0,
                                  Point3(0, 0, 0),
                                  Point3(1, -1, -1),
                                  Point3(1, 1, 1)));
  polygons.push_back(MakeTriangle(0,
                                  Point3(0, 0, 0),
                                  Point3(2, -1, -1),
                                  Point3(2, 2, 2)));
  polygons.push_back(MakeTriangle(0,
                                  Point3(0, 0, 0),
                                  Point3(3, -1, -1),
                                  Point3(3, 3, 3)));
  PolygonEventPoint event_points[6];
  MakeEventPoints(/*dimension=*/0, polygons, event_points);
  CheckSorted(/*dimension=*/0, polygons, event_points);
  PolygonEventPoint secondary_event_points[6];
  MakeEventPoints(/*dimension=*/1, polygons, secondary_event_points);
  CheckSorted(/*dimension=*/1, polygons, secondary_event_points);

  PolygonEventPointPartition partition;
  partition.split_index = 3;
  partition.neg_poly_count = 3;
  partition.pos_poly_count = 2;
  partition.DiscountBorderPolygons(event_points, polygons.size());

  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_border_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_border_polygons;
  std::vector<size_t> polygon_index_map(3);
  PolygonEventPoint neg_event_points[6];
  HalfSpace3 split_plane;
  partition.ApplyPrimary(/*dimension=*/0, event_points, &split_plane, polygons,
                         polygon_index_map.data(), neg_event_points,
                         neg_polygons, pos_polygons, neg_border_polygons,
                         pos_border_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, false, neg_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, true, pos_polygons);

  EXPECT_THAT(neg_polygons, SizeIs(3));
  EXPECT_THAT(pos_polygons, SizeIs(2));

  PolygonEventPoint secondary_neg_event_points[6];
  PolygonEventPoint secondary_pos_event_points[4];
  std::vector<PolygonMergeEvent> merge_heap;
  partition.ApplySecondary(/*dimension=*/1, polygon_index_map.size(),
                           neg_polygons, pos_polygons,
                           polygon_index_map.data(), secondary_event_points,
                           secondary_neg_event_points,
                           secondary_pos_event_points, merge_heap);
  ASSERT_TRUE(secondary_neg_event_points[0].start);
  ASSERT_TRUE(secondary_pos_event_points[0].start);

  CheckSorted(/*dimension=*/1, neg_polygons, secondary_neg_event_points);
  CheckSorted(/*dimension=*/1, pos_polygons, secondary_pos_event_points);
}

TEST(PolygonEventPointPartition, ApplySecondaryTwoOverlapsSkewedDownMore) {
  /*  0 1 2 3
   *  |-|
   *  |---|
   *  |-----|
   *
   *    ^
   *    |
   *
   */
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  polygons.push_back(MakeTriangle(0,
                                  Point3(0, 0, 0),
                                  Point3(0, 1, 1),
                                  Point3(1, -1, -1)));
  polygons.push_back(MakeTriangle(0,
                                  Point3(0, 0, 0),
                                  Point3(0, 1, 1),
                                  Point3(2, -2, -2)));
  polygons.push_back(MakeTriangle(0,
                                  Point3(0, 0, 0),
                                  Point3(0, 1, 1),
                                  Point3(3, -3, -3)));
  PolygonEventPoint event_points[6];
  MakeEventPoints(/*dimension=*/0, polygons, event_points);
  CheckSorted(/*dimension=*/0, polygons, event_points);
  PolygonEventPoint secondary_event_points[6];
  MakeEventPoints(/*dimension=*/1, polygons, secondary_event_points);
  CheckSorted(/*dimension=*/1, polygons, secondary_event_points);

  PolygonEventPointPartition partition;
  partition.split_index = 3;
  partition.neg_poly_count = 3;
  partition.pos_poly_count = 2;
  partition.DiscountBorderPolygons(event_points, polygons.size());

  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_border_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_border_polygons;
  std::vector<size_t> polygon_index_map(3);
  PolygonEventPoint neg_event_points[6];
  HalfSpace3 split_plane;
  partition.ApplyPrimary(/*dimension=*/0, event_points, &split_plane, polygons,
                         polygon_index_map.data(), neg_event_points,
                         neg_polygons, pos_polygons, neg_border_polygons,
                         pos_border_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, false, neg_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, true, pos_polygons);

  EXPECT_THAT(neg_polygons, SizeIs(3));
  EXPECT_THAT(pos_polygons, SizeIs(2));

  PolygonEventPoint secondary_neg_event_points[6];
  PolygonEventPoint secondary_pos_event_points[4];
  std::vector<PolygonMergeEvent> merge_heap;
  partition.ApplySecondary(/*dimension=*/1, polygon_index_map.size(),
                           neg_polygons, pos_polygons,
                           polygon_index_map.data(), secondary_event_points,
                           secondary_neg_event_points,
                           secondary_pos_event_points, merge_heap);
  ASSERT_TRUE(secondary_neg_event_points[0].start);
  ASSERT_TRUE(secondary_pos_event_points[0].start);

  CheckSorted(/*dimension=*/1, neg_polygons, secondary_neg_event_points);
  CheckSorted(/*dimension=*/1, pos_polygons, secondary_pos_event_points);
}

TEST(PolygonEventPointPartition, ApplySecondaryOneOverlap) {
  /*  0 1 2 3 4 5 6
   *  |-|-| |-|-|-|
   *      |-----|
   *
   *          ^
   *          |
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

  PolygonEventPoint secondary_event_points[12];
  MakeEventPoints(/*dimension=*/1, polygons, secondary_event_points);
  CheckSorted(/*dimension=*/1, polygons, secondary_event_points);

  PolygonEventPointPartition partition;
  partition.split_index = 6;
  partition.neg_poly_count = 4;
  partition.pos_poly_count = 3;
  partition.DiscountBorderPolygons(event_points, polygons.size());

  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_border_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_border_polygons;
  std::vector<size_t> polygon_index_map(6);
  PolygonEventPoint neg_event_points[8];
  HalfSpace3 split_plane;
  partition.ApplyPrimary(/*dimension=*/0, event_points, &split_plane, polygons,
                         polygon_index_map.data(), neg_event_points,
                         neg_polygons, pos_polygons, neg_border_polygons,
                         pos_border_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, false, neg_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, true, pos_polygons);

  EXPECT_THAT(neg_polygons, SizeIs(4));
  EXPECT_THAT(pos_polygons, SizeIs(3));

  PolygonEventPoint secondary_neg_event_points[8];
  PolygonEventPoint secondary_pos_event_points[6];
  std::vector<PolygonMergeEvent> merge_heap;
  partition.ApplySecondary(/*dimension=*/1, polygon_index_map.size(),
                           neg_polygons, pos_polygons,
                           polygon_index_map.data(), secondary_event_points,
                           secondary_neg_event_points,
                           secondary_pos_event_points, merge_heap);
  ASSERT_TRUE(secondary_neg_event_points[0].start);
  ASSERT_TRUE(secondary_pos_event_points[0].start);

  CheckSorted(/*dimension=*/1, neg_polygons, secondary_neg_event_points);
  CheckSorted(/*dimension=*/1, pos_polygons, secondary_pos_event_points);
}

TEST(PolygonEventPointPartition, ApplySecondaryEmptyInterval) {
  /*  x-view:
   *
   *  0 1 2 3 4
   *  |-|-|-|-|
   *
   *        ^
   *        |
   *
   *  y-view:
   *  2  -
   *     |
   *  1---  <- 0 height interval
   *   |
   *  0-
   *
   *
   */
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  polygons.push_back(MakeTriangleForInterval(0, Point3(0, 0, 0),
                                             Point3(1, 1, 1)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(1, 1, 1),
                                             Point3(2, 1, 2)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(2, 1, 2),
                                             Point3(3, 1, 3)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(3, 1, 3),
                                             Point3(4, 2, 4)));
  PolygonEventPoint event_points[8];
  MakeEventPoints(/*dimension=*/0, polygons, event_points);
  CheckSorted(/*dimension=*/0, polygons, event_points);
  PolygonEventPoint secondary_event_points[8];
  MakeEventPoints(/*dimension=*/1, polygons, secondary_event_points);
  CheckSorted(/*dimension=*/1, polygons, secondary_event_points);

  PolygonEventPointPartition partition;
  partition.split_index = 5;
  partition.neg_poly_count = 3;
  partition.pos_poly_count = 1;
  partition.DiscountBorderPolygons(event_points, polygons.size());

  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_border_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_border_polygons;
  std::vector<size_t> polygon_index_map(4);
  PolygonEventPoint neg_event_points[6];
  HalfSpace3 split_plane;
  partition.ApplyPrimary(/*dimension=*/0, event_points, &split_plane, polygons,
                         polygon_index_map.data(), neg_event_points,
                         neg_polygons, pos_polygons, neg_border_polygons,
                         pos_border_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, false, neg_polygons);
  CheckCoincidentVerticesAndEdges(&split_plane, true, pos_polygons);

  EXPECT_THAT(neg_polygons, SizeIs(3));
  EXPECT_THAT(pos_polygons, SizeIs(1));

  PolygonEventPoint secondary_neg_event_points[6];
  PolygonEventPoint secondary_pos_event_points[2];
  std::vector<PolygonMergeEvent> merge_heap;
  partition.ApplySecondary(/*dimension=*/1, polygon_index_map.size(),
                           neg_polygons, pos_polygons,
                           polygon_index_map.data(), secondary_event_points,
                           secondary_neg_event_points,
                           secondary_pos_event_points, merge_heap);
  ASSERT_TRUE(secondary_neg_event_points[0].start);
  ASSERT_TRUE(secondary_pos_event_points[0].start);

  CheckSorted(/*dimension=*/1, neg_polygons, secondary_neg_event_points);
  CheckSorted(/*dimension=*/1, pos_polygons, secondary_pos_event_points);
}

TEST(PolygonEventPointPartition, ApplySecondaryOverlapGapAtSplit) {
  /*  0 1 2 3 4
   *  |-----|
   *  |-|
   *      |-|-|
   *
   *    ^
   *    |
   */
  std::vector<BSPPolygon<AABBConvexPolygon<>>> polygons;
  polygons.push_back(MakeTriangleForInterval(0, Point3(0, 0, 0),
                                             Point3(3, 3, 3)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(0, 0, 0),
                                             Point3(1, 1, 1)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(2, 2, 2),
                                             Point3(3, 3, 3)));
  polygons.push_back(MakeTriangleForInterval(0, Point3(3, 3, 3),
                                             Point3(4, 4, 4)));
  PolygonEventPoint event_points[8];
  MakeEventPoints(/*dimension=*/0, polygons, event_points);
  CheckSorted(/*dimension=*/0, polygons, event_points);
  PolygonEventPoint secondary_event_points[8];
  MakeEventPoints(/*dimension=*/1, polygons, secondary_event_points);
  CheckSorted(/*dimension=*/1, polygons, secondary_event_points);

  PolygonEventPointPartition partition;
  partition.split_index = 2;
  partition.neg_poly_count = 2;
  partition.pos_poly_count = 3;
  partition.DiscountBorderPolygons(event_points, polygons.size());

  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> neg_border_polygons;
  std::vector<BSPPolygon<AABBConvexPolygon<>>> pos_border_polygons;
  std::vector<size_t> polygon_index_map(4);
  PolygonEventPoint neg_event_points[4];
  HalfSpace3 split_plane;
  partition.ApplyPrimary(/*dimension=*/0, event_points, &split_plane, polygons,
                         polygon_index_map.data(), neg_event_points,
                         neg_polygons, pos_polygons, neg_border_polygons,
                         pos_border_polygons);

  PolygonEventPoint secondary_neg_event_points[4];
  PolygonEventPoint secondary_pos_event_points[6];
  std::vector<PolygonMergeEvent> merge_heap;
  partition.ApplySecondary(/*dimension=*/1, polygon_index_map.size(),
                           neg_polygons, pos_polygons,
                           polygon_index_map.data(), secondary_event_points,
                           secondary_neg_event_points,
                           secondary_pos_event_points, merge_heap);
  ASSERT_TRUE(secondary_neg_event_points[0].start);
  ASSERT_TRUE(secondary_pos_event_points[0].start);

  CheckSorted(/*dimension=*/1, neg_polygons, secondary_neg_event_points);
  CheckSorted(/*dimension=*/1, pos_polygons, secondary_pos_event_points);
}

}  // walnut
