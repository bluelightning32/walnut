#include "walnut/polygon_event_point.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

AABBConvexPolygon<> MakeTriangleForInterval(const HomoPoint3& start,
                                            const HomoPoint3& end) {
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
  return AABBConvexPolygon<>(std::move(plane), drop_dimension, std::move(vertices));
}

TEST(MakeEventPoints, ConstructEmpty) {
  MakeEventPoints<ConvexPolygon<>>(/*dimension=*/0, /*polygon_count=*/0,
                                   /*polygons=*/nullptr, /*event_points=*/nullptr);
}

void CheckSorted(size_t dimension, size_t polygon_count,
                 const AABBConvexPolygon<>* polygons,
                 const PolygonEventPoint* event_points) {
  std::map<const AABBConvexPolygon<>*, size_t> seen;
  const HomoPoint3* prev = nullptr;
  bool seen_start_points = false;
  for (size_t i = 0; i < polygon_count*2; ++i) {
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
  for (size_t i = 0; i < polygon_count; ++i) {
    EXPECT_EQ(seen[&polygons[i]], 2);
  }
}

TEST(MakeEventPoints, ThreeOverlaps) {
  std::vector<AABBConvexPolygon<>> polygons;
  polygons.push_back(MakeTriangleForInterval(Point3(0, 0, 0),
                                             Point3(5, 5, 5)));
  polygons.push_back(MakeTriangleForInterval(Point3(1, 1, 1),
                                             Point3(4, 4, 4)));
  polygons.push_back(MakeTriangleForInterval(Point3(2, 2, 2),
                                             Point3(3, 3, 3)));

  for (size_t dimension = 0; dimension < 3; ++dimension) {
    PolygonEventPoint event_points[6];
    MakeEventPoints(dimension, polygons.size(), polygons.data(), event_points);
    CheckSorted(dimension, polygons.size(), polygons.data(), event_points);
  }
}

TEST(MakeEventPoints, EndBeforeStart) {
  std::vector<AABBConvexPolygon<>> polygons;
  polygons.push_back(MakeTriangleForInterval(Point3(0, 0, 0),
                                             Point3(1, 1, 1)));
  polygons.push_back(MakeTriangleForInterval(Point3(1, 1, 1),
                                             Point3(2, 2, 2)));

  for (size_t dimension = 0; dimension < 3; ++dimension) {
    PolygonEventPoint event_points[6];
    MakeEventPoints(dimension, polygons.size(), polygons.data(), event_points);
    CheckSorted(dimension, polygons.size(), polygons.data(), event_points);
  }
}

// Tests two sets of intervals, where the two sets do not touch in the middle.
TEST(MakeEventPoints, Discontinuity) {
  std::vector<AABBConvexPolygon<>> polygons;
  polygons.push_back(MakeTriangleForInterval(Point3(0, 0, 0),
                                             Point3(2, 2, 2)));
  polygons.push_back(MakeTriangleForInterval(Point3(1, 1, 1),
                                             Point3(2, 2, 2)));
  polygons.push_back(MakeTriangleForInterval(Point3(3, 3, 3),
                                             Point3(5, 5, 5)));
  polygons.push_back(MakeTriangleForInterval(Point3(4, 4, 4),
                                             Point3(5, 5, 5)));

  for (size_t dimension = 0; dimension < 3; ++dimension) {
    PolygonEventPoint event_points[8];
    MakeEventPoints(dimension, polygons.size(), polygons.data(), event_points);
    CheckSorted(dimension, polygons.size(), polygons.data(), event_points);
  }
}

// The polygons are sorted in different orders in different dimensions.
TEST(MakeEventPoints, DifferentDimensionSort) {
  std::vector<AABBConvexPolygon<>> polygons;
  polygons.push_back(MakeTriangleForInterval(Point3(0, 0, 0),
                                             Point3(1, 1, 1)));
  polygons.push_back(MakeTriangleForInterval(Point3(1, 2, 2),
                                             Point3(2, 3, 3)));
  polygons.push_back(MakeTriangleForInterval(Point3(2, 1, 1),
                                             Point3(3, 2, 2)));

  for (size_t dimension = 0; dimension < 3; ++dimension) {
    PolygonEventPoint event_points[8];
    MakeEventPoints(dimension, polygons.size(), polygons.data(), event_points);
    CheckSorted(dimension, polygons.size(), polygons.data(), event_points);
  }
}

}  // walnut
