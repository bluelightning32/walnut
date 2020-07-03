#include "walnut/polygon.h"

#include "gtest/gtest.h"

namespace walnut {

TEST(PolygonGetNextPlanar, InputTooShort) {
  Vertex3<32> p1(1, 1, 1);
  Polygon<32>::PlaneRep plane;
  std::vector<Vertex3<32>>::iterator next_start;
  std::vector<Vertex3<32>>::iterator polygon_end;
  std::vector<Vertex3<32>> input;

  // Test empty input
  next_start = input.begin();
  polygon_end = Polygon<32>::GetNextPlanarPolygon(p1, next_start, input.end(),
                                                  plane);
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.end());

  // Test input with only 1 vertex
  input.emplace_back(2, 2, 2);
  next_start = input.begin();
  polygon_end = Polygon<32>::GetNextPlanarPolygon(p1, next_start, input.end(),
                                                  plane);
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.begin());

  // Test input with 2 vertices, but they're all colinear with p1.
  input.emplace_back(3, 3, 3);
  next_start = input.begin();
  polygon_end = Polygon<32>::GetNextPlanarPolygon(p1, next_start, input.end(),
                                                  plane);
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.begin());

  // Test input with 3 vertices, but they're all colinear with p1.
  input.emplace_back(4, 4, 4);
  next_start = input.begin();
  polygon_end = Polygon<32>::GetNextPlanarPolygon(p1, next_start, input.end(),
                                                  plane);
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.begin());
}

TEST(PolygonGetNextPlanar, AllPlanar) {
  Vertex3<32> p1(0, 0, 0);
  Polygon<32>::PlaneRep plane;
  std::vector<Vertex3<32>>::iterator next_start;
  std::vector<Vertex3<32>>::iterator polygon_end;
  std::vector<Vertex3<32>> input{Vertex3<32>(1, 0, 0), Vertex3<32>(0, 1, 0)};

  // Try a triangle
  next_start = input.begin();
  polygon_end = Polygon<32>::GetNextPlanarPolygon(p1, next_start, input.end(),
                                                  plane);
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.end());
  EXPECT_TRUE(plane.normal().IsSameDir(Vector<>(0, 0, 1)));

  // Add one more point
  input.emplace_back(-1, 0, 0);
  next_start = input.begin();
  polygon_end = Polygon<32>::GetNextPlanarPolygon(p1, next_start, input.end(),
                                                  plane);
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.end());
  EXPECT_TRUE(plane.normal().IsSameDir(Vector<>(0, 0, 1)));
}

TEST(PolygonGetNextPlanar, StopsAtNonplanar) {
  Vertex3<32> p1(0, 0, 0);
  Polygon<32>::PlaneRep plane;
  std::vector<Vertex3<32>>::iterator next_start;
  std::vector<Vertex3<32>>::iterator polygon_end;
  std::vector<Vertex3<32>> input{Vertex3<32>(1, 0, 0),
                                 Vertex3<32>(0, 1, 0),
                                 Vertex3<32>(0, -1, 1)};

  next_start = input.begin();
  polygon_end = Polygon<32>::GetNextPlanarPolygon(p1, next_start, input.end(),
                                                  plane);
  EXPECT_EQ(next_start, input.begin() + 1);
  EXPECT_EQ(polygon_end, input.begin() + 2);
  EXPECT_TRUE(plane.normal().IsSameDir(Vector<>(0, 0, 1)));

  // Make sure it finds the next polygon too.
  polygon_end = Polygon<32>::GetNextPlanarPolygon(p1, next_start, input.end(),
                                                  plane);
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.end());
  EXPECT_TRUE(plane.normal().IsSameDir(Vector<>(1, 0, 0)));
}

}  // walnut
