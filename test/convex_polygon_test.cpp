#include "walnut/convex_polygon.h"

#include "gtest/gtest.h"

namespace walnut {

TEST(ConvexPolygonGetNextPlanar, InputTooShort) {
  Vertex3<32> p1(1, 1, 1);
  ConvexPolygon<32>::PlaneRep plane;
  std::vector<Vertex3<32>>::iterator next_start;
  std::vector<Vertex3<32>>::iterator polygon_end;
  std::vector<Vertex3<32>> input;

  // Test empty input
  next_start = input.begin();
  polygon_end = ConvexPolygon<32>::GetNextPlanar(p1, next_start, input.end(),
                                                 plane);
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.end());

  // Test input with only 1 vertex
  input.emplace_back(2, 2, 2);
  next_start = input.begin();
  polygon_end = ConvexPolygon<32>::GetNextPlanar(p1, next_start, input.end(),
                                                 plane);
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.begin());

  // Test input with 2 vertices, but they're all colinear with p1.
  input.emplace_back(3, 3, 3);
  next_start = input.begin();
  polygon_end = ConvexPolygon<32>::GetNextPlanar(p1, next_start, input.end(),
                                                 plane);
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.begin());

  // Test input with 3 vertices, but they're all colinear with p1.
  input.emplace_back(4, 4, 4);
  next_start = input.begin();
  polygon_end = ConvexPolygon<32>::GetNextPlanar(p1, next_start, input.end(),
                                                 plane);
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.begin());
}

TEST(PolygonGetNextPlanar, AllPlanar) {
  Vertex3<32> p1(0, 0, 0);
  ConvexPolygon<32>::PlaneRep plane;
  std::vector<Vertex3<32>>::iterator next_start;
  std::vector<Vertex3<32>>::iterator polygon_end;
  std::vector<Vertex3<32>> input{Vertex3<32>(1, 0, 0), Vertex3<32>(0, 1, 0)};

  // Try a triangle
  next_start = input.begin();
  polygon_end = ConvexPolygon<32>::GetNextPlanar(p1, next_start, input.end(),
                                                 plane);
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.end());
  EXPECT_TRUE(plane.normal().IsSameDir(Vector3<>(0, 0, 1)));

  // Add one more point
  input.emplace_back(-1, 0, 0);
  next_start = input.begin();
  polygon_end = ConvexPolygon<32>::GetNextPlanar(p1, next_start, input.end(),
                                                 plane);
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.end());
  EXPECT_TRUE(plane.normal().IsSameDir(Vector3<>(0, 0, 1)));
}

TEST(ConvexPolygonGetNextPlanar, StopsAtNonplanar) {
  Vertex3<32> p1(0, 0, 0);
  ConvexPolygon<32>::PlaneRep plane;
  std::vector<Vertex3<32>>::iterator next_start;
  std::vector<Vertex3<32>>::iterator polygon_end;
  std::vector<Vertex3<32>> input{Vertex3<32>(1, 0, 0),
                                 Vertex3<32>(0, 1, 0),
                                 Vertex3<32>(0, -1, 1)};

  next_start = input.begin();
  polygon_end = ConvexPolygon<32>::GetNextPlanar(p1, next_start, input.end(),
                                                 plane);
  EXPECT_EQ(next_start, input.begin() + 1);
  EXPECT_EQ(polygon_end, input.begin() + 2);
  EXPECT_TRUE(plane.normal().IsSameDir(Vector3<>(0, 0, 1)));

  // Make sure it finds the next polygon too.
  polygon_end = ConvexPolygon<32>::GetNextPlanar(p1, next_start, input.end(),
                                                 plane);
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.end());
  EXPECT_TRUE(plane.normal().IsSameDir(Vector3<>(1, 0, 0)));
}

TEST(ConvexPolygonGetNextMonotone, StopsAtInitial) {
  //
  //        p1 -> q0
  //                \
  //     q4 -> q5    q1
  //       \        /
  //        q3 <- q2
  //
  Vertex3<32> p1(0, 0, 0);
  ConvexPolygon<32>::PlaneRep plane;
  std::vector<Vertex3<32>>::iterator next_start;
  std::vector<Vertex3<32>>::iterator polygon_end;
  std::vector<Vertex3<32>> input{Vertex3<32>(2, 0, 0),
                                 Vertex3<32>(3, 1, 0),
                                 Vertex3<32>(2, 2, 0),
                                 Vertex3<32>(0, 2, 0),
                                 Vertex3<32>(-1, 1, 0),
                                 Vertex3<32>(1, 1, 0)};

  next_start = input.begin();
  polygon_end = ConvexPolygon<32>::GetNextMonotone(/*monotone_dimension=*/0, p1,
                                                   next_start, input.end());
  EXPECT_EQ(next_start, input.begin() + 4);
  EXPECT_EQ(polygon_end, input.begin() + 5);

  // Make sure it finds the next polygon too.
  polygon_end = ConvexPolygon<32>::GetNextMonotone(/*monotone_dimension=*/0, p1,
                                                   next_start, input.end());
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.end());

  // Ensure GetNextMonotone returns the same result if all of the vertices are
  // flipped.
  for (Vertex3<32>& v : input) {
    v.x() = -v.x();
  }

  next_start = input.begin();
  polygon_end = ConvexPolygon<32>::GetNextMonotone(/*monotone_dimension=*/0, p1,
                                                   next_start, input.end());
  EXPECT_EQ(next_start, input.begin() + 4);
  EXPECT_EQ(polygon_end, input.begin() + 5);

  // Make sure it finds the next polygon too.
  polygon_end = ConvexPolygon<32>::GetNextMonotone(/*monotone_dimension=*/0, p1,
                                                   next_start, input.end());
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.end());
}

TEST(ConvexPolygonGetNextMonotone, StopsNonMonotone) {
  //
  //        p1 -> q0 -> q1
  //                      \
  //                       q2
  //                       |
  //                       q3
  //                      /
  //                    q4
  //                    |
  //                    q5
  //                      \
  //                       q6
  //
  Vertex3<32> p1(0, 0, 0);
  ConvexPolygon<32>::PlaneRep plane;
  std::vector<Vertex3<32>>::iterator next_start;
  std::vector<Vertex3<32>>::iterator polygon_end;
  std::vector<Vertex3<32>> input{Vertex3<32>(1, 0, 0),
                                 Vertex3<32>(2, 0, 0),
                                 Vertex3<32>(3, 1, 0),
                                 Vertex3<32>(3, 2, 0),
                                 Vertex3<32>(2, 3, 0),
                                 Vertex3<32>(2, 4, 0),
                                 Vertex3<32>(3, 5, 0)};

  next_start = input.begin();
  polygon_end = ConvexPolygon<32>::GetNextMonotone(/*monotone_dimension=*/0, p1,
                                                   next_start, input.end());
  EXPECT_EQ(next_start, input.begin() + 5);
  EXPECT_EQ(polygon_end, input.begin() + 6);

  // Make sure it finds the next polygon too.
  polygon_end = ConvexPolygon<32>::GetNextMonotone(/*monotone_dimension=*/0, p1,
                                                   next_start, input.end());
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.end());

  // Ensure GetNextMonotone returns the same result if all of the vertices are
  // flipped.
  for (Vertex3<32>& v : input) {
    v.x() = -v.x();
  }

  next_start = input.begin();
  polygon_end = ConvexPolygon<32>::GetNextMonotone(/*monotone_dimension=*/0, p1,
                                                   next_start, input.end());
  EXPECT_EQ(next_start, input.begin() + 5);
  EXPECT_EQ(polygon_end, input.begin() + 6);

  // Make sure it finds the next polygon too.
  polygon_end = ConvexPolygon<32>::GetNextMonotone(/*monotone_dimension=*/0, p1,
                                                   next_start, input.end());
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.end());
}

}  // walnut
