#include "walnut/planar_range.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::ElementsAre;

TEST(PlanarRange, InputTooShort) {
  std::vector<Point3<32>> input;
  using InputIterator = std::vector<Point3<32>>::const_iterator;
  InputIterator remaining_begin;
  InputIterator remaining_end;
  PlanarRange<InputIterator> result;

  // Test empty input
  remaining_begin = input.begin();
  remaining_end = input.end();
  result.Build(remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, input.begin());
  EXPECT_EQ(remaining_end, input.end());
  EXPECT_EQ(result.size(), 0);
  EXPECT_EQ(std::vector<Point3<32>>(result.begin(), result.end()),
            input);

  // Test input with only 1 point
  input.emplace_back(1, 1, 1);
  remaining_begin = input.begin();
  remaining_end = input.end();
  result.Build(remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, remaining_end);
  EXPECT_EQ(result.size(), 1);
  EXPECT_EQ(result.plane(), Plane<>::Zero());
  EXPECT_EQ(std::vector<Point3<32>>(result.begin(), result.end()),
            input);

  // Test input with only 2 vertices
  input.emplace_back(2, 2, 2);
  remaining_begin = input.begin();
  remaining_end = input.end();
  result.Build(remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, remaining_end);
  EXPECT_EQ(result.size(), 2);
  EXPECT_EQ(result.plane(), Plane<>::Zero());
  EXPECT_EQ(std::vector<Point3<32>>(result.begin(), result.end()),
            input);

  // Test input with 3 vertices, but they're all collinear.
  input.emplace_back(3, 3, 3);
  remaining_begin = input.begin();
  remaining_end = input.end();
  result.Build(remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, remaining_end);
  EXPECT_EQ(result.size(), 3);
  EXPECT_EQ(result.plane(), Plane<>::Zero());
  EXPECT_EQ(std::vector<Point3<32>>(result.begin(), result.end()),
            input);
}

TEST(PolygonGetNextPlanar, AllPlanar) {
  std::vector<Point3<32>> input{Point3<32>(0, 0, 0), Point3<32>(1, 0, 0),
                                 Point3<32>(1, 1, 0)};
  using InputIterator = std::vector<Point3<32>>::const_iterator;
  InputIterator remaining_begin;
  InputIterator remaining_end;
  PlanarRange<InputIterator> result;

  // Try a triangle
  remaining_begin = input.begin();
  remaining_end = input.end();
  result.Build(remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, remaining_end);
  EXPECT_EQ(result.size(), input.size());
  EXPECT_TRUE(result.plane().normal().IsSameDir(Vector3<>(0, 0, 1)));
  EXPECT_EQ(std::vector<Point3<32>>(result.begin(), result.end()),
            input);

  // Add one more point to make a square
  input.emplace_back(0, 1, 0);
  result = PlanarRange<InputIterator>();
  remaining_begin = input.begin();
  remaining_end = input.end();
  result.Build(remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, remaining_end);
  EXPECT_EQ(result.size(), input.size());
  EXPECT_TRUE(result.plane().normal().IsSameDir(Vector3<>(0, 0, 1)));
  EXPECT_EQ(std::vector<Point3<32>>(result.begin(), result.end()),
            input);

  // Add one more point, but it's collinear with the first 2
  input.emplace_back(-1, 0, 0);
  result = PlanarRange<InputIterator>();
  remaining_begin = input.begin();
  remaining_end = input.end();
  result.Build(remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, remaining_end);
  EXPECT_EQ(result.size(), input.size());
  EXPECT_TRUE(result.plane().normal().IsSameDir(Vector3<>(0, 0, 1)));
  EXPECT_EQ(std::vector<Point3<32>>(result.begin(), result.end()),
            input);
}

TEST(PlanarRange, StopsAtNonplanar) {
  std::vector<Point3<32>> input{Point3<32>(1, 0, 0),
                                 Point3<32>(0, 1, 0),
                                 Point3<32>(0, -1, 1),
                                 Point3<32>(0, 0, 0)};
  using InputIterator = std::vector<Point3<32>>::const_iterator;
  InputIterator remaining_begin;
  InputIterator remaining_end;
  PlanarRange<InputIterator> result;

  remaining_begin = input.begin();
  remaining_end = input.end();
  result.Build(remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, input.begin() + 1);
  EXPECT_EQ(remaining_end, input.end());
  EXPECT_EQ(result.size(), 3);
  EXPECT_TRUE(result.plane().normal().IsSameDir(Vector3<>(0, 0, 1)));
  EXPECT_THAT(std::vector<Point3<32>>(result.begin(), result.end()),
              ElementsAre(input[0], input[1], input[3]));

  // Make sure it finds the next polygon too.
  result.Build(remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, remaining_end);
  EXPECT_EQ(result.size(), 3);
  EXPECT_TRUE(result.plane().normal().IsSameDir(Vector3<>(1, 0, 0)));
  EXPECT_THAT(std::vector<Point3<32>>(result.begin(), result.end()),
              ElementsAre(input[1], input[2], input[3]));
}

}  // walnut
