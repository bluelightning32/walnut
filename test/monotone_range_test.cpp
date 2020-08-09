#include "walnut/monotone_range.h"

#include "gtest/gtest.h"

namespace walnut {

TEST(MonotoneRange, StopsAtInitial) {
  //
  //        p1 -> q0
  //                \
  //     q4 -> q5    q1
  //       \        /
  //        q3 <- q2
  //
  Vertex3<32> p1(0, 0, 0);
  using Vertex3Iterator = std::vector<Vertex3<32>>::iterator;
  using MonotoneRange = MonotoneRange<Vertex3Iterator>;
  Vertex3Iterator next_start;
  Vertex3Iterator polygon_end;
  std::vector<Vertex3<32>> input{Vertex3<32>(2, 0, 0),
                                 Vertex3<32>(3, 1, 0),
                                 Vertex3<32>(2, 2, 0),
                                 Vertex3<32>(0, 2, 0),
                                 Vertex3<32>(-1, 1, 0),
                                 Vertex3<32>(1, 1, 0)};

  next_start = input.begin();
  polygon_end = MonotoneRange::GetNextMonotone(/*monotone_dimension=*/0, p1,
                                                   next_start, input.end());
  EXPECT_EQ(next_start, input.begin() + 4);
  EXPECT_EQ(polygon_end, input.begin() + 5);

  // Make sure it finds the next polygon too.
  polygon_end = MonotoneRange::GetNextMonotone(/*monotone_dimension=*/0, p1,
                                                   next_start, input.end());
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.end());

  // Ensure GetNextMonotone returns the same result if all of the vertices are
  // flipped.
  for (Vertex3<32>& v : input) {
    v.x() = -v.x();
  }

  next_start = input.begin();
  polygon_end = MonotoneRange::GetNextMonotone(/*monotone_dimension=*/0, p1,
                                                   next_start, input.end());
  EXPECT_EQ(next_start, input.begin() + 4);
  EXPECT_EQ(polygon_end, input.begin() + 5);

  // Make sure it finds the next polygon too.
  polygon_end = MonotoneRange::GetNextMonotone(/*monotone_dimension=*/0, p1,
                                                   next_start, input.end());
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.end());
}

TEST(MonotoneRange, StopsNonMonotone) {
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
  using Vertex3Iterator = std::vector<Vertex3<32>>::iterator;
  using MonotoneRange = MonotoneRange<Vertex3Iterator>;
  Vertex3Iterator next_start;
  Vertex3Iterator polygon_end;
  std::vector<Vertex3<32>> input{Vertex3<32>(1, 0, 0),
                                 Vertex3<32>(2, 0, 0),
                                 Vertex3<32>(3, 1, 0),
                                 Vertex3<32>(3, 2, 0),
                                 Vertex3<32>(2, 3, 0),
                                 Vertex3<32>(2, 4, 0),
                                 Vertex3<32>(3, 5, 0)};

  next_start = input.begin();
  polygon_end = MonotoneRange::GetNextMonotone(/*monotone_dimension=*/0, p1,
                                                   next_start, input.end());
  EXPECT_EQ(next_start, input.begin() + 5);
  EXPECT_EQ(polygon_end, input.begin() + 6);

  // Make sure it finds the next polygon too.
  polygon_end = MonotoneRange::GetNextMonotone(/*monotone_dimension=*/0, p1,
                                                   next_start, input.end());
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.end());

  // Ensure GetNextMonotone returns the same result if all of the vertices are
  // flipped.
  for (Vertex3<32>& v : input) {
    v.x() = -v.x();
  }

  next_start = input.begin();
  polygon_end = MonotoneRange::GetNextMonotone(/*monotone_dimension=*/0, p1,
                                                   next_start, input.end());
  EXPECT_EQ(next_start, input.begin() + 5);
  EXPECT_EQ(polygon_end, input.begin() + 6);

  // Make sure it finds the next polygon too.
  polygon_end = MonotoneRange::GetNextMonotone(/*monotone_dimension=*/0, p1,
                                                   next_start, input.end());
  EXPECT_EQ(next_start, input.end());
  EXPECT_EQ(polygon_end, input.end());
}

}  // walnut
