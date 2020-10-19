#include "walnut/monotone_range.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::ElementsAre;

TEST(MonotoneRange, Triangle) {
  //
  //        p0 ->  p1
  //         \     /
  //           p2 L
  //
  using Point3Iterator = std::vector<Point3<32>>::iterator;
  using MonotoneRange = MonotoneRange<Point3Iterator>;
  using ConcatRange = MonotoneRange::ConcatRangeRep;
  std::vector<Point3<32>> input{Point3<32>(17, 1, 0),
                                 Point3<32>(37, 1, 0),
                                 Point3<32>(27, 0, 0)};

  Point3Iterator remaining_begin = input.begin();
  Point3Iterator remaining_end = input.end();
  MonotoneRange range;
  range.Build(/*monotone_dimension=*/0, remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, remaining_end);

  ConcatRange::const_iterator chain1_begin;
  ConcatRange::const_iterator chain1_end;
  ConcatRange::const_reverse_iterator chain2_begin;
  ConcatRange::const_reverse_iterator chain2_end;
  range.GetChains(chain1_begin, chain1_end, chain2_begin, chain2_end);

  std::vector<Point3<32>> chain1(chain1_begin, chain1_end);
  EXPECT_THAT(chain1, ElementsAre(input[0], input[1]));

  std::vector<Point3<32>> chain2(chain2_begin, chain2_end);
  EXPECT_THAT(chain2, ElementsAre(input[0], input[2], input[1]));
}

TEST(MonotoneRange, TiltedSquare) {
  //
  //          > p1
  //         /     \
  //        p0     p2
  //         \     /
  //           p3 L
  //
  using Point3Iterator = std::vector<Point3<32>>::iterator;
  using MonotoneRange = MonotoneRange<Point3Iterator>;
  using ConcatRange = MonotoneRange::ConcatRangeRep;
  std::vector<Point3<32>> input{Point3<32>(17, 0, 0),
                                 Point3<32>(27, 1, 0),
                                 Point3<32>(37, 0, 0),
                                 Point3<32>(27, -1, 0)};

  Point3Iterator remaining_begin = input.begin();
  Point3Iterator remaining_end = input.end();
  MonotoneRange range;
  range.Build(/*monotone_dimension=*/0, remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, remaining_end);

  ConcatRange::const_iterator chain1_begin;
  ConcatRange::const_iterator chain1_end;
  ConcatRange::const_reverse_iterator chain2_begin;
  ConcatRange::const_reverse_iterator chain2_end;
  range.GetChains(chain1_begin, chain1_end, chain2_begin, chain2_end);

  std::vector<Point3<32>> chain1(chain1_begin, chain1_end);
  EXPECT_THAT(chain1, ElementsAre(input[0], input[1], input[2]));

  std::vector<Point3<32>> chain2(chain2_begin, chain2_end);
  EXPECT_THAT(chain2, ElementsAre(input[0], input[3], input[2]));
}

TEST(MonotoneRange, Square) {
  //
  //  p0 ->  p1
  //  /\
  //  |      |
  //         v
  //  p3 <-  p2
  //
  using Point3Iterator = std::vector<Point3<32>>::iterator;
  using MonotoneRange = MonotoneRange<Point3Iterator>;
  using ConcatRange = MonotoneRange::ConcatRangeRep;
  std::vector<Point3<32>> input{Point3<32>(17, 1, 0),
                                 Point3<32>(27, 1, 0),
                                 Point3<32>(27, 0, 0),
                                 Point3<32>(17, 0, 0)};

  Point3Iterator remaining_begin = input.begin();
  Point3Iterator remaining_end = input.end();
  MonotoneRange range;
  range.Build(/*monotone_dimension=*/0, remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, remaining_end);

  ConcatRange::const_iterator chain1_begin;
  ConcatRange::const_iterator chain1_end;
  ConcatRange::const_reverse_iterator chain2_begin;
  ConcatRange::const_reverse_iterator chain2_end;
  range.GetChains(chain1_begin, chain1_end, chain2_begin, chain2_end);

  std::vector<Point3<32>> chain1(chain1_begin, chain1_end);
  EXPECT_THAT(chain1, ElementsAre(input[3], input[0], input[1], input[2]));

  std::vector<Point3<32>> chain2(chain2_begin, chain2_end);
  EXPECT_THAT(chain2, ElementsAre(input[3], input[2]));
}

TEST(MonotoneRange, StopsAtNonMonotone) {
  //
  //        p0 -> p1 -> p2
  //                      \
  //      p12              p3
  //       |               |
  //      p11              p4
  //        \             /
  //        p10         p5
  //         |          |
  //        p9          p6
  //        /             \
  //       p8 <----------- p7
  //
  using Point3Iterator = std::vector<Point3<32>>::iterator;
  using MonotoneRange = MonotoneRange<Point3Iterator>;
  using ConcatRange = MonotoneRange::ConcatRangeRep;
  std::vector<Point3<32>> input{Point3<32>(1, 5, 0), // p0
                                 Point3<32>(2, 5, 0), // p1
                                 Point3<32>(3, 5, 0), // p2
                                 Point3<32>(4, 4, 0), // p3
                                 Point3<32>(4, 3, 0), // p4
                                 Point3<32>(3, 2, 0), // p5
                                 Point3<32>(3, 1, 0), // p6
                                 Point3<32>(4, 0, 0), // p7
                                 Point3<32>(0, 0, 0), // p8
                                 Point3<32>(1, 1, 0), // p9
                                 Point3<32>(1, 2, 0), // p10
                                 Point3<32>(0, 3, 0), // p11
                                 Point3<32>(0, 4, 0), // p12
  };

  Point3Iterator remaining_begin = input.begin();
  Point3Iterator remaining_end = input.end();
  MonotoneRange range;
  range.Build(/*monotone_dimension=*/0, remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, input.begin() + 6);
  EXPECT_EQ(remaining_end, input.begin() + 10);

  ConcatRange::const_iterator chain1_begin;
  ConcatRange::const_iterator chain1_end;
  ConcatRange::const_reverse_iterator chain2_begin;
  ConcatRange::const_reverse_iterator chain2_end;
  range.GetChains(chain1_begin, chain1_end, chain2_begin, chain2_end);

  std::vector<Point3<32>> chain1(chain1_begin, chain1_end);
  EXPECT_THAT(chain1, ElementsAre(input[11], input[12], input[0], input[1],
                                  input[2], input[3], input[4]));

  std::vector<Point3<32>> chain2(chain2_begin, chain2_end);
  EXPECT_THAT(chain2, ElementsAre(input[11], input[10], input[9], input[6],
                                  input[5], input[4]));
}

TEST(MonotoneRange, EmptyInput) {
  using Point3Iterator = std::vector<Point3<32>>::iterator;
  using MonotoneRange = MonotoneRange<Point3Iterator>;
  using ConcatRange = MonotoneRange::ConcatRangeRep;
  std::vector<Point3<32>> input{};

  Point3Iterator remaining_begin = input.begin();
  Point3Iterator remaining_end = input.end();
  MonotoneRange range;
  range.Build(/*monotone_dimension=*/0, remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, remaining_end);

  ConcatRange::const_iterator chain1_begin;
  ConcatRange::const_iterator chain1_end;
  ConcatRange::const_reverse_iterator chain2_begin;
  ConcatRange::const_reverse_iterator chain2_end;
  range.GetChains(chain1_begin, chain1_end, chain2_begin, chain2_end);

  EXPECT_EQ(chain1_begin, chain1_end);
  EXPECT_EQ(chain2_begin, chain2_end);
}

TEST(MonotoneRange, SinglePoint) {
  using Point3Iterator = std::vector<Point3<32>>::iterator;
  using MonotoneRange = MonotoneRange<Point3Iterator>;
  using ConcatRange = MonotoneRange::ConcatRangeRep;
  std::vector<Point3<32>> input{Point3<32>(1, 1, 0)};

  Point3Iterator remaining_begin = input.begin();
  Point3Iterator remaining_end = input.end();
  MonotoneRange range;
  range.Build(/*monotone_dimension=*/0, remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, remaining_end);

  ConcatRange::const_iterator chain1_begin;
  ConcatRange::const_iterator chain1_end;
  ConcatRange::const_reverse_iterator chain2_begin;
  ConcatRange::const_reverse_iterator chain2_end;
  range.GetChains(chain1_begin, chain1_end, chain2_begin, chain2_end);

  std::vector<Point3<32>> chain1(chain1_begin, chain1_end);
  EXPECT_THAT(chain1, ElementsAre(input[0]));

  std::vector<Point3<32>> chain2(chain2_begin, chain2_end);
  EXPECT_THAT(chain2, ElementsAre(input[0]));
}

TEST(MonotoneRange, TwoVertices) {
  using Point3Iterator = std::vector<Point3<32>>::iterator;
  using MonotoneRange = MonotoneRange<Point3Iterator>;
  using ConcatRange = MonotoneRange::ConcatRangeRep;
  std::vector<Point3<32>> input{Point3<32>(1, 1, 0), Point3<32>(2, 2, 0)};

  Point3Iterator remaining_begin = input.begin();
  Point3Iterator remaining_end = input.end();
  MonotoneRange range;
  range.Build(/*monotone_dimension=*/0, remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, remaining_end);

  ConcatRange::const_iterator chain1_begin;
  ConcatRange::const_iterator chain1_end;
  ConcatRange::const_reverse_iterator chain2_begin;
  ConcatRange::const_reverse_iterator chain2_end;
  range.GetChains(chain1_begin, chain1_end, chain2_begin, chain2_end);

  std::vector<Point3<32>> chain1(chain1_begin, chain1_end);
  EXPECT_THAT(chain1, ElementsAre(input[0], input[1]));

  std::vector<Point3<32>> chain2(chain2_begin, chain2_end);
  EXPECT_THAT(chain2, ElementsAre(input[0], input[1]));
}

TEST(MonotoneRange, AllCollinearInCompareDim) {
  using Point3Iterator = std::vector<Point3<32>>::iterator;
  using MonotoneRange = MonotoneRange<Point3Iterator>;
  using ConcatRange = MonotoneRange::ConcatRangeRep;
  std::vector<Point3<32>> input{Point3<32>(0, 1, 0),
                                 Point3<32>(0, 2, 0),
                                 Point3<32>(0, 3, 0),
                                 Point3<32>(0, 1, 0),
                                 Point3<32>(0, 2, 0),
                                 Point3<32>(0, 3, 0)};

  Point3Iterator remaining_begin = input.begin();
  Point3Iterator remaining_end = input.end();
  MonotoneRange range;
  range.Build(/*monotone_dimension=*/0, remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, remaining_end);

  ConcatRange::const_iterator chain1_begin;
  ConcatRange::const_iterator chain1_end;
  ConcatRange::const_reverse_iterator chain2_begin;
  ConcatRange::const_reverse_iterator chain2_end;
  range.GetChains(chain1_begin, chain1_end, chain2_begin, chain2_end);

  std::vector<Point3<32>> chain1(chain1_begin, chain1_end);
  EXPECT_THAT(chain1, ElementsAre(input[0], input[1], input[2], input[3],
                                  input[4], input[5]));

  std::vector<Point3<32>> chain2(chain2_begin, chain2_end);
  EXPECT_THAT(chain2, ElementsAre(input[0], input[5]));
}

TEST(MonotoneRange, Step4DoesntOvershoot) {
  //
  //        p0 -> p1 -> p2
  //                      \
  //      p9               p3
  //       \              /
  //     ___p8          p4
  //    /              /
  //  p7 <- p6 <- p5 <-
  //
  using Point3Iterator = std::vector<Point3<32>>::iterator;
  using MonotoneRange = MonotoneRange<Point3Iterator>;
  using ConcatRange = MonotoneRange::ConcatRangeRep;
  std::vector<Point3<32>> input{Point3<32>(2, 3, 0), // p0
                                 Point3<32>(3, 3, 0), // p1
                                 Point3<32>(4, 3, 0), // p2
                                 Point3<32>(5, 2, 0), // p3
                                 Point3<32>(4, 1, 0), // p4
                                 Point3<32>(3, 0, 0), // p5
                                 Point3<32>(2, 0, 0), // p6
                                 Point3<32>(0, 0, 0), // p7
                                 Point3<32>(2, 1, 0), // p8
                                 Point3<32>(1, 2, 0), // p9
  };

  Point3Iterator remaining_begin = input.begin();
  Point3Iterator remaining_end = input.end();
  MonotoneRange range;
  range.Build(/*monotone_dimension=*/0, remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, input.begin() + 6);
  EXPECT_EQ(remaining_end, input.begin() + 9);

  ConcatRange::const_iterator chain1_begin;
  ConcatRange::const_iterator chain1_end;
  ConcatRange::const_reverse_iterator chain2_begin;
  ConcatRange::const_reverse_iterator chain2_end;
  range.GetChains(chain1_begin, chain1_end, chain2_begin, chain2_end);

  std::vector<Point3<32>> chain1(chain1_begin, chain1_end);
  EXPECT_THAT(chain1, ElementsAre(input[9], input[0], input[1], input[2],
                                  input[3]));

  std::vector<Point3<32>> chain2(chain2_begin, chain2_end);
  EXPECT_THAT(chain2, ElementsAre(input[9], input[8], input[6], input[5],
                                  input[4], input[3]));
}

TEST(MonotoneRange, Step5DoesntOvershoot) {
  //
  //        p0 -> p1 -> p2
  //                      \
  //      p9               p3
  //       |              /
  //       \            p4 --------,
  //        \                       \
  //        p8 <------- p7 <- p6 <- p5
  //
  using Point3Iterator = std::vector<Point3<32>>::iterator;
  using MonotoneRange = MonotoneRange<Point3Iterator>;
  using ConcatRange = MonotoneRange::ConcatRangeRep;
  std::vector<Point3<32>> input{Point3<32>(2, 3, 0), // p0
                                 Point3<32>(3, 3, 0), // p1
                                 Point3<32>(4, 3, 0), // p2
                                 Point3<32>(5, 2, 0), // p3
                                 Point3<32>(4, 1, 0), // p4
                                 Point3<32>(7, 0, 0), // p5
                                 Point3<32>(6, 0, 0), // p6
                                 Point3<32>(4, 0, 0), // p7
                                 Point3<32>(2, 0, 0), // p8
                                 Point3<32>(1, 5, 0), // p9
  };

  Point3Iterator remaining_begin = input.begin();
  Point3Iterator remaining_end = input.end();
  MonotoneRange range;
  range.Build(/*monotone_dimension=*/0, remaining_begin, remaining_end);
  EXPECT_EQ(remaining_begin, input.begin() + 4);
  EXPECT_EQ(remaining_end, input.begin() + 8);

  ConcatRange::const_iterator chain1_begin;
  ConcatRange::const_iterator chain1_end;
  ConcatRange::const_reverse_iterator chain2_begin;
  ConcatRange::const_reverse_iterator chain2_end;
  range.GetChains(chain1_begin, chain1_end, chain2_begin, chain2_end);

  std::vector<Point3<32>> chain1(chain1_begin, chain1_end);
  EXPECT_THAT(chain1, ElementsAre(input[9], input[0], input[1], input[2],
                                  input[3]));

  std::vector<Point3<32>> chain2(chain2_begin, chain2_end);
  EXPECT_THAT(chain2, ElementsAre(input[9], input[8], input[7], input[4],
                                  input[3]));
}

}  // walnut
