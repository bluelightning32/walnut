#include "walnut/concat_range.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::ElementsAre;

struct NoMoveNoCopy {
  explicit NoMoveNoCopy(int v) : v(v) { }

  bool operator == (const NoMoveNoCopy& other) const {
    return v == other.v;
  }

  int v;
};

TEST(ConcatRange, NonconstToConstIterator) {
  std::array<NoMoveNoCopy, 3> input_container{NoMoveNoCopy(0),
                                              NoMoveNoCopy(1),
                                              NoMoveNoCopy(2)};

  using InputIterator = std::array<NoMoveNoCopy, 3>::iterator;
  ConcatRange<InputIterator> ranges;
  ranges.Append(input_container.begin(), input_container.end());

  ConcatRange<InputIterator>::iterator begin = ranges.begin();
  ConcatRange<InputIterator>::const_iterator cbegin(begin);
  cbegin = begin;

  EXPECT_EQ(cbegin, begin);

  ConcatRange<InputIterator>::iterator end = ranges.end();
  ConcatRange<InputIterator>::const_iterator cend(end);
  cend = end;

  EXPECT_EQ(cend, end);
  EXPECT_NE(cbegin, end);
}

TEST(ConcatRange, Append2Ranges) {
  std::array<NoMoveNoCopy, 6> input_container{NoMoveNoCopy(0),
                                              NoMoveNoCopy(1),
                                              NoMoveNoCopy(2),
                                              NoMoveNoCopy(3),
                                              NoMoveNoCopy(4),
                                              NoMoveNoCopy(5)};

  using InputIterator = std::array<NoMoveNoCopy, 6>::iterator;
  ConcatRange<InputIterator> ranges;
  ranges.Append(input_container.begin(), input_container.begin() + 1);
  ranges.Append(input_container.begin() + 2, input_container.end());

  EXPECT_THAT(ranges, ElementsAre(NoMoveNoCopy(0),
                                  NoMoveNoCopy(2),
                                  NoMoveNoCopy(3),
                                  NoMoveNoCopy(4),
                                  NoMoveNoCopy(5)));
}

TEST(ConcatRange, Prepend2Ranges) {
  std::array<NoMoveNoCopy, 6> input_container{NoMoveNoCopy(0),
                                              NoMoveNoCopy(1),
                                              NoMoveNoCopy(2),
                                              NoMoveNoCopy(3),
                                              NoMoveNoCopy(4),
                                              NoMoveNoCopy(5)};

  using InputIterator = std::array<NoMoveNoCopy, 6>::iterator;
  ConcatRange<InputIterator> ranges;
  ranges.Prepend(input_container.begin(), input_container.begin() + 2);
  ranges.Prepend(input_container.begin() + 3, input_container.end());

  EXPECT_THAT(ranges, ElementsAre(NoMoveNoCopy(3),
                                  NoMoveNoCopy(4),
                                  NoMoveNoCopy(5),
                                  NoMoveNoCopy(0),
                                  NoMoveNoCopy(1)));
}

TEST(ConcatRange, Reverse) {
  std::array<NoMoveNoCopy, 6> input_container{NoMoveNoCopy(0),
                                              NoMoveNoCopy(1),
                                              NoMoveNoCopy(2),
                                              NoMoveNoCopy(3),
                                              NoMoveNoCopy(4),
                                              NoMoveNoCopy(5)};

  using InputIterator = std::array<NoMoveNoCopy, 6>::iterator;
  ConcatRange<InputIterator> ranges;
  ranges.Append(input_container.begin(), input_container.begin() + 1);
  ranges.Append(input_container.begin() + 2, input_container.end());

  std::vector<int> result;
  for (auto it = ranges.rbegin(); it != ranges.rend(); ++it) {
    result.push_back(it->v);
  }

  EXPECT_THAT(result, ElementsAre(5, 4, 3, 2, 0));
}

TEST(ConcatRange, Decrement) {
  std::array<NoMoveNoCopy, 6> input_container{NoMoveNoCopy(0),
                                              NoMoveNoCopy(1),
                                              NoMoveNoCopy(2),
                                              NoMoveNoCopy(3),
                                              NoMoveNoCopy(4),
                                              NoMoveNoCopy(5)};

  using InputIterator = std::array<NoMoveNoCopy, 6>::iterator;
  ConcatRange<InputIterator> ranges;
  ranges.Append(input_container.begin(), input_container.begin() + 1);
  ranges.Append(input_container.begin() + 2, input_container.end());

  std::vector<int> result;
  for (auto it = ranges.end(); it != ranges.begin(); ) {
    --it;
    result.push_back(it->v);
  }

  EXPECT_THAT(result, ElementsAre(5, 4, 3, 2, 0));
}

}  // walnut
