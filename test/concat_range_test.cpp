#include "walnut/concat_range.h"

#include <deque>

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

TEST(ConcatRange, AppendEmpty) {
  std::deque<NoMoveNoCopy> input_container;

  using InputIterator = decltype(input_container)::iterator;
  ConcatRange<InputIterator> ranges;
  ranges.Append(input_container.begin(), input_container.begin());

  EXPECT_THAT(ranges, ElementsAre());
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

TEST(ConcatRange, PrependEmpty) {
  std::deque<NoMoveNoCopy> input_container;

  using InputIterator = decltype(input_container)::iterator;
  ConcatRange<InputIterator> ranges;
  ranges.Prepend(input_container.begin(), input_container.begin());

  EXPECT_THAT(ranges, ElementsAre());
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

TEST(ConcatRange, CastIteratorToConstIterator) {
  using InputIterator = std::deque<NoMoveNoCopy>::iterator;
  ConcatRange<InputIterator> ranges;

  // Copy initialization requires a non-explicit constructor.
  ConcatRange<InputIterator>::const_iterator cbegin1 = ranges.begin();
  EXPECT_EQ(cbegin1, ranges.cbegin());

  ConcatRange<InputIterator>::const_iterator cbegin2(ranges.begin());
  EXPECT_EQ(cbegin2, ranges.cbegin());

  ConcatRange<InputIterator>::const_iterator cbegin3;
  cbegin3 = ranges.cbegin();
  EXPECT_EQ(cbegin3, ranges.cbegin());

  EXPECT_EQ(ranges.begin(), ranges.cbegin());
}


TEST(ConcatRangeConcatRange, AppendRanges) {
  std::deque<NoMoveNoCopy> input_container;
  for (int i = 0; i < 3 * 4; ++i) {
    input_container.emplace_back(i);
  }

  using InputIterator1 = decltype(input_container)::iterator;
  ConcatRange<InputIterator1> ranges1;

  // Add every 2 out of 3 of the input container.
  for (int i = 0; i < 3 * 4; i += 3) {
    ranges1.Append(input_container.begin() + i,
                   input_container.begin() + i + 2);
  }

  EXPECT_THAT(ranges1, ElementsAre(NoMoveNoCopy(0),
                                   NoMoveNoCopy(1),
                                   NoMoveNoCopy(3),
                                   NoMoveNoCopy(4),
                                   NoMoveNoCopy(6),
                                   NoMoveNoCopy(7),
                                   NoMoveNoCopy(9),
                                   NoMoveNoCopy(10)));

  using InputIterator2 = decltype(ranges1)::iterator;
  ConcatRange<InputIterator2> ranges2;

  InputIterator2 pos = ranges1.begin();
  InputIterator2 start = pos;
  for (int i = 0; i < 3; ++i) {
    ++pos;
  }
  ranges2.Append(start, pos);

  for (int i = 0; i < 2; ++i) {
    ++pos;
  }
  ranges2.Append(pos, pos);

  start = pos;
  for (int i = 0; i < 3; ++i) {
    ++pos;
  }
  ranges2.Append(start, pos);

  EXPECT_THAT(ranges2, ElementsAre(NoMoveNoCopy(0),
                                   NoMoveNoCopy(1),
                                   NoMoveNoCopy(3),
                                   NoMoveNoCopy(7),
                                   NoMoveNoCopy(9),
                                   NoMoveNoCopy(10)));
}

TEST(ConcatRangeConcatRange, PrependRanges) {
  std::deque<NoMoveNoCopy> input_container;
  for (int i = 0; i < 3 * 4; ++i) {
    input_container.emplace_back(i);
  }

  using InputIterator1 = decltype(input_container)::iterator;
  ConcatRange<InputIterator1> ranges1;

  // Add every 2 out of 3 of the input container.
  for (int i = 0; i < 3 * 4; i += 3) {
    ranges1.Append(input_container.begin() + i,
                   input_container.begin() + i + 2);
  }

  EXPECT_THAT(ranges1, ElementsAre(NoMoveNoCopy(0),
                                   NoMoveNoCopy(1),
                                   NoMoveNoCopy(3),
                                   NoMoveNoCopy(4),
                                   NoMoveNoCopy(6),
                                   NoMoveNoCopy(7),
                                   NoMoveNoCopy(9),
                                   NoMoveNoCopy(10)));

  using InputIterator2 = decltype(ranges1)::iterator;
  ConcatRange<InputIterator2> ranges2;

  InputIterator2 pos = ranges1.end();
  InputIterator2 end = pos;
  for (int i = 0; i < 3; ++i) {
    --pos;
  }
  ranges2.Prepend(pos, end);

  for (int i = 0; i < 2; ++i) {
    --pos;
  }
  ranges2.Prepend(pos, pos);

  end = pos;
  for (int i = 0; i < 3; ++i) {
    --pos;
  }
  ranges2.Prepend(pos, end);

  EXPECT_THAT(ranges2, ElementsAre(NoMoveNoCopy(0),
                                   NoMoveNoCopy(1),
                                   NoMoveNoCopy(3),
                                   NoMoveNoCopy(7),
                                   NoMoveNoCopy(9),
                                   NoMoveNoCopy(10)));
}

}  // walnut
