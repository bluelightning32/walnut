#include "walnut/big_uint_word.h"

#include "gtest/gtest.h"
#include "walnut/stop_watch.h"

namespace walnut {

template <typename T>
class BigUIntWordTest : public ::testing::Test {
};

TEST(BigUIntWordTest, SubtractUnderflowThroughCarry) {
  bool carry;
  BigUIntWord result = BigUIntWord(0).Subtract(BigUIntWord(0), true, &carry);
  EXPECT_EQ(result, BigUIntWord(-1));
  EXPECT_TRUE(carry);
}

TEST(BigUIntWordTest, SubtractCompleteWrapAround) {
  bool carry;
  BigUIntWord result = BigUIntWord(0).Subtract(BigUIntWord(-1), true, &carry);
  EXPECT_EQ(result, BigUIntWord(0));
  EXPECT_TRUE(carry);

  result = BigUIntWord(-1).Subtract(BigUIntWord(-1), true, &carry);
  EXPECT_EQ(result, BigUIntWord(-1));
  EXPECT_TRUE(carry);
}

TEST(BigUIntWordTest, AddOverflowThroughCarry) {
  bool carry;
  BigUIntWord result = BigUIntWord(-1).Add(BigUIntWord(0), true, &carry);
  EXPECT_EQ(result, BigUIntWord(0));
  EXPECT_TRUE(carry);
}

TEST(BigUIntWordTest, AddCompleteWrapAround) {
  bool carry;
  BigUIntWord result = BigUIntWord(0).Add(BigUIntWord(-1), true, &carry);
  EXPECT_EQ(result, BigUIntWord(0));
  EXPECT_TRUE(carry);

  result = BigUIntWord(-1).Add(BigUIntWord(-1), true, &carry);
  EXPECT_EQ(result, BigUIntWord(-1));
  EXPECT_TRUE(carry);
}

TEST(BigUIntWordTest, ShiftRightAllOnes) {
  BigUIntWord low{-1};
  BigUIntWord high{-1};
  for (unsigned i = 0; i < BigUIntWord::bits_per_word; ++i) {
    EXPECT_EQ(low.ShiftRight(high, i), low);
  }
}

TEST(BigUIntWordTest, ShiftRightLowAllOnes) {
  BigUIntWord low{-1};
  BigUIntWord high{0};
  for (unsigned i = 0; i < BigUIntWord::bits_per_word; ++i) {
    EXPECT_EQ(low.ShiftRight(high, i), low >> i);
  }
}

TEST(BigUIntWordTest, ShiftRightHighAllOnes) {
  BigUIntWord low{0};
  BigUIntWord high{-1};
  EXPECT_EQ(low.ShiftRight(high, 0), low);
  for (unsigned i = 1; i < BigUIntWord::bits_per_word; ++i) {
    EXPECT_EQ(low.ShiftRight(high, i), high << (BigUIntWord::bits_per_word - i));
  }
}

TEST(BigUIntWordTest, GetHighestSingleBitSet) {
  for (unsigned i = 0; i < BigUIntWord::bits_per_word; i++) {
    EXPECT_EQ((BigUIntWord{1} << i).GetHighestSetBit(), i + 1);
  }
}

TEST(BigUIntWordTest, GetHighestSetBit0) {
  EXPECT_EQ(BigUIntWord{0}.GetHighestSetBit(), 0);
}

TEST(BigUIntWordTest, GetHighestDoubleBitSet) {
  for (unsigned i = 0; i < BigUIntWord::bits_per_word; i++) {
    EXPECT_EQ((BigUIntWord{1} << i | BigUIntWord{1}).GetHighestSetBit(), i + 1);
  }
}

TEST(BigUIntWordTest, GetTrailingZeros) {
  for (unsigned i = 0; i < BigUIntWord::bits_per_word; i++) {
    EXPECT_EQ((BigUIntWord{1} << i).GetTrailingZeros(), i);
  }
}

}  // walnut
