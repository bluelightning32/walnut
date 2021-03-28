#include "walnut/big_int.h"

#include <cmath>

#include "gtest/gtest.h"

namespace walnut {

TEST(BigInt, Int32Construction) {
  int32_t a = (1 << 31) + 3;
  BigInt b(a);

  EXPECT_EQ(a, b.low_uint32());
  EXPECT_EQ(a, b.low_uint64());
}

TEST(BigInt, Int64Construction) {
  int64_t a = (static_cast<int64_t>(1) << 62) + 5;
  BigInt b(a);

  EXPECT_EQ(static_cast<uint32_t>(a), b.low_uint32());
  EXPECT_EQ(a, b.low_uint64());
}

TEST(BigInt, CopyConstructBig) {
  static constexpr int big_bits = 10*BigInt::bits_per_word*2;
  BigInt big_value = BigInt::max_value(big_bits - 1);
  BigInt copy(big_value);
}

TEST(BigInt, LeftShiftPos) {
  BigInt a(3);
  EXPECT_EQ(a << 1, BigInt{3 << 1});
  EXPECT_EQ(a << 61, BigInt{static_cast<int64_t>(3) << 61});

  BigInt b(3);
  EXPECT_LT(b << 63, b << 64);
  EXPECT_LE(b << 63, b << 64);
  EXPECT_LT(b << 64, b << 65);
  EXPECT_LE(b << 64, b << 65);
  EXPECT_GT(b << 64, b << 63);
  EXPECT_GE(b << 64, b << 63);
  EXPECT_GT(b << 65, b << 64);
  EXPECT_GE(b << 65, b << 64);
  EXPECT_EQ((b << 63).low_uint64(), static_cast<uint64_t>(1) << 63);

  BigInt c(static_cast<int64_t>(1) << 62);
  EXPECT_LT(c, c << 1);
  EXPECT_LE(c, c << 1);
  EXPECT_LT(c << 1, c << 2);
  EXPECT_LE(c << 1, c << 2);
  EXPECT_GT(c << 1, c);
  EXPECT_GE(c << 1, c);
  EXPECT_GT(c << 2, c << 1);
  EXPECT_GE(c << 2, c << 1);
  EXPECT_EQ((c << 1).low_uint64(), static_cast<uint64_t>(1) << 63);
  EXPECT_EQ((c << 2).low_uint64(), 0);
}

TEST(BigInt, LeftShiftNeg) {
  BigInt a(-3);
  EXPECT_EQ(a << 1, BigInt{static_cast<int64_t>(-3) *
                               static_cast<int64_t>(
                                 (static_cast<uint64_t>(1) << 1))});
  EXPECT_EQ(a << 61, BigInt{static_cast<int64_t>(-3) *
                                static_cast<int64_t>(
                                  (static_cast<uint64_t>(1) << 61))});

  BigInt b(-3);
  EXPECT_GT(b << 63, b << 64);
  EXPECT_GE(b << 63, b << 64);
  EXPECT_GT(b << 64, b << 65);
  EXPECT_GE(b << 64, b << 65);
  EXPECT_LT(b << 64, b << 63);
  EXPECT_LE(b << 64, b << 63);
  EXPECT_LT(b << 65, b << 64);
  EXPECT_LE(b << 65, b << 64);
  EXPECT_EQ((b << 63).low_uint64(), static_cast<uint64_t>(1) << 63);

  BigInt c(static_cast<int64_t>(-1) *
                static_cast<int64_t>(static_cast<uint64_t>(1) << 62));
  EXPECT_GT(c, c << 1);
  EXPECT_GE(c, c << 1);
  EXPECT_GT(c << 1, c << 2);
  EXPECT_GE(c << 1, c << 2);
  EXPECT_LT(c << 1, c);
  EXPECT_LE(c << 1, c);
  EXPECT_LT(c << 2, c << 1);
  EXPECT_LE(c << 2, c << 1);
  EXPECT_EQ((c << 1).low_uint64(), static_cast<uint64_t>(1) << 63);
  EXPECT_EQ((c << 2).low_uint64(), 0);
}

TEST(BigInt, LeftShift0) {
  BigInt b(-1);
  BigInt result = b << 0;
  EXPECT_EQ(result.low_uint64(), -1);
  EXPECT_LT(result, BigInt{0});
  EXPECT_GT(result, BigInt{-1} << 1);
}

TEST(BigInt, LeftShift64) {
  BigInt b(-1);
  BigInt result = b << 64;
  EXPECT_EQ(result.low_uint64(), 0);
  EXPECT_LT(result, BigInt{0});
  EXPECT_GT(result, BigInt{-1} << 65);
}

TEST(BigInt, AddInt32CarryPos) {
  BigInt a(static_cast<int32_t>(1) << 30);
  BigInt b(static_cast<int32_t>(1) << 30);
  BigInt result = a.Add(b);

  EXPECT_EQ(result.low_uint64(), static_cast<int64_t>(1) << 31);
  EXPECT_GT(result, a);
  EXPECT_LT(a, result);
}

TEST(BigInt, AddInt32CarryNeg) {
  BigInt a(std::numeric_limits<int32_t>::min());
  BigInt b(std::numeric_limits<int32_t>::min());
  BigInt result = a.Add(b);

  EXPECT_EQ(result.low_uint64(),
            static_cast<int64_t>(-1) *
            static_cast<int64_t>(static_cast<uint64_t>(1) << 32));
  EXPECT_LT(result, a);
  EXPECT_GT(a, result);
}

TEST(BigInt, AddInt32ToInt64) {
  BigInt a(static_cast<int32_t>(1) << 30);
  BigInt b(static_cast<int64_t>(1) << 62);
  BigInt result = a.Add(b);

  EXPECT_EQ(result.low_uint64(),
      static_cast<int64_t>(1) << 30 | static_cast<int64_t>(1) << 62);
  EXPECT_GT(result, a);
  EXPECT_LT(a, result);
}

TEST(BigInt, AddInt64Carry) {
  BigInt a(std::numeric_limits<int64_t>::min());
  BigInt b(std::numeric_limits<int64_t>::min());
  BigInt result = a.Add(b);

  EXPECT_EQ(result.low_uint64(), 0);
  EXPECT_EQ(result, BigInt(a) << 1);
  EXPECT_LT(result, a);
  EXPECT_GT(a, result);
}

TEST(BigInt, AddSubtractCarry3Words) {
  BigInt uint64_max = (BigInt{
      std::numeric_limits<int64_t>::max()} << 1) + BigInt{1};

  BigInt a;
  a = a.Add(uint64_max);
  a = a.Add(uint64_max << 64);
  a = a.Add(uint64_max << 128);
  BigInt result = a.Add(BigInt{1});

  EXPECT_EQ(result.low_uint64(), 0);
  EXPECT_EQ(result, BigInt{1} << 192);

  BigInt sub_result = result.Subtract(BigInt{1});
  EXPECT_EQ(sub_result, a);
}

TEST(BigInt, PlusEqual) {
  BigInt result(1);
  BigInt add(1);

  for (int i = 0; i < 120; ++i) {
    result += (add << i);
  }

  EXPECT_EQ(result, BigInt(1) << 120);
}

TEST(BigInt, PlusEqualCountDown) {
  BigInt result(1);
  BigInt add(1);

  for (int i = 119; i >= 0; --i) {
    result += (add << i);
  }

  EXPECT_EQ(result, BigInt(1) << 120);
}

TEST(BigInt, PlusEqualNeg) {
  BigInt result(-1);
  BigInt add(-1);

  for (int i = 0; i < 120; ++i) {
    result += (add << i);
  }

  EXPECT_EQ(result, BigInt(-1) << 120);
}

TEST(BigInt, SubtractEqual) {
  BigInt result(-1);
  BigInt subtract(1);

  for (int i = 0; i < 120; ++i) {
    result -= (subtract << i);
  }

  EXPECT_EQ(result, BigInt(-1) << 120);
}

TEST(BigInt, SubtractUInt64Carry) {
  BigInt a(static_cast<int64_t>(1) << 32);
  BigInt b(static_cast<int64_t>(1) << 31);
  BigInt result = a.Subtract(b);

  EXPECT_EQ(result.low_uint64(), static_cast<int64_t>(1) << 31);
  EXPECT_EQ(result, b);
  EXPECT_LT(result, a);
}

TEST(BigInt, SubtractInt32ToNeg) {
  BigInt a(static_cast<int32_t>(1));
  BigInt b(static_cast<int32_t>(2));
  BigInt result = a.Subtract(b);

  EXPECT_EQ(result.low_uint64(), static_cast<uint64_t>(-1));
  EXPECT_EQ(result.Add(b), a);
  EXPECT_EQ(result, BigInt{-1});
}

TEST(BigInt, SubtractInt64ToNeg) {
  BigInt a(static_cast<int64_t>(1) << 32);
  BigInt b(static_cast<int64_t>(2) << 32);
  BigInt result = a.Subtract(b);

  EXPECT_EQ(result.low_uint64(), static_cast<uint64_t>(-1) << 32);
  EXPECT_EQ(result.Add(b), a);
  EXPECT_EQ(result, BigInt{static_cast<int64_t>(-1) *
                               static_cast<int64_t>(
                                   static_cast<uint64_t>(1) << 32)});
}

TEST(BigInt, SubtractCarryFrom127) {
  BigInt a = BigInt{1} << 128;
  BigInt b = BigInt{1} << 127;
  BigInt result = a.Subtract(b);

  EXPECT_EQ(result, b);
  EXPECT_LT(result, a);
}

TEST(BigInt, Subtract192bNeg1Sub128bNeg1) {
  BigInt a(-1);
  BigInt b(-1);
  BigInt result = a.Subtract(b);

  BigInt expected{0};
  EXPECT_EQ(result, expected);
}

TEST(BigInt, SubtractAdjacentPosPow2) {
  for (int i = 1; i < 190; ++i) {
    BigInt a = BigInt{1} << i;
    BigInt b = BigInt{1} << (i - 1);
    BigInt result = a.Subtract(b);

    EXPECT_EQ(result, b);
  }
}

TEST(BigInt, SubtractAdjacentNegPow2) {
  for (int i = 1; i < 191; ++i) {
    BigInt a = BigInt{-1} << i;
    BigInt b = BigInt{-1} << (i - 1);
    BigInt result = a.Subtract(b);

    EXPECT_EQ(result, b);
  }
}

TEST(BigInt, MultiplyInt32Max) {
  BigInt a{std::numeric_limits<int32_t>::max()};
  BigInt result = a.Multiply(a);

  BigInt expected{
    static_cast<int64_t>(std::numeric_limits<int32_t>::max()) *
    static_cast<int64_t>(std::numeric_limits<int32_t>::max())};
  EXPECT_EQ(result, expected);
}

TEST(BigInt, MultiplyInt64Max) {
  BigInt a{std::numeric_limits<int64_t>::max()};
  BigInt result = a.Multiply(a);

  BigInt expected = (BigInt{1} << 126) -
    a - a - BigInt{1};
  EXPECT_EQ(result, expected);
}

TEST(BigInt, MultiplyInt32Min) {
  BigInt a{std::numeric_limits<int32_t>::min()};
  BigInt result = a.Multiply(a);

  BigInt expected{
    static_cast<int64_t>(std::numeric_limits<int32_t>::min()) *
    static_cast<int64_t>(std::numeric_limits<int32_t>::min())};
  EXPECT_EQ(result, expected);
}

TEST(BigInt, MultiplyInt64Min) {
  BigInt a{std::numeric_limits<int64_t>::min()};
  BigInt result = a.Multiply(a);

  BigInt expected = (BigInt{1} << 126);
  EXPECT_EQ(result, expected);
}

TEST(BigInt, Multiply1With2Pow63) {
  const BigInt a(1);
  const BigInt a_shifted = a << 0;
  const BigInt b_shifted = a << 63;
  const BigInt result = a_shifted * b_shifted;
  const BigInt expected = BigInt{1} << 63;
  EXPECT_EQ(result, expected);
}

TEST(BigInt, MultiplyQuadPower2PosPos) {
  const BigInt a(1);
  for (size_t i = 0; i < BigUIntWord::bits_per_word*4 - 1; i++) {
    const BigInt a_shifted = a << i;
    for (size_t j = 0; j < BigUIntWord::bits_per_word*4 - 1; j++) {
      const BigInt b_shifted = a << j;
      const BigInt result = a_shifted * b_shifted;
      const BigInt expected = (BigInt{1} << (i + j));
      EXPECT_EQ(result, expected);
    }
  }
}

TEST(BigInt, MultiplyQuadPower2PosNeg) {
  const BigInt a(1);
  const BigInt b(-1);
  for (size_t i = 0; i < BigUIntWord::bits_per_word*4 - 1; i++) {
    const BigInt a_shifted = a << i;
    for (size_t j = 0; j < BigUIntWord::bits_per_word*4 - 1; j++) {
      const BigInt b_shifted = b << j;
      const BigInt result = a_shifted * b_shifted;
      const BigInt expected = (BigInt{-1} << (i + j));
      EXPECT_EQ(result, expected);
    }
  }
}

TEST(BigInt, MultiplyQuadPower2NegNeg) {
  const BigInt a(-1);
  const BigInt b(-1);
  for (size_t i = 0; i < BigUIntWord::bits_per_word*4 - 1; i++) {
    const BigInt a_shifted = a << i;
    for (size_t j = 0; j < BigUIntWord::bits_per_word*4 - 1; j++) {
      const BigInt b_shifted = b << j;
      const BigInt result = a_shifted * b_shifted;
      const BigInt expected = (BigInt{1} << (i + j));
      EXPECT_EQ(result, expected);
    }
  }
}

TEST(BigInt, Negate0) {
  const BigInt a{0};
  const BigInt result = -a;
  EXPECT_EQ(result, a);
}

TEST(BigInt, NegateInt32Min) {
  const BigInt a{std::numeric_limits<int32_t>::min()};
  const BigInt result = -a;
  const BigInt expected = BigInt{1} << 31;
  EXPECT_EQ(result, expected);
}

TEST(BigInt, NegateInt64Min) {
  const BigInt a{std::numeric_limits<int64_t>::min()};
  const BigInt result = -a;
  const BigInt expected = BigInt{1} << 63;
  EXPECT_EQ(result, expected);
}

TEST(BigInt, NegateInt128Min) {
  const BigInt a = BigInt{std::numeric_limits<int64_t>::min()} << 64;
  const BigInt result = -a;
  const BigInt expected = BigInt{1} << 127;
  EXPECT_EQ(result, expected);
}

TEST(BigInt, NegateInt128MinAfterAssign) {
  BigInt a = BigInt{5} << 128;
  a = BigInt{std::numeric_limits<int64_t>::min()} << 64;
  a.Negate();
  const BigInt expected = BigInt{1} << 127;
  EXPECT_EQ(a, expected);
}

TEST(BigInt, NegateInt128MaxExtraRoom) {
  const BigInt a = BigInt::max_value(127);
  const BigInt result = -a;
  EXPECT_LT(result, 0);
  EXPECT_EQ(result - BigInt(1), BigInt::min_value(127));
}

TEST(BigInt, GetAbs64Int64Min) {
  const BigInt a{std::numeric_limits<int64_t>::min()};
  bool was_signed;
  const auto result = a.GetAbs(was_signed);
  const BigInt expected = BigInt{1} << 63;
  EXPECT_GT(result, 0);
  EXPECT_EQ(result, expected);
  EXPECT_TRUE(was_signed);
}

TEST(BigInt, DividePos1byPos1) {
  BigInt a(1);
  BigInt b(1);
  BigInt remainder;
  BigInt result = a.DivideRemainder(b, &remainder);

  EXPECT_EQ(result, BigInt{1});
  EXPECT_EQ(remainder, BigInt{0});
}

TEST(BigInt, DivideNeg3byPos2) {
  BigInt a(-3);
  BigInt b(2);
  BigInt remainder;
  BigInt result = a.DivideRemainder(b, &remainder);

  EXPECT_EQ(result, BigInt{-1});
  EXPECT_EQ(remainder, BigInt{-1});
}

TEST(BigInt, DividePos3byNeg2) {
  BigInt a(3);
  BigInt b(-2);
  BigInt remainder;
  BigInt result = a.DivideRemainder(b, &remainder);

  EXPECT_EQ(result, BigInt{-1});
  EXPECT_EQ(remainder, BigInt{1});
}

TEST(BigInt, DivideNeg3byNeg2) {
  BigInt a(-3);
  BigInt b(-2);
  BigInt remainder;
  BigInt result = a.DivideRemainder(b, &remainder);

  EXPECT_EQ(result, BigInt{1});
  EXPECT_EQ(remainder, BigInt{-1});
}

TEST(BigInt, DivideBig3And2Combinations) {
  int shift_values[] = {1, 31, 32, 63, 64, 127, 128};
  int mult[] = {1, -1};
  for (int shift : shift_values) {
    for (int a_mult : mult) {
      for (int b_mult : mult) {
        BigInt a = BigInt{3 * a_mult} << shift;
        BigInt b = BigInt{2 * b_mult} << shift;
        BigInt remainder;
        BigInt result = a.DivideRemainder(b, &remainder);

        EXPECT_EQ(result, BigInt{1 * a_mult * b_mult});
        EXPECT_EQ(remainder, BigInt{1 * a_mult} << shift);
      }
    }
  }
}

TEST(BigInt, DivideManyBitsByFewBits) {
  BigInt a = BigInt{1} << 64;
  BigInt b{2};
  auto result = a / b;
  EXPECT_EQ(result, BigInt{1} << 63);
}

TEST(BigInt, DivideTouchBothWords) {
  BigInt a = (BigInt{2} << 64) + BigInt{2};
  BigInt b{2};
  BigInt remainder;
  BigInt result = a.DivideRemainder(b, &remainder);

  EXPECT_EQ(result, (BigInt{1} << 64) + BigInt{1});
  EXPECT_EQ(remainder, BigInt{0});
}

TEST(BigInt, Divide33bitOverflow) {
  // The internal algorithm will try to divide the greatest 64 bits (which is
  // 2^64-1) by one greater than the divisor (which is 2^31 + 23169 + 1). The
  // remainder will be larger than 2^33. So the test verifies that the
  // algorithm is expecting that large of a remainder.
  BigInt a = (BigInt{1} << 128) - BigInt{1};
  BigInt divisor{(1l<<31) + 23169};
  BigInt remainder;
  BigInt result = a.DivideRemainder(divisor, &remainder);

  EXPECT_LT(remainder, divisor);
  EXPECT_EQ(remainder, a - result*divisor);
}

TEST(BigInt, GetSign) {
  constexpr int test_bits = 256;
  EXPECT_EQ(BigInt{0}.GetSign(), 0);
  for (int shift = 0; shift < test_bits - 1; ++shift) {
    EXPECT_GT((BigInt{1} << shift).GetSign(), 0) << shift;
  }
  for (int shift = 0; shift < test_bits; ++shift) {
    EXPECT_LT((BigInt{-1} << shift).GetSign(), 0) << shift;
  }
}

TEST(BigInt, PrintNeg) {
  std::ostringstream os;
  os << BigInt(-1);
  EXPECT_EQ(os.str(), "-1");

  os.str("");
  os << BigInt(-25);
  EXPECT_EQ(os.str(), "-25");
}

TEST(BigInt, PrintPos) {
  std::ostringstream os;
  os << BigInt(1);
  EXPECT_EQ(os.str(), "1");

  os.str("");
  os << BigInt(25);
  EXPECT_EQ(os.str(), "25");
}

TEST(BigInt, Print2WordLargePos) {
  BigInt a = (BigInt{2147483647} << 64) +
                  (BigInt{9223372033633550336} << 1) +
                  (BigInt{1});
  std::ostringstream os;
  os << a;
  EXPECT_EQ(os.str(), "39614081257132168790329524225");
}

TEST(BigInt, Print4WordLargePos) {
  BigInt a = (BigInt{1} << 192) -
                  BigInt{1};
  std::ostringstream os;
  os << a;
  EXPECT_EQ(os.str(),
    "6277101735386680763835789423207666416102355444464034512895");
}

TEST(BigInt, PrintRepeating2s) {
  BigInt a = BigInt{3577192789080335246} +
                  (BigInt{6023345402697246855} << 65) +
                  (BigInt{1} << 64);
  std::ostringstream os;
  os << a;
  EXPECT_EQ(os.str(), "222222222222222222222222222222222222222");
}

TEST(BigInt, PrintZero) {
  std::ostringstream os;
  os << BigInt(0);
  EXPECT_EQ(os.str(), "0");
}

TEST(BigInt, MinValue) {
  EXPECT_EQ(BigInt::min_value(97), BigInt{-1} << 97);
}

TEST(BigInt, MinValue65) {
  EXPECT_EQ(BigInt::min_value(64), BigInt{-1} << 64);
}

TEST(BigInt, MinValueInt256) {
  EXPECT_EQ(BigInt::min_value(255), BigInt{-1} << 255);
}

TEST(BigInt, MaxValue) {
  EXPECT_EQ(BigInt::max_value(97), (BigInt{1} << 97) - BigInt{1});
}

TEST(BigInt, MaxValue65) {
  EXPECT_EQ(BigInt::max_value(64), (BigInt{1} << 64) - BigInt{1});
}

template <int i>
constexpr int force_constexpr_int = i;

TEST(BigInt, CastDoubleInt64Min) {
  const BigInt a{std::numeric_limits<int64_t>::min()};
  EXPECT_EQ((double)a, double(std::numeric_limits<int64_t>::min()));
}

TEST(BigInt, CastDoubleInt64Max) {
  const BigInt a{std::numeric_limits<int64_t>::max()};
  EXPECT_EQ((double)a, double(std::numeric_limits<int64_t>::max()));
}

TEST(BigInt, CastDoubleInt64MaxShifted32) {
  const BigInt a(BigInt{std::numeric_limits<int64_t>::max()} << 32);
  EXPECT_EQ((double)a,
            double(std::numeric_limits<int64_t>::max()) * std::ldexp(1, 32));
}

TEST(BigInt, CastDoubleInt64MinShifted32) {
  const BigInt a(BigInt{std::numeric_limits<int64_t>::min()} << 32);
  EXPECT_EQ((double)a,
            double(std::numeric_limits<int64_t>::min()) * std::ldexp(1, 32));
}

TEST(BigInt, CastDoubleInt32MinShifted48) {
  const BigInt a(BigInt{std::numeric_limits<int32_t>::min()} << 48);
  EXPECT_EQ((double)a,
            double(std::numeric_limits<int32_t>::min()) * std::ldexp(1, 48));
}

TEST(BigInt, Compare0Against0) {
  const BigInt a(0);
  const BigInt b(0);
  EXPECT_EQ(a.Compare(b), 0);
}

TEST(BigInt, Compare0Against2Pow63) {
  const BigInt a(0);
  const BigInt b = BigInt{1} << 63;
  EXPECT_EQ(a.Compare(b), -1);
}

TEST(BigInt, Compare0AgainstInt128Max) {
  const BigInt a(0);
  const BigInt b = BigInt::max_value(127);
  EXPECT_EQ(a.Compare(b), -1);
}

TEST(BigInt, Compare2Pow63Against0) {
  const BigInt a = BigInt{1} << 63;
  const BigInt b(0);
  EXPECT_EQ(a.Compare(b), 1);
}

TEST(BigInt, CompareInt128MaxAgainst0) {
  const BigInt a = BigInt::max_value(127);
  const BigInt b(0);
  EXPECT_EQ(a.Compare(b), 1);
}

TEST(BigInt, Compare2Pow63Against2Pow64) {
  const BigInt a = BigInt{1} << 63;
  const BigInt b = BigInt{1} << 64;
  EXPECT_EQ(a.Compare(b), -1);
}

TEST(BigInt, CompareInt128MinAgainst2Pow64) {
  const BigInt a = BigInt::min_value(127);
  const BigInt b = BigInt{1} << 64;
  EXPECT_EQ(a.Compare(b), -1);
}

TEST(BigInt, Compare2Pow64Plus2Pow63Against2Pow64) {
  const BigInt a = (BigInt{1} << 64) + (BigInt{1} << 63);
  const BigInt b = BigInt{1} << 64;
  EXPECT_EQ(a.Compare(b), 1);
}

TEST(BigInt, CompareInt64MinAgainstInt64Max) {
  const BigInt a(std::numeric_limits<int64_t>::min());
  const BigInt b(std::numeric_limits<int64_t>::max());
  EXPECT_EQ(a.Compare(b), -1);
}

TEST(BigInt, Lt0Against0) {
  const BigInt a(0);
  const BigInt b(0);
  EXPECT_FALSE(a < b);
}

TEST(BigInt, LtEq0Against0) {
  const BigInt a(0);
  const BigInt b(0);
  EXPECT_TRUE(a <= b);
}

TEST(BigInt, Gt0Against0) {
  const BigInt a(0);
  const BigInt b(0);
  EXPECT_FALSE(a > b);
}

TEST(BigInt, GtEq0Against0) {
  const BigInt a(0);
  const BigInt b(0);
  EXPECT_TRUE(a >= b);
}

TEST(BigInt, LtFlippable0Against0) {
  const BigInt a(0);
  const BigInt b(0);
  EXPECT_FALSE(a.LessThan(/*flip=*/false, b));
  EXPECT_FALSE(a.LessThan(/*flip=*/true, b));
}

TEST(BigInt, LtFlippable0Against2Pow63) {
  const BigInt a(0);
  const BigInt b = BigInt{1} << 63;
  EXPECT_TRUE(a.LessThan(/*flip=*/false, b));
  EXPECT_TRUE(b.LessThan(/*flip=*/true, a));

  EXPECT_FALSE(a.LessThan(/*flip=*/true, b));
  EXPECT_FALSE(b.LessThan(/*flip=*/false, a));
}

TEST(BigInt, LtFlippableMinIntAgainstMaxInt) {
  const BigInt min_value = BigInt::min_value(127);
  const BigInt max_value = BigInt::max_value(127);
  EXPECT_TRUE(min_value.LessThan(/*flip=*/false, max_value));
  EXPECT_FALSE(min_value.LessThan(/*flip=*/true, max_value));

  EXPECT_FALSE(max_value.LessThan(/*flip=*/false, min_value));
  EXPECT_TRUE(max_value.LessThan(/*flip=*/true, min_value));
}

TEST(BigInt, LtFlippable2Pow63Against2Pow63) {
  const BigInt a = BigInt{1} << 63;
  EXPECT_FALSE(a.LessThan(/*flip=*/false, a));
  EXPECT_FALSE(a.LessThan(/*flip=*/true, a));
}

TEST(BigInt, LtFlippable2Pow63Against2Pow63Plus1) {
  const BigInt a = BigInt{1} << 63;
  const BigInt b = a + 1;
  EXPECT_TRUE(a.LessThan(/*flip=*/false, b));
  EXPECT_FALSE(a.LessThan(/*flip=*/true, b));

  EXPECT_FALSE(b.LessThan(/*flip=*/false, a));
  EXPECT_TRUE(b.LessThan(/*flip=*/true, a));
}

TEST(BigInt, LtFlippableSmallInts) {
  for (int i = -5; i < 5; ++i) {
    for (int j = -5; j < 5; ++j) {
      const BigInt a(i);
      const BigInt b(j);
      EXPECT_EQ(i < j, a.LessThan(/*flip=*/false, b));
      EXPECT_EQ(j < i, a.LessThan(/*flip=*/true, b));
    }
  }
}

TEST(BigInt, DecrementInt64Min) {
  BigInt a = BigInt::min_value(63);
  --a;

  EXPECT_LT(a, BigInt(0));
}

TEST(BigInt, GetGreatestCommonDivisorInt64Min) {
  BigInt a = BigInt::min_value(63);
  BigInt b = BigInt::min_value(63);
  EXPECT_EQ(a.GetGreatestCommonDivisor(b), a);
}

TEST(BigInt, GetGreatestCommonDivisorInt128Min) {
  BigInt a = BigInt::min_value(127);
  BigInt b = BigInt::min_value(127);
  EXPECT_EQ(a.GetGreatestCommonDivisor(b), a);
}

TEST(BigInt, GetGreatestCommonDivisorInt256Min) {
  BigInt a = BigInt::min_value(255);
  BigInt b = BigInt::min_value(255);
  EXPECT_EQ(a.GetGreatestCommonDivisor(b), a);
}

}  // walnut
