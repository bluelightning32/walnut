#include "walnut/big_int.h"

#include <cmath>

#include "gtest/gtest.h"

namespace walnut {

TEST(BigInt, Int32Construction) {
  int32_t a = (1 << 31) + 3;
  BigInt<64> b(a);

  EXPECT_EQ(a, b.low_uint32());
  EXPECT_EQ(a, b.low_uint64());
}

TEST(BigInt, Int64Construction) {
  int64_t a = (static_cast<int64_t>(1) << 62) + 5;
  BigInt<64> b(a);

  EXPECT_EQ(static_cast<uint32_t>(a), b.low_uint32());
  EXPECT_EQ(a, b.low_uint64());
}

TEST(BigInt, ConstructorAssertsOnOverflow) {
  static constexpr int big_bits = BigInt<32>::word_count*BigInt<32>::bits_per_word*2;
  BigInt<big_bits> big_value = BigInt<big_bits>::max_value();
  ASSERT_DEBUG_DEATH(BigInt<32> constructed(big_value), "max_words");
}

TEST(BigInt, LeftShiftPos) {
  BigInt<64> a(3);
  EXPECT_EQ(a << 1, BigInt<64>{3 << 1});
  EXPECT_EQ(a << 61, BigInt<64>{static_cast<int64_t>(3) << 61});
  EXPECT_EQ(a << 62, BigInt<64>{static_cast<int64_t>(3) << 62});

  BigInt<128> b(3);
  EXPECT_LT(b << 63, b << 64);
  EXPECT_LE(b << 63, b << 64);
  EXPECT_LT(b << 64, b << 65);
  EXPECT_LE(b << 64, b << 65);
  EXPECT_GT(b << 64, b << 63);
  EXPECT_GE(b << 64, b << 63);
  EXPECT_GT(b << 65, b << 64);
  EXPECT_GE(b << 65, b << 64);
  EXPECT_EQ((b << 63).low_uint64(), static_cast<uint64_t>(1) << 63);

  BigInt<128> c(static_cast<int64_t>(1) << 62);
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
  BigInt<64> a(-3);
  EXPECT_EQ(a << 1, BigInt<64>{static_cast<int64_t>(-3) *
                               static_cast<int64_t>(
                                 (static_cast<uint64_t>(1) << 1))});
  EXPECT_EQ(a << 61, BigInt<64>{static_cast<int64_t>(-3) *
                                static_cast<int64_t>(
                                  (static_cast<uint64_t>(1) << 61))});
  EXPECT_EQ(a << 62, BigInt<64>{static_cast<int64_t>(1) *
                                static_cast<int64_t>(
                                  (static_cast<uint64_t>(1) << 62))});

  BigInt<128> b(-3);
  EXPECT_GT(b << 63, b << 64);
  EXPECT_GE(b << 63, b << 64);
  EXPECT_GT(b << 64, b << 65);
  EXPECT_GE(b << 64, b << 65);
  EXPECT_LT(b << 64, b << 63);
  EXPECT_LE(b << 64, b << 63);
  EXPECT_LT(b << 65, b << 64);
  EXPECT_LE(b << 65, b << 64);
  EXPECT_EQ((b << 63).low_uint64(), static_cast<uint64_t>(1) << 63);

  BigInt<128> c(static_cast<int64_t>(-1) *
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
  BigInt<128> b(-1);
  BigInt<128> result = b << 0;
  EXPECT_EQ(result.low_uint64(), -1);
  EXPECT_LT(result, BigInt<128>{0});
  EXPECT_GT(result, BigInt<128>{-1} << 1);
}

TEST(BigInt, LeftShift64) {
  BigInt<192> b(-1);
  BigInt<192> result = b << 64;
  EXPECT_EQ(result.low_uint64(), 0);
  EXPECT_LT(result, BigInt<128>{0});
  EXPECT_GT(result, BigInt<128>{-1} << 65);
}

TEST(BigInt, AddInt32CarryPos) {
  BigInt<64> a(static_cast<int32_t>(1) << 30);
  BigInt<64> b(static_cast<int32_t>(1) << 30);
  BigInt<64> result = a.Add(b);

  EXPECT_EQ(result.low_uint64(), static_cast<int64_t>(1) << 31);
  EXPECT_GT(result, a);
  EXPECT_LT(a, result);
}

TEST(BigInt, AddInt32CarryNeg) {
  BigInt<64> a(std::numeric_limits<int32_t>::min());
  BigInt<64> b(std::numeric_limits<int32_t>::min());
  BigInt<64> result = a.Add(b);

  EXPECT_EQ(result.low_uint64(),
            static_cast<int64_t>(-1) *
            static_cast<int64_t>(static_cast<uint64_t>(1) << 32));
  EXPECT_LT(result, a);
  EXPECT_GT(a, result);
}

TEST(BigInt, AddInt32ToInt64) {
  BigInt<64> a(static_cast<int32_t>(1) << 30);
  BigInt<64> b(static_cast<int64_t>(1) << 62);
  BigInt<64> result = a.Add<>(b);

  EXPECT_EQ(result.low_uint64(),
      static_cast<int64_t>(1) << 30 | static_cast<int64_t>(1) << 62);
  EXPECT_GT(result, a);
  EXPECT_LT(a, result);
}

TEST(BigInt, AddInt64Carry) {
  BigInt<64> a(std::numeric_limits<int64_t>::min());
  BigInt<64> b(std::numeric_limits<int64_t>::min());
  BigInt<65> result = a.Add<65>(b);

  EXPECT_EQ(result.low_uint64(), 0);
  EXPECT_EQ(result, BigInt<128>(a) << 1);
  EXPECT_LT(result, a);
  EXPECT_GT(a, result);
}

TEST(BigInt, AddSubtractCarry3Words) {
  BigInt<256> uint64_max = (BigInt<128>{
      std::numeric_limits<int64_t>::max()} << 1) + BigInt<128>{1};

  BigInt<256> a;
  a = a.Add<>(uint64_max);
  a = a.Add<>(uint64_max << 64);
  a = a.Add<>(uint64_max << 128);
  BigInt<256> result = a.Add<>(BigInt<64>{1});

  EXPECT_EQ(result.low_uint64(), 0);
  EXPECT_EQ(result, BigInt<256>{1} << 192);

  BigInt<256> sub_result = result.Subtract<>(BigInt<64>{1});
  EXPECT_EQ(sub_result, a);
}

TEST(BigInt, PlusEqual) {
  BigInt<128> result(1);
  BigInt<128> add(1);

  for (int i = 0; i < 120; ++i) {
    result += (add << i);
  }

  EXPECT_EQ(result, BigInt<128>(1) << 120);
}

TEST(BigInt, PlusEqualCountDown) {
  BigInt<128> result(1);
  BigInt<128> add(1);

  for (int i = 119; i >= 0; --i) {
    result += (add << i);
  }

  EXPECT_EQ(result, BigInt<128>(1) << 120);
}

TEST(BigInt, PlusEqualNeg) {
  BigInt<128> result(-1);
  BigInt<128> add(-1);

  for (int i = 0; i < 120; ++i) {
    result += (add << i);
  }

  EXPECT_EQ(result, BigInt<128>(-1) << 120);
}

TEST(BigInt, SubtractEqual) {
  BigInt<128> result(-1);
  BigInt<128> subtract(1);

  for (int i = 0; i < 120; ++i) {
    result -= (subtract << i);
  }

  EXPECT_EQ(result, BigInt<128>(-1) << 120);
}

TEST(BigInt, SubtractUInt64Carry) {
  BigInt<64> a(static_cast<int64_t>(1) << 32);
  BigInt<64> b(static_cast<int64_t>(1) << 31);
  BigInt<64> result = a.Subtract<>(b);

  EXPECT_EQ(result.low_uint64(), static_cast<int64_t>(1) << 31);
  EXPECT_EQ(result, b);
  EXPECT_LT(result, a);
}

TEST(BigInt, SubtractInt32ToNeg) {
  BigInt<128> a(static_cast<int32_t>(1));
  BigInt<64> b(static_cast<int32_t>(2));
  BigInt<128> result = a.Subtract<>(b);

  EXPECT_EQ(result.low_uint64(), static_cast<uint64_t>(-1));
  EXPECT_EQ(result.Add<>(b), a);
  EXPECT_EQ(result, BigInt<64>{-1});
}

TEST(BigInt, SubtractInt64ToNeg) {
  BigInt<128> a(static_cast<int64_t>(1) << 32);
  BigInt<64> b(static_cast<int64_t>(2) << 32);
  BigInt<128> result = a.Subtract<>(b);

  EXPECT_EQ(result.low_uint64(), static_cast<uint64_t>(-1) << 32);
  EXPECT_EQ(result.Add<>(b), a);
  EXPECT_EQ(result, BigInt<64>{static_cast<int64_t>(-1) *
                               static_cast<int64_t>(
                                   static_cast<uint64_t>(1) << 32)});
}

TEST(BigInt, SubtractCarryFrom127) {
  BigInt<192> a = BigInt<192>{1} << 128;
  BigInt<192> b = BigInt<192>{1} << 127;
  BigInt<192> result = a.Subtract<>(b);

  EXPECT_EQ(result, b);
  EXPECT_LT(result, a);
}

TEST(BigInt, Subtract192bNeg1Sub128bNeg1) {
  BigInt<192> a(-1);
  BigInt<128> b(-1);
  BigInt<192> result = a.Subtract<>(b);

  BigInt<192> expected{0};
  EXPECT_EQ(result, expected);
}

TEST(BigInt, SubtractAdjacentPosPow2) {
  for (int i = 1; i < 190; ++i) {
    BigInt<192> a = BigInt<192>{1} << i;
    BigInt<192> b = BigInt<192>{1} << (i - 1);
    BigInt<192> result = a.Subtract<>(b);

    EXPECT_EQ(result, b);
  }
}

TEST(BigInt, SubtractAdjacentNegPow2) {
  for (int i = 1; i < 191; ++i) {
    BigInt<192> a = BigInt<192>{-1} << i;
    BigInt<192> b = BigInt<192>{-1} << (i - 1);
    BigInt<192> result = a.Subtract<>(b);

    EXPECT_EQ(result, b);
  }
}

TEST(BigInt, MultiplyInt32Max) {
  BigInt<64> a{std::numeric_limits<int32_t>::max()};
  BigInt<128> result = a.Multiply<>(a);

  BigInt<128> expected{
    static_cast<int64_t>(std::numeric_limits<int32_t>::max()) *
    static_cast<int64_t>(std::numeric_limits<int32_t>::max())};
  EXPECT_EQ(result, expected);
}

TEST(BigInt, MultiplyInt64Max) {
  BigInt<64> a{std::numeric_limits<int64_t>::max()};
  BigInt<128> result = a.Multiply<>(a);

  BigInt<128> expected = (BigInt<128>{1} << 126) -
    a - a - BigInt<64>{1};
  EXPECT_EQ(result, expected);
}

TEST(BigInt, MultiplyInt32Min) {
  BigInt<64> a{std::numeric_limits<int32_t>::min()};
  BigInt<128> result = a.Multiply<>(a);

  BigInt<128> expected{
    static_cast<int64_t>(std::numeric_limits<int32_t>::min()) *
    static_cast<int64_t>(std::numeric_limits<int32_t>::min())};
  EXPECT_EQ(result, expected);
}

TEST(BigInt, MultiplyInt64Min) {
  BigInt<64> a{std::numeric_limits<int64_t>::min()};
  BigInt<128> result = a.Multiply<>(a);

  BigInt<128> expected = (BigInt<128>{1} << 126);
  EXPECT_EQ(result, expected);
}

TEST(BigInt, MultiplyQuadPower2PosPos) {
  const BigInt<256> a(1);
  for (size_t i = 0; i < BigUIntWord::bits_per_word*4 - 1; i++) {
    const BigInt<256> a_shifted = a << i;
    for (size_t j = 0; j < BigUIntWord::bits_per_word*4 - 1; j++) {
      const BigInt<256> b_shifted = a << j;
      const BigInt<512> result = a_shifted * b_shifted;
      const BigInt<512> expected = (BigInt<512>{1} << (i + j));
      EXPECT_EQ(result, expected);
    }
  }
}

TEST(BigInt, MultiplyQuadPower2PosNeg) {
  const BigInt<256> a(1);
  const BigInt<256> b(-1);
  for (size_t i = 0; i < BigUIntWord::bits_per_word*4 - 1; i++) {
    const BigInt<256> a_shifted = a << i;
    for (size_t j = 0; j < BigUIntWord::bits_per_word*4 - 1; j++) {
      const BigInt<256> b_shifted = b << j;
      const BigInt<512> result = a_shifted * b_shifted;
      const BigInt<512> expected = (BigInt<512>{-1} << (i + j));
      EXPECT_EQ(result, expected);
    }
  }
}

TEST(BigInt, MultiplyQuadPower2NegNeg) {
  const BigInt<256> a(-1);
  const BigInt<256> b(-1);
  for (size_t i = 0; i < BigUIntWord::bits_per_word*4 - 1; i++) {
    const BigInt<256> a_shifted = a << i;
    for (size_t j = 0; j < BigUIntWord::bits_per_word*4 - 1; j++) {
      const BigInt<256> b_shifted = b << j;
      const BigInt<512> result = a_shifted * b_shifted;
      const BigInt<512> expected = (BigInt<512>{1} << (i + j));
      EXPECT_EQ(result, expected);
    }
  }
}

TEST(BigInt, Negate0) {
  const BigInt<128> a{0};
  const BigInt<128> result = -a;
  EXPECT_EQ(result, a);
}

TEST(BigInt, NegateInt32Min) {
  const BigInt<64> a{std::numeric_limits<int32_t>::min()};
  const BigInt<64> result = -a;
  const BigInt<64> expected = BigInt<64>{1} << 31;
  EXPECT_EQ(result, expected);
}

TEST(BigInt, NegateInt64Min) {
  const BigInt<128> a{std::numeric_limits<int64_t>::min()};
  const BigInt<128> result = -a;
  const BigInt<128> expected = BigInt<128>{1} << 63;
  EXPECT_EQ(result, expected);
}

TEST(BigInt, NegateInt128Min) {
  const BigInt<192> a = BigInt<192>{std::numeric_limits<int64_t>::min()} << 64;
  const BigInt<192> result = -a;
  const BigInt<192> expected = BigInt<192>{1} << 127;
  EXPECT_EQ(result, expected);
}

TEST(BigInt, NegateInt128MinAfterAssign) {
  BigInt<192> a = BigInt<192>{5} << 128;
  a = BigInt<192>{std::numeric_limits<int64_t>::min()} << 64;
  a.Negate();
  const BigInt<192> expected = BigInt<192>{1} << 127;
  EXPECT_EQ(a, expected);
}

TEST(BigInt, NegateInt128MaxExtraRoom) {
  const BigInt<192> a = BigInt<128>::max_value();
  const BigInt<192> result = -a;
  EXPECT_LT(result, 0);
  EXPECT_EQ(result - BigInt<192>(1), BigInt<128>::min_value());
}

TEST(BigInt, NegateOverflowFalse) {
  BigInt<128> a{std::numeric_limits<int64_t>::min()};
  EXPECT_FALSE(a.Negate());
}

TEST(BigInt, NegateOverflowTrue) {
  BigInt<128> a = BigInt<128>::min_value();
  EXPECT_TRUE(a.Negate());
}

TEST(BigInt, GetAbs64Int64Min) {
  const BigInt<64> a{std::numeric_limits<int64_t>::min()};
  bool was_signed;
  const auto result = a.GetAbs(was_signed);
  const BigInt<65> expected = BigInt<65>{1} << 63;
  EXPECT_GT(result, 0);
  EXPECT_EQ(result, expected);
  EXPECT_TRUE(was_signed);
}

TEST(BigInt, DividePos1byPos1) {
  BigInt<64> a(1);
  BigInt<64> b(1);
  BigInt<64> remainder;
  BigInt<64> result = a.DivideRemainder(b, &remainder);

  EXPECT_EQ(result, BigInt<64>{1});
  EXPECT_EQ(remainder, BigInt<64>{0});
}

TEST(BigInt, DivideNeg3byPos2) {
  BigInt<64> a(-3);
  BigInt<64> b(2);
  BigInt<64> remainder;
  BigInt<64> result = a.DivideRemainder(b, &remainder);

  EXPECT_EQ(result, BigInt<64>{-1});
  EXPECT_EQ(remainder, BigInt<64>{-1});
}

TEST(BigInt, DividePos3byNeg2) {
  BigInt<64> a(3);
  BigInt<64> b(-2);
  BigInt<64> remainder;
  BigInt<64> result = a.DivideRemainder(b, &remainder);

  EXPECT_EQ(result, BigInt<64>{-1});
  EXPECT_EQ(remainder, BigInt<64>{1});
}

TEST(BigInt, DivideNeg3byNeg2) {
  BigInt<64> a(-3);
  BigInt<64> b(-2);
  BigInt<64> remainder;
  BigInt<64> result = a.DivideRemainder(b, &remainder);

  EXPECT_EQ(result, BigInt<64>{1});
  EXPECT_EQ(remainder, BigInt<64>{-1});
}

TEST(BigInt, DivideBig3And2Combinations) {
  int shift_values[] = {1, 31, 32, 63, 64, 127, 128};
  int mult[] = {1, -1};
  for (int shift : shift_values) {
    for (int a_mult : mult) {
      for (int b_mult : mult) {
        BigInt<192> a = BigInt<192>{3 * a_mult} << shift;
        BigInt<192> b = BigInt<192>{2 * b_mult} << shift;
        BigInt<192> remainder;
        BigInt<192> result = a.DivideRemainder(b, &remainder);

        EXPECT_EQ(result, BigInt<192>{1 * a_mult * b_mult});
        EXPECT_EQ(remainder, BigInt<192>{1 * a_mult} << shift);
      }
    }
  }
}

TEST(BigInt, DivideManyBitsByFewBits) {
  BigInt<128> a = BigInt<128>{1} << 64;
  BigInt<8> b{2};
  auto result = a / b;
  EXPECT_EQ(result, BigInt<128>{1} << 63);
}

TEST(BigInt, DivideTouchBothWords) {
  BigInt<128> a = (BigInt<128>{2} << 64) + BigInt<128>{2};
  BigInt<128> b{2};
  BigInt<128> remainder;
  BigInt<128> result = a.DivideRemainder(b, &remainder);

  EXPECT_EQ(result, (BigInt<128>{1} << 64) + BigInt<128>{1});
  EXPECT_EQ(remainder, BigInt<64>{0});
}

TEST(BigInt, Divide33bitOverflow) {
  // The internal algorithm will try to divide the greatest 64 bits (which is
  // 2^64-1) by one greater than the divisor (which is 2^31 + 23169 + 1). The
  // remainder will be larger than 2^33. So the test verifies that the
  // algorithm is expecting that large of a remainder.
  BigInt<196> a = (BigInt<196>{1} << 128) - BigInt<64>{1};
  BigInt<64> divisor{(1l<<31) + 23169};
  BigInt<64> remainder;
  BigInt<196> result = a.DivideRemainder(divisor, &remainder);

  EXPECT_LT(remainder, divisor);
  EXPECT_EQ(remainder, a - result*divisor);
}

TEST(BigInt, GetSign) {
  constexpr int test_bits = 256;
  EXPECT_EQ(BigInt<test_bits>{0}.GetSign(), 0);
  for (int shift = 0; shift < test_bits - 1; ++shift) {
    EXPECT_GT((BigInt<test_bits>{1} << shift).GetSign(), 0) << shift;
  }
  for (int shift = 0; shift < test_bits; ++shift) {
    EXPECT_LT((BigInt<test_bits>{-1} << shift).GetSign(), 0) << shift;
  }
}

TEST(BigInt, PrintNeg) {
  std::ostringstream os;
  os << BigInt<32>(-1);
  EXPECT_EQ(os.str(), "-1");

  os.str("");
  os << BigInt<32>(-25);
  EXPECT_EQ(os.str(), "-25");
}

TEST(BigInt, PrintPos) {
  std::ostringstream os;
  os << BigInt<32>(1);
  EXPECT_EQ(os.str(), "1");

  os.str("");
  os << BigInt<32>(25);
  EXPECT_EQ(os.str(), "25");
}

TEST(BigInt, Print2WordLargePos) {
  BigInt<128> a = (BigInt<128>{2147483647} << 64) +
                  (BigInt<128>{9223372033633550336} << 1) +
                  (BigInt<128>{1});
  std::ostringstream os;
  os << a;
  EXPECT_EQ(os.str(), "39614081257132168790329524225");
}

TEST(BigInt, Print4WordLargePos) {
  BigInt<256> a = (BigInt<256>{1} << 192) -
                  BigInt<256>{1};
  std::ostringstream os;
  os << a;
  EXPECT_EQ(os.str(),
    "6277101735386680763835789423207666416102355444464034512895");
}

TEST(BigInt, PrintRepeating2s) {
  BigInt<512> a = BigInt<512>{3577192789080335246} +
                  (BigInt<512>{6023345402697246855} << 65) +
                  (BigInt<512>{1} << 64);
  std::ostringstream os;
  os << a;
  EXPECT_EQ(os.str(), "222222222222222222222222222222222222222");
}

TEST(BigInt, PrintZero) {
  std::ostringstream os;
  os << BigInt<32>(0);
  EXPECT_EQ(os.str(), "0");
}

TEST(BigInt, MinValue) {
  EXPECT_EQ(BigInt<98>::min_value(), BigInt<128>{-1} << 97);
}

TEST(BigInt, MaxValue) {
  EXPECT_EQ(BigInt<98>::max_value(), (BigInt<128>{1} << 97) - BigInt<128>{1});
}

template <int i>
constexpr int force_constexpr_int = i;

TEST(BigInt, ConstexprConstructor) {
  constexpr BigInt<32> a(1);
  constexpr BigInt<256> b(a);
  constexpr BigInt<32> c(b);

  EXPECT_EQ(1, force_constexpr_int<a.low_uint32()>);
  EXPECT_EQ(1, force_constexpr_int<b.low_uint32()>);
  EXPECT_EQ(1, force_constexpr_int<c.low_uint32()>);
}

TEST(BigInt, ConstexprMaxValue) {
  EXPECT_EQ(force_constexpr_int<BigInt<32>::max_value().low_uint32()> & 0xFF,
            0xFF);
}

TEST(BigInt, ConstexprMinValue) {
  constexpr BigInt<32> min_value = BigInt<32>::min_value();
  constexpr int min_value_int = static_cast<int>(min_value.low_uint32());
  EXPECT_EQ(force_constexpr_int<min_value_int> & 0x00, 0x00);
}

TEST(BigInt, CastDoubleInt64Min) {
  const BigInt<128> a{std::numeric_limits<int64_t>::min()};
  EXPECT_EQ((double)a, double(std::numeric_limits<int64_t>::min()));
}

TEST(BigInt, CastDoubleInt64Max) {
  const BigInt<128> a{std::numeric_limits<int64_t>::max()};
  EXPECT_EQ((double)a, double(std::numeric_limits<int64_t>::max()));
}

TEST(BigInt, CastDoubleInt64MaxShifted32) {
  const BigInt<128> a(BigInt<128>{std::numeric_limits<int64_t>::max()} << 32);
  EXPECT_EQ((double)a,
            double(std::numeric_limits<int64_t>::max()) * std::ldexp(1, 32));
}

TEST(BigInt, CastDoubleInt64MinShifted32) {
  const BigInt<128> a(BigInt<128>{std::numeric_limits<int64_t>::min()} << 32);
  EXPECT_EQ((double)a,
            double(std::numeric_limits<int64_t>::min()) * std::ldexp(1, 32));
}

TEST(BigInt, CastDoubleInt32MinShifted48) {
  const BigInt<128> a(BigInt<128>{std::numeric_limits<int32_t>::min()} << 48);
  EXPECT_EQ((double)a,
            double(std::numeric_limits<int32_t>::min()) * std::ldexp(1, 48));
}

TEST(BigInt, Compare0Against0) {
  const BigInt<128> a(0);
  const BigInt<128> b(0);
  EXPECT_EQ(a.Compare(b), 0);
}

TEST(BigInt, Compare0Against2Pow63) {
  const BigInt<128> a(0);
  const BigInt<128> b = BigInt<128>{1} << 63;
  EXPECT_EQ(a.Compare(b), -1);
}

TEST(BigInt, Compare0AgainstInt128Max) {
  const BigInt<128> a(0);
  const BigInt<128> b = BigInt<128>::max_value();
  EXPECT_EQ(a.Compare(b), -1);
}

TEST(BigInt, Compare2Pow63Against0) {
  const BigInt<128> a = BigInt<128>{1} << 63;
  const BigInt<128> b(0);
  EXPECT_EQ(a.Compare(b), 1);
}

TEST(BigInt, CompareInt128MaxAgainst0) {
  const BigInt<128> a = BigInt<128>::max_value();
  const BigInt<128> b(0);
  EXPECT_EQ(a.Compare(b), 1);
}

TEST(BigInt, Compare2Pow63Against2Pow64) {
  const BigInt<128> a = BigInt<128>{1} << 63;
  const BigInt<128> b = BigInt<128>{1} << 64;
  EXPECT_EQ(a.Compare(b), -1);
}

TEST(BigInt, CompareInt128MinAgainst2Pow64) {
  const BigInt<128> a = BigInt<128>::min_value();
  const BigInt<128> b = BigInt<128>{1} << 64;
  EXPECT_EQ(a.Compare(b), -1);
}

TEST(BigInt, Compare2Pow64Plus2Pow63Against2Pow64) {
  const BigInt<128> a = (BigInt<128>{1} << 64) + (BigInt<128>{1} << 63);
  const BigInt<128> b = BigInt<128>{1} << 64;
  EXPECT_EQ(a.Compare(b), 1);
}

TEST(BigInt, CompareInt64MinAgainstInt64Max) {
  const BigInt<128> a(std::numeric_limits<int64_t>::min());
  const BigInt<128> b(std::numeric_limits<int64_t>::max());
  EXPECT_EQ(a.Compare(b), -1);
}

TEST(BigInt, Lt0Against0) {
  const BigInt<128> a(0);
  const BigInt<128> b(0);
  EXPECT_FALSE(a < b);
}

TEST(BigInt, LtEq0Against0) {
  const BigInt<128> a(0);
  const BigInt<128> b(0);
  EXPECT_TRUE(a <= b);
}

TEST(BigInt, Gt0Against0) {
  const BigInt<128> a(0);
  const BigInt<128> b(0);
  EXPECT_FALSE(a > b);
}

TEST(BigInt, GtEq0Against0) {
  const BigInt<128> a(0);
  const BigInt<128> b(0);
  EXPECT_TRUE(a >= b);
}

TEST(BigInt, LtFlippable0Against0) {
  const BigInt<128> a(0);
  const BigInt<128> b(0);
  EXPECT_FALSE(a.LessThan(/*flip=*/false, b));
  EXPECT_FALSE(a.LessThan(/*flip=*/true, b));
}

TEST(BigInt, LtFlippable0Against2Pow63) {
  const BigInt<128> a(0);
  const BigInt<128> b = BigInt<128>{1} << 63;
  EXPECT_TRUE(a.LessThan(/*flip=*/false, b));
  EXPECT_TRUE(b.LessThan(/*flip=*/true, a));

  EXPECT_FALSE(a.LessThan(/*flip=*/true, b));
  EXPECT_FALSE(b.LessThan(/*flip=*/false, a));
}

TEST(BigInt, LtFlippableMinIntAgainstMaxInt) {
  const BigInt<128> min_value = BigInt<128>::min_value();
  const BigInt<128> max_value = BigInt<128>::max_value();
  EXPECT_TRUE(min_value.LessThan(/*flip=*/false, max_value));
  EXPECT_FALSE(min_value.LessThan(/*flip=*/true, max_value));

  EXPECT_FALSE(max_value.LessThan(/*flip=*/false, min_value));
  EXPECT_TRUE(max_value.LessThan(/*flip=*/true, min_value));
}

TEST(BigInt, LtFlippable2Pow63Against2Pow63) {
  const BigInt<128> a = BigInt<128>{1} << 63;
  EXPECT_FALSE(a.LessThan(/*flip=*/false, a));
  EXPECT_FALSE(a.LessThan(/*flip=*/true, a));
}

TEST(BigInt, LtFlippable2Pow63Against2Pow63Plus1) {
  const BigInt<128> a = BigInt<128>{1} << 63;
  const BigInt<128> b = a + 1;
  EXPECT_TRUE(a.LessThan(/*flip=*/false, b));
  EXPECT_FALSE(a.LessThan(/*flip=*/true, b));

  EXPECT_FALSE(b.LessThan(/*flip=*/false, a));
  EXPECT_TRUE(b.LessThan(/*flip=*/true, a));
}

TEST(BigInt, LtFlippableSmallInts) {
  for (int i = -5; i < 5; ++i) {
    for (int j = -5; j < 5; ++j) {
      const BigInt<128> a(i);
      const BigInt<128> b(j);
      EXPECT_EQ(i < j, a.LessThan(/*flip=*/false, b));
      EXPECT_EQ(j < i, a.LessThan(/*flip=*/true, b));
    }
  }
}

TEST(BigInt, DecrementInt64Min) {
  BigInt<128> a = BigInt<64>::min_value();
  --a;

  EXPECT_LT(a, BigInt<128>(0));
}

TEST(BigInt, GetGreatestCommonDivisorInt64Min) {
  BigInt<64> a = BigInt<64>::min_value();
  BigInt<64> b = BigInt<64>::min_value();
  EXPECT_EQ(a.GetGreatestCommonDivisor(b), a);
}

TEST(BigInt, GetGreatestCommonDivisorInt128Min) {
  BigInt<128> a = BigInt<128>::min_value();
  BigInt<128> b = BigInt<128>::min_value();
  EXPECT_EQ(a.GetGreatestCommonDivisor(b), a);
}

TEST(BigInt, GetGreatestCommonDivisorInt256Min) {
  BigInt<256> a = BigInt<256>::min_value();
  BigInt<256> b = BigInt<256>::min_value();
  EXPECT_EQ(a.GetGreatestCommonDivisor(b), a);
}

}  // walnut
