#include "walnut/big_int.h"

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
  static constexpr int big_bits = BigInt<32>::max_bits*2;
  BigInt<big_bits> big_value = BigInt<big_bits>::max_value();
  ASSERT_DEBUG_DEATH(BigInt<32> constructed(big_value), "overflow");
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
  EXPECT_EQ(a << 1, BigInt<64>{static_cast<int64_t>(-3) << 1});
  EXPECT_EQ(a << 61, BigInt<64>{static_cast<int64_t>(-3) << 61});
  EXPECT_EQ(a << 62, BigInt<64>{static_cast<int64_t>(1) << 62});

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

  BigInt<128> c(static_cast<int64_t>(-1) << 62);
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

  EXPECT_EQ(result.low_uint64(), static_cast<int64_t>(-1) << 32);
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
  BigInt<128> result = a.Add<2>(b);

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
  EXPECT_EQ(result, BigInt<64>{static_cast<int64_t>(-1) << 32});
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
  for (int i = 0; i < BigUIntWord::bits_per_word*4 - 1; i++) {
    const BigInt<256> a_shifted = a << i;
    for (int j = 0; j < BigUIntWord::bits_per_word*4 - 1; j++) {
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
  for (int i = 0; i < BigUIntWord::bits_per_word*4 - 1; i++) {
    const BigInt<256> a_shifted = a << i;
    for (int j = 0; j < BigUIntWord::bits_per_word*4 - 1; j++) {
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
  for (int i = 0; i < BigUIntWord::bits_per_word*4 - 1; i++) {
    const BigInt<256> a_shifted = a << i;
    for (int j = 0; j < BigUIntWord::bits_per_word*4 - 1; j++) {
      const BigInt<256> b_shifted = b << j;
      const BigInt<512> result = a_shifted * b_shifted;
      const BigInt<512> expected = (BigInt<512>{1} << (i + j));
      EXPECT_EQ(result, expected);
    }
  }
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

TEST(BigInt, GetUIntAbsInt32Min) {
  const BigInt<64> a{std::numeric_limits<int32_t>::min()};
  bool was_signed;
  const BigUInt<64> result = a.GetUIntAbs(&was_signed);
  const BigUInt<64> expected = BigUInt<64>{1} << 31;
  EXPECT_EQ(result, expected);
  EXPECT_TRUE(was_signed);
}

TEST(BigInt, GetUIntAbsInt64Min) {
  const BigInt<128> a{std::numeric_limits<int64_t>::min()};
  bool was_signed;
  const BigUInt<128> result = a.GetUIntAbs(&was_signed);
  const BigUInt<128> expected = BigUInt<128>{1} << 63;
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

}  // walnut
