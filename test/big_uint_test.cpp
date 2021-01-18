#include "walnut/big_uint.h"

#include "gtest/gtest.h"
#include "walnut/stop_watch.h"

#include <iostream>

namespace walnut {

TEST(BigUIntAcceleration, UInt32VsUInt64Multiply) {
  StopWatch uint32_time;
  StopWatch uint64_time;
  for (int attempt = 0, runs = 1000000; attempt < 10; attempt++, runs += runs/4) {
    {
      uint32_time.Start();
      BigUInt<64> product{static_cast<uint32_t>(1)};
      for (int i = 0; i < runs; i++) {
        product.AssignIgnoreOverflow(product.Multiply<1>(product));
      }
      uint32_time.Stop();
      std::cout << "uint32 time: " << uint32_time << " product: " << product.low_uint64() << std::endl;
    }

    {
      uint64_time.Start();
      BigUInt<64> product{-1};
      for (int i = 0; i < runs; i++) {
        product.AssignIgnoreOverflow(product.Multiply<1>(product));
      }
      uint64_time.Stop();
      std::cout << "uint64 time: " << uint64_time << " product: " << product.low_uint64() << std::endl;
    }
    if (uint32_time.diff() <= uint64_time.diff() * 1.1)
      break;
  }
  EXPECT_LE(uint32_time.diff(), uint64_time.diff() * 1.1);
}

TEST(BigUInt, UInt32Construction) {
  uint32_t a = (1 << 31) + 3;
  BigUInt<64> b(a);

  EXPECT_EQ(a, b.low_uint32());
  EXPECT_EQ(a, b.low_uint64());
}

TEST(BigUInt, UInt64Construction) {
  uint64_t a = (static_cast<uint64_t>(1) << 63) + 5;
  BigUInt<64> b(a);

  EXPECT_EQ(static_cast<uint32_t>(a), b.low_uint32());
  EXPECT_EQ(a, b.low_uint64());
}

TEST(BigUInt, max_value128) {
  BigUInt<128> max_value = BigUInt<128>::max_value();
  for (int i = 0; i < 128; i++) {
    EXPECT_EQ(max_value & (BigUInt<128>{1} << i), BigUInt<128>{1} << i);
  }
}

TEST(BigUInt, ConstructorAssertsOnOverflow) {
  static constexpr int big_bits = BigUInt<32>::max_bits*2;
  BigUInt<big_bits> big_value = BigUInt<big_bits>::max_value();
  ASSERT_DEBUG_DEATH(BigUInt<32> constructed(big_value), "max_bytes");
}

TEST(BigUInt, LeftShift) {
  BigUInt<64> a(static_cast<uint32_t>(3));
  EXPECT_EQ(a << 1, BigUInt<64>{static_cast<uint64_t>(3) << 1});
  EXPECT_EQ(a << 62, BigUInt<64>{static_cast<uint64_t>(3) << 62});

  BigUInt<128> b(static_cast<uint32_t>(3));
  EXPECT_LT(b << 63, b << 64);
  EXPECT_LT(b << 64, b << 65);
  EXPECT_EQ((b << 63).low_uint64(), static_cast<uint64_t>(1) << 63);

  BigUInt<128> c(static_cast<uint64_t>(1) << 63);
  EXPECT_LT(c, c << 1);
  EXPECT_LT(c << 1, c << 2);
  EXPECT_EQ((c << 1).low_uint64(), 0);
}

TEST(BigUInt, LeftShift0) {
  BigUInt<128> b(static_cast<uint64_t>(-1));
  BigUInt<128> result = b << 0;
  EXPECT_EQ(result.low_uint64(), -1);
  EXPECT_LT(result, BigUInt<128>{1} << 64);
}

TEST(BigUInt, LeftShift64) {
  BigUInt<192> b(static_cast<uint64_t>(-1));
  BigUInt<192> result = b << 64;
  EXPECT_EQ(result.low_uint64(), 0);
  EXPECT_LT(result, BigUInt<192>{1} << 128);
}

TEST(BigUInt, AddUInt32Carry) {
  BigUInt<64> a(static_cast<uint32_t>(1) << 31);
  BigUInt<64> b(static_cast<uint32_t>(1) << 31);
  BigUInt<64> result = a.Add<>(b);

  EXPECT_EQ(result.low_uint64(), static_cast<uint64_t>(1) << 32);
  EXPECT_GT(result, a);
  EXPECT_LT(a, result);
}

TEST(BigUInt, AddUInt32ToUInt64) {
  BigUInt<64> a(static_cast<uint32_t>(1) << 31);
  BigUInt<64> b(static_cast<uint64_t>(1) << 63);
  BigUInt<64> result = a.Add<>(b);

  EXPECT_EQ(result.low_uint64(),
      static_cast<uint64_t>(1) << 31 | static_cast<uint64_t>(1) << 63);
  EXPECT_GT(result, a);
  EXPECT_LT(a, result);
}

TEST(BigUInt, AddUInt64Carry) {
  BigUInt<64> a(static_cast<uint64_t>(1) << 63);
  BigUInt<64> b(static_cast<uint64_t>(1) << 63);
  BigUInt<128> result = a.Add<2>(b);

  EXPECT_EQ(result.low_uint64(), 0);
  EXPECT_EQ(result, BigUInt<128>(static_cast<uint64_t>(1) << 63) << 1);
  EXPECT_GT(result, a);
  EXPECT_LT(a, result);
}

TEST(BigUInt, AddSubtractCarry3Words) {
  BigUInt<256> a;
  a = a.Add<>(BigUInt<64>{static_cast<uint64_t>(-1)});
  a = a.Add<>(BigUInt<128>{static_cast<uint64_t>(-1)} << 64);
  a = a.Add<>(BigUInt<192>{static_cast<uint64_t>(-1)} << 128);
  BigUInt<256> result = a.Add<>(BigUInt<64>{static_cast<uint64_t>(1)});

  EXPECT_EQ(result.low_uint64(), 0);
  EXPECT_EQ(result, BigUInt<256>{static_cast<uint64_t>(1)} << 192);

  BigUInt<256> sub_result = result.Subtract<>(BigUInt<64>{static_cast<uint64_t>(1)});
  EXPECT_EQ(sub_result, a);
}

TEST(BigUInt, AddLeftShiftedAll1sTo0SingleWord) {
  for (size_t i = 0; i < BigUIntWord::bits_per_word; ++i) {
    BigUInt<64> result(0);
    result.AddLeftShifted(BigUIntWord{-1}, i);
    BigUInt<64> expected = BigUInt<64>{BigUIntWord{-1}} << i;
    EXPECT_EQ(result, expected);
  }
}

TEST(BigUInt, AddLeftShiftedAll1sTo0DoubleWord) {
  for (size_t i = 0; i < BigUIntWord::bits_per_word * 2; ++i) {
    BigUInt<128> result(0);
    result.AddLeftShifted(BigUIntWord{-1}, i);
    BigUInt<128> expected = BigUInt<128>{BigUIntWord{-1}} << i;
    EXPECT_EQ(result, expected);
  }
}

TEST(BigUInt, AddLeftShiftedAll1sTo0TripleWord) {
  for (size_t i = 0; i < BigUIntWord::bits_per_word * 3; ++i) {
    BigUInt<192> result(0);
    result.AddLeftShifted(BigUIntWord{-1}, i);
    EXPECT_EQ(result, BigUInt<192>{BigUIntWord{-1}} << i);
  }
}

TEST(BigUInt, PlusEqual) {
  BigUInt<128> result(1);
  BigUInt<128> add(1);

  for (int i = 0; i < 120; ++i) {
    result += (add << i);
  }

  EXPECT_EQ(result, BigUInt<128>(1) << 120);
}

TEST(BigUInt, SubtractLeftShiftedAll1sFromAll1sSingleWord) {
  for (size_t i = 0; i < BigUIntWord::bits_per_word; ++i) {
    BigUInt<64> result(-1);
    result.SubtractLeftShifted(BigUInt<64>{-1}, i);
    BigUInt<64> expected = i > 0 ?
      BigUInt<64>{BigUIntWord{-1} >> (BigUIntWord::bits_per_word - i)} :
      BigUInt<64>{0};
    EXPECT_EQ(result, expected);
  }
}

TEST(BigUInt, SubtractLeftShiftedTripleFromTriple) {
  for (size_t i = 0; i < BigUIntWord::bits_per_word; ++i) {
    BigUInt<192> result(-1);
    result.SubtractLeftShifted(BigUInt<192>{-1}, i);
    BigUInt<64> expected = i > 0 ?
      BigUInt<64>{BigUIntWord{-1} >> (BigUIntWord::bits_per_word - i)} :
      BigUInt<64>{0};
    EXPECT_EQ(result, expected);
  }
}

TEST(BigUInt, SubtractLeftShiftedTripleUnderflow) {
  const BigUInt<192> all1s(-1);
  for (size_t i = 0; i < BigUIntWord::bits_per_word; ++i) {
    BigUInt<192> result(0);
    result.SubtractLeftShifted(BigUInt<64>{1}, i);
    BigUInt<192> expected = i > 0 ?
      all1s.Subtract(BigUInt<64>{BigUIntWord{-1} >> (BigUIntWord::bits_per_word - i)}) :
      all1s;
    EXPECT_EQ(result, expected);
  }
}

TEST(BigUInt, SubtractUInt64Carry) {
  BigUInt<64> a(static_cast<uint64_t>(1) << 32);
  BigUInt<64> b(static_cast<uint32_t>(1) << 31);
  BigUInt<64> result = a.Subtract<>(b);

  EXPECT_EQ(result.low_uint64(), static_cast<uint64_t>(1) << 31);
  EXPECT_EQ(result, b);
  EXPECT_LT(result, a);
}

TEST(BigUInt, SubtractUInt32Underflow) {
  BigUInt<128> a(static_cast<uint32_t>(1));
  BigUInt<64> b(static_cast<uint32_t>(2));
  BigUInt<128> result = a.Subtract<>(b);

  EXPECT_EQ(result.low_uint64(), static_cast<uint64_t>(-1));
  EXPECT_EQ(result.Add<>(b), a);
  EXPECT_GT(result,
            BigUInt<128>{static_cast<uint32_t>(1)} <<
            (2 * BigUIntWord::bits_per_word - 1));
}

TEST(BigUInt, SubtractUInt64Underflow) {
  BigUInt<128> a(static_cast<uint64_t>(1) << 32);
  BigUInt<64> b(static_cast<uint64_t>(2) << 32);
  BigUInt<128> result = a.Subtract<>(b);

  EXPECT_EQ(result.low_uint64(), static_cast<uint64_t>(-1) << 32);
  EXPECT_EQ(result.Add<>(b), a);
  EXPECT_GT(result,
            BigUInt<128>{static_cast<uint32_t>(1)} <<
            (2 * BigUIntWord::bits_per_word - 1));
}

TEST(BigUInt, SubtractCarryFrom127) {
  BigUInt<192> a = BigUInt<192>{1} << 128;
  BigUInt<128> b = BigUInt<128>{1} << 127;
  BigUInt<192> result = a.Subtract<>(b);

  EXPECT_EQ(result, b);
  EXPECT_LT(result, a);
}

TEST(BigUInt, SubtractLower128) {
  BigUInt<192> a(-1);
  BigUInt<128> b(-1);
  BigUInt<192> result = a.Subtract<>(b);

  BigUInt<192> expected = BigUInt<192>{static_cast<uint64_t>(-1)} << 128;
  EXPECT_EQ(result, expected);
}

TEST(BigUInt, SubtractLargeFromSmallWithUnderflow) {
  BigUInt<192> a(0);
  BigUInt<128> b(-1);
  BigUInt<192> result = a.Subtract<>(b);

  BigUInt<192> expected = (BigUInt<192>{static_cast<uint64_t>(-1)} << 128).Add(BigUInt<64>{1});
  EXPECT_EQ(result, expected);
}

TEST(BigUInt, SubtractLargeFromSmallWithUnderflow2) {
  BigUInt<192> a = BigUInt<192>{1} << 127;
  BigUInt<128> b(-1);
  BigUInt<192> result = a.Subtract<>(b);

  BigUInt<192> expected = (BigUInt<192>{static_cast<uint64_t>(-1)} << 128)
    .Add(BigUInt<192>{1} << 127).Add(BigUInt<64>{1});
  EXPECT_EQ(result, expected);
}

TEST(BigUInt, AddCarryUInt128Max) {
  BigUInt<128> a(-1);
  BigUInt<192> result = a.Add<3>(a);

  BigUInt<192> expected = (BigUInt<192>{a} << 1);
  EXPECT_EQ(result, expected);
}

TEST(BigUInt, MultiplyUInt32Max) {
  BigUInt<64> a(static_cast<uint32_t>(-1));
  BigUInt<64> b(static_cast<uint32_t>(-1));
  BigUInt<128> result = a.Multiply<>(b);

  BigUInt<128> expected = (BigUInt<128>{static_cast<uint32_t>(1)} << 64)
    .Subtract(a)
    .Subtract(b)
    .Subtract(BigUInt<64>{1});
  EXPECT_EQ(result, expected);
}

TEST(BigUInt, MultiplyUInt64Max) {
  BigUInt<64> a(static_cast<uint64_t>(-1));
  BigUInt<64> b(static_cast<uint64_t>(-1));
  BigUInt<128> result = a.Multiply<>(b);

  BigUInt<128> expected = (BigUInt<128>{static_cast<uint32_t>(1)} << 128)
    .Subtract(a)
    .Subtract(b)
    .Subtract(BigUInt<64>{1});
  EXPECT_EQ(result, expected);
}

TEST(BigUInt, MultiplyUInt64MaxToUInt128Max) {
  BigUInt<64> a(static_cast<uint64_t>(-1));
  BigUInt<128> b = (BigUInt<128>{1} << 128).Subtract<>(BigUInt<64>(1));
  BigUInt<192> result = a.Multiply<>(b);

  BigUInt<192> expected = (BigUInt<192>{static_cast<uint32_t>(1)} << 192)
    .Subtract(a)
    .Subtract(b)
    .Subtract(BigUInt<64>{1});
  EXPECT_EQ(result, expected);
}

TEST(BigUInt, MultiplyUInt32MaxWord) {
  BigUInt<64> a(-1);
  BigUIntWord b(static_cast<uint32_t>(-1));
  BigUInt<128> result = a.Multiply<>(b);

  BigUInt<128> expected = (BigUInt<128>{static_cast<uint32_t>(1)} << 96)
    .Subtract(a)
    .Subtract(BigUInt<64>{b})
    .Subtract(BigUInt<64>{1});
  EXPECT_EQ(result, expected);
}

TEST(BigUInt, MultiplyWordPower2) {
  const BigUInt<128> a(1);
  const BigUIntWord b(1);
  for (size_t i = 0; i < BigUIntWord::bits_per_word * 2; i++) {
    const BigUInt<192> a_shifted = a << i;
    for (size_t j = 0; j < BigUIntWord::bits_per_word - 2; j++) {
      const BigUIntWord b_shifted = b << j;
      const BigUInt<192> result = a_shifted.Multiply<>(b_shifted);
      const BigUInt<192> expected = (BigUInt<192>{1} << (i + j));
      EXPECT_EQ(result, expected);
    }
  }
}

TEST(BigUInt, Multiply128Carry) {
  // Perform a multiplication that requires a lot of carrying.
  const BigUInt<128> a = BigUInt<128>::max_value();
  const BigUInt<256> result = a*a;
  EXPECT_EQ(result - BigUInt<256>{2} + (BigUInt<256>{1} << 129),
            BigUInt<256>::max_value());
}

TEST(BigUInt, Divide1by1) {
  BigUInt<64> a(1);
  BigUInt<64> b(1);
  BigUInt<64> result = a / b;

  EXPECT_EQ(result, BigUInt<64>{1});
}

TEST(BigUInt, DivideP32by2) {
  BigUInt<64> a(static_cast<uint64_t>(1) << 32);
  BigUInt<64> b(2);
  BigUInt<64> result = a / b;

  BigUInt<64> expected(static_cast<uint64_t>(1) << 31);
  EXPECT_EQ(result, expected);
}

TEST(BigUInt, DivideP32byP31) {
  BigUInt<64> a(static_cast<uint64_t>(1) << 32);
  BigUInt<64> b(static_cast<uint64_t>(1) << 31);
  BigUInt<64> result = a / b;

  BigUInt<64> expected(2);
  EXPECT_EQ(result, expected);
}

TEST(BigUInt, DivideP33byP32) {
  BigUInt<64> a(static_cast<uint64_t>(1) << 33);
  BigUInt<64> b(static_cast<uint64_t>(1) << 32);
  BigUInt<64> result = a / b;

  BigUInt<64> expected(2);
  EXPECT_EQ(result, expected);
}

TEST(BigUInt, DivideUInt64MaxByUInt32Max) {
  BigUInt<64> a(static_cast<uint64_t>(-1));
  BigUInt<64> b(static_cast<uint32_t>(-1));
  BigUInt<64> result = a / b;

  BigUInt<64> expected = b.Add(BigUInt<64>{2});
  EXPECT_EQ(result, expected);
}

TEST(BigUInt, DivideUInt64MaxByUInt32Max2) {
  BigUInt<128> a = BigUInt<128>(static_cast<uint64_t>(-1)) << 64;
  BigUInt<128> b = BigUInt<128>(static_cast<uint32_t>(-1)) << 64;
  BigUInt<128> result = a / b;

  BigUInt<64> expected = BigUInt<64>{static_cast<uint32_t>(-1)}.Add(BigUInt<64>{2});
  EXPECT_EQ(result, expected);
}

TEST(BigUInt, DivideUInt64MaxByUInt32Max3) {
  BigUInt<192> a = BigUInt<192>(static_cast<uint64_t>(-1)) << 128;
  BigUInt<192> b = BigUInt<192>(static_cast<uint32_t>(-1)) << 128;
  BigUInt<192> result = a / b;

  BigUInt<64> expected = BigUInt<64>{static_cast<uint32_t>(-1)}.Add(BigUInt<64>{2});
  EXPECT_EQ(result, expected);
}

TEST(BigUInt, DivideSinglePower2) {
  for (size_t i = 1; i < BigUIntWord::bits_per_word * 1; i++) {
    const BigUInt<64> a = BigUInt<64>{1} << i;
    for (size_t j = 1; j <= i; j++) {
      const BigUInt<64> b = BigUInt<64>{1} << j;
      const BigUInt<64> result = a / b;
      const BigUInt<64> expected = (BigUInt<64>{1} << (i - j));
      EXPECT_EQ(result, expected);
    }
  }
}

TEST(BigUInt, DivideDoublePower2) {
  for (size_t i = 1; i < BigUIntWord::bits_per_word * 2; i++) {
    const BigUInt<128> a = BigUInt<128>{1} << i;
    for (size_t j = 1; j <= i; j++) {
      const BigUInt<128> b = BigUInt<128>{1} << j;
      const BigUInt<128> result = a / b;
      const BigUInt<128> expected = (BigUInt<128>{1} << (i - j));
      EXPECT_EQ(result, expected);
    }
  }
}

TEST(BigUInt, ModSinglePower2Plus1) {
  for (size_t i = 1; i < BigUIntWord::bits_per_word * 1; i++) {
    const BigUInt<64> a = (BigUInt<64>{1} << i) + BigUInt<64>{1};
    for (size_t j = 1; j <= i; j++) {
      const BigUInt<64> b = BigUInt<64>{1} << j;
      const BigUInt<64> result = a % b;
      const BigUInt<64> expected{1};
      EXPECT_EQ(result, expected);
    }
  }
}

TEST(BigUInt, ModP65plus1by2) {
  const BigUInt<128> a = (BigUInt<128>{1} << 64) + BigUInt<64>{1};
  const BigUInt<64> b{2};
  const BigUInt<128> result = a % b;
  const BigUInt<64> expected{1};
  EXPECT_EQ(result, expected);
}

TEST(BigUInt, ModDoublePower2Plus1) {
  for (size_t i = 1; i < BigUIntWord::bits_per_word * 2; i++) {
    const BigUInt<128> a = (BigUInt<128>{1} << i) + BigUInt<64>{1};
    for (size_t j = 1; j <= i; j++) {
      const BigUInt<128> b = BigUInt<128>{1} << j;
      const BigUInt<128> result = a % b;
      const BigUInt<64> expected{1};
      EXPECT_EQ(result, expected);
    }
  }
}

TEST(BigUInt, Gcd15And50) {
  BigUInt<64> a{15};
  BigUInt<64> b{50};
  BigUInt<64> result = a.GetGreatestCommonDivisor(b);
  BigUInt<64> expected{5};
  EXPECT_EQ(result, expected);
}

TEST(BigUInt, GcdLargePrimeMultiple) {
  // 338327950288419716939 split into the upper and lower 64 bits.
  BigUInt<128> prime = (BigUInt<128>{18} << 64) +
      BigUInt<128>{static_cast<uint64_t>(6286556961647787851)};

  BigUInt<128> a = BigUInt<128>{2} * prime;
  BigUInt<128> b = BigUInt<128>{3} * prime;
  BigUInt<128> result = a.GetGreatestCommonDivisor(b);
  EXPECT_EQ(result, prime);
}

TEST(BigUInt, Gcd5And0) {
  BigUInt<64> a{5};
  BigUInt<64> b{0};
  BigUInt<64> result = a.GetGreatestCommonDivisor(b);
  BigUInt<64> expected{5};
  EXPECT_EQ(result, expected);
}

TEST(BigUInt, Gcd0And5) {
  BigUInt<64> a{0};
  BigUInt<64> b{5};
  BigUInt<64> result = a.GetGreatestCommonDivisor(b);
  BigUInt<64> expected{5};
  EXPECT_EQ(result, expected);
}

}  // walnut
