#include "walnut/big_uint_word.h"

#include "gtest/gtest.h"
#include "walnut/stop_watch.h"

namespace walnut {

TEST(BigUIntWordAcceleration, Multiply) {
  StopWatch accelerated_time;
  StopWatch unaccelerated_time;
  for (int attempt = 0, runs = 1000000; attempt < 10; attempt++, runs += runs/4) {
    {
      accelerated_time.Start();
      BigUIntWord product{2};
      for (int i = 0; i < runs; i++) {
        BigUIntWord high;
        product = product.Multiply(product, &high);
      }
      accelerated_time.Stop();
      std::cout << "accelerated time: " << accelerated_time << " product: " << product.low_uint64() << std::endl;
    }

    {
      unaccelerated_time.Start();
      BigUIntWordBase<> product{2};
      for (int i = 0; i < runs; i++) {
        BigUIntWordBase<> high;
        product = product.Multiply(product, &high);
      }
      unaccelerated_time.Stop();
      std::cout << "unaccelerated time: " << unaccelerated_time << " product: " << product.low_uint64() << std::endl;
    }
    if (accelerated_time.diff() <= unaccelerated_time.diff() * 1.1)
      break;
  }
  EXPECT_LE(accelerated_time.diff(), unaccelerated_time.diff() * 1.1);
}

template <typename T>
class BigUIntWordTest : public ::testing::Test {
};

// Used to build a tuple of all the BigInt subclasses that have a non-trivial
// implementation. The result is progressively built up in accepted_tuple as
// BigInts gets smaller. Eventually the inner most class sets filtered to
// accepted_tuple, and all the outer classes copy filtered from it.
template <class accepted_tuple, class... BigInts>
struct FilterTrivialBigInts {
  using filtered = accepted_tuple;
};

template <class First, class... Accepted, class... Remaining>
struct FilterTrivialBigInts<std::tuple<Accepted...>, First, Remaining...> {
  using now_accepted = typename std::conditional_t<
    First::has_overloads,
    std::tuple<Accepted..., First>,
    std::tuple<Accepted...>>;

  using filtered =
    typename FilterTrivialBigInts<now_accepted, Remaining...>::filtered;
};

using NontrivialBigUIntsTuple = FilterTrivialBigInts<std::tuple<>, BigUIntWordBase<>, BigUIntWordGCC<>>::filtered;

template <class tuple> struct TupleToTestingTypes { };

template <class... TupleTypes>
struct TupleToTestingTypes<std::tuple<TupleTypes...>> {
  using testing_types = ::testing::Types<TupleTypes...>;
};

using BigUIntWordTypes = TupleToTestingTypes<NontrivialBigUIntsTuple>::testing_types;
TYPED_TEST_SUITE(BigUIntWordTest, BigUIntWordTypes);

TYPED_TEST(BigUIntWordTest, SubtractUnderflowThroughCarry) {
  bool carry;
  TypeParam result = TypeParam(0).Subtract(TypeParam(0), true, &carry);
  EXPECT_EQ(result, TypeParam(-1));
  EXPECT_TRUE(carry);
}

TYPED_TEST(BigUIntWordTest, SubtractCompleteWrapAround) {
  bool carry;
  TypeParam result = TypeParam(0).Subtract(TypeParam(-1), true, &carry);
  EXPECT_EQ(result, TypeParam(0));
  EXPECT_TRUE(carry);

  result = TypeParam(-1).Subtract(TypeParam(-1), true, &carry);
  EXPECT_EQ(result, TypeParam(-1));
  EXPECT_TRUE(carry);
}

TYPED_TEST(BigUIntWordTest, AddOverflowThroughCarry) {
  bool carry;
  TypeParam result = TypeParam(-1).Add(TypeParam(0), true, &carry);
  EXPECT_EQ(result, TypeParam(0));
  EXPECT_TRUE(carry);
}

TYPED_TEST(BigUIntWordTest, AddCompleteWrapAround) {
  bool carry;
  TypeParam result = TypeParam(0).Add(TypeParam(-1), true, &carry);
  EXPECT_EQ(result, TypeParam(0));
  EXPECT_TRUE(carry);

  result = TypeParam(-1).Add(TypeParam(-1), true, &carry);
  EXPECT_EQ(result, TypeParam(-1));
  EXPECT_TRUE(carry);
}

TYPED_TEST(BigUIntWordTest, ShiftRightAllOnes) {
  TypeParam low{-1};
  TypeParam high{-1};
  for (size_t i = 0; i < TypeParam::bits_per_word; ++i) {
    EXPECT_EQ(low.ShiftRight(high, i), low);
  }
}

TYPED_TEST(BigUIntWordTest, ShiftRightLowAllOnes) {
  TypeParam low{-1};
  TypeParam high{0};
  for (size_t i = 0; i < TypeParam::bits_per_word; ++i) {
    EXPECT_EQ(low.ShiftRight(high, i), low >> i);
  }
}

TYPED_TEST(BigUIntWordTest, ShiftRightHighAllOnes) {
  TypeParam low{0};
  TypeParam high{-1};
  EXPECT_EQ(low.ShiftRight(high, 0), low);
  for (size_t i = 1; i < TypeParam::bits_per_word; ++i) {
    EXPECT_EQ(low.ShiftRight(high, i), high << (TypeParam::bits_per_word - i));
  }
}

TYPED_TEST(BigUIntWordTest, GetHighestSingleBitSet) {
  for (unsigned i = 0; i < TypeParam::bits_per_word; i++) {
    EXPECT_EQ((TypeParam{1} << i).GetHighestSetBit(), i + 1);
  }
}

TYPED_TEST(BigUIntWordTest, GetHighestSetBit0) {
  EXPECT_EQ(TypeParam{0}.GetHighestSetBit(), 0);
}

TYPED_TEST(BigUIntWordTest, GetHighestDoubleBitSet) {
  for (unsigned i = 0; i < TypeParam::bits_per_word; i++) {
    EXPECT_EQ((TypeParam{1} << i | TypeParam{1}).GetHighestSetBit(), i + 1);
  }
}

TYPED_TEST(BigUIntWordTest, GetTrailingZeros) {
  for (unsigned i = 0; i < TypeParam::bits_per_word; i++) {
    EXPECT_EQ((TypeParam{1} << i).GetTrailingZeros(), i);
  }
}

}  // walnut
