#include "walnut/vector2.h"

#include "gtest/gtest.h"

namespace walnut {

TEST(Vector2, IsSameDir) {
  Vector2<> v1a(0, 1);
  Vector2<> v1b(0, 2);
  Vector2<> v2a(1, 0);
  Vector2<> v2b(10, 0);

  EXPECT_TRUE(v1a.IsSameDir(v1b));
  EXPECT_TRUE(v1b.IsSameDir(v1a));
  EXPECT_FALSE(v1a.IsSameDir(v2a));

  EXPECT_TRUE(v2a.IsSameDir(v2b));
  EXPECT_TRUE(v2b.IsSameDir(v2a));
  EXPECT_FALSE(v2a.IsSameDir(v1a));
}

TEST(Vector2, OppositeDirIsNotSame) {
  Vector2<> v1(1, 1);
  Vector2<> v2(-1, -1);
  Vector2<> v3(-2, -2);

  EXPECT_FALSE(v1.IsSameDir(v2));
  EXPECT_FALSE(v1.IsSameDir(v3));
  EXPECT_TRUE(v2.IsSameDir(v3));
}

TEST(Vector2, IsSameOrOppositeDir) {
  Vector2<> v1a(0, -1);
  Vector2<> v1b(0, -2);
  Vector2<> v1c(0, 1);
  Vector2<> v1d(0, 2);
  Vector2<> v2(1, 0);

  EXPECT_TRUE(v1a.IsSameOrOppositeDir(v1b));
  EXPECT_TRUE(v1b.IsSameOrOppositeDir(v1c));
  EXPECT_TRUE(v1c.IsSameOrOppositeDir(v1d));
  EXPECT_TRUE(v1d.IsSameOrOppositeDir(v1a));

  EXPECT_FALSE(v1a.IsSameOrOppositeDir(v2));
  EXPECT_FALSE(v1c.IsSameOrOppositeDir(v2));
}

TEST(Vector2, Minus) {
  Vector2<> v1(1, 100);
  Vector2<> v2(2, 200);

  EXPECT_EQ(v1 - v2, Vector2<>(-1, -100));
}

TEST(Vector3, MinusMax) {
  static constexpr int coord_bits = Vector2<>::BigIntRep::word_count *
                                    Vector2<>::BigIntRep::bits_per_word *
                                    10;
  BigInt<coord_bits> min_value = BigInt<coord_bits>::min_value();
  BigInt<coord_bits> max_value = BigInt<coord_bits>::max_value();
  Vector2<coord_bits> v1(min_value, min_value);
  Vector2<coord_bits> v2(max_value, max_value);

  BigInt<coord_bits + 1> expected = BigInt<coord_bits + 1>(min_value) -
                                    BigInt<coord_bits + 1>(max_value);

  EXPECT_EQ(v1 - v2, Vector2<coord_bits + 1>(expected, expected));
}

TEST(Vector2, Dot) {
  Vector2<> v1(5, 0);
  Vector2<> v2(10, 0);

  EXPECT_EQ(v1.Dot(v2), 50);
}

TEST(Vector2, GetScaleSquared) {
  Vector2<> v1(1, 2);

  EXPECT_EQ(v1.GetScaleSquared(), 1*1 + 2*2);
}

TEST(Vector2, Scale) {
  Vector2<> v1(0, 2);
  EXPECT_EQ(v1, v1.Scale(1));

  EXPECT_TRUE(v1.IsSameDir(v1.Scale(2)));
  EXPECT_EQ(v1.Scale(2).GetScaleSquared(), v1.GetScaleSquared() * 2 * 2);

  EXPECT_TRUE(v1.IsSameDir(v1.Scale(10)));
  EXPECT_EQ(v1.Scale(10).GetScaleSquared(), v1.GetScaleSquared() * 10 * 10);
}

TEST(Vector2, DotNegMax) {
  static constexpr int coord_bits = Vector2<>::BigIntRep::word_count *
                                    Vector2<>::BigIntRep::bits_per_word;
  BigInt<coord_bits> min_value = BigInt<coord_bits>::min_value();
  // Note that since min_value uses all coord_bits, coord_bits must be
  // explicitly passed to Vector2, because coord_bits could be larger than
  // Vector2<>::coord_bits.
  Vector2<coord_bits> min_vector(min_value, min_value);

  BigInt<coord_bits*2 + 5> expected_scale = min_value;
  expected_scale = expected_scale * min_value;
  expected_scale = expected_scale * 2;

  EXPECT_EQ(min_vector.Dot(min_vector), expected_scale);
}

TEST(Vector2, CrossMax) {
  static constexpr int coord_bits = Vector2<>::BigIntRep::word_count *
                                    Vector2<>::BigIntRep::bits_per_word *
                                    10;
  BigInt<coord_bits> min_value = BigInt<coord_bits>::min_value();
  BigInt<coord_bits> max_value = BigInt<coord_bits>::max_value();
  Vector2<coord_bits> v1(min_value, min_value);
  Vector2<coord_bits> v2(max_value, min_value);

  // Calculate the z coordinate of the cross product using an extra large
  // integer type, then verify it produces the same value.
  BigInt<coord_bits*2 + 5> casted_min_value(min_value);
  BigInt<coord_bits*2 + 5> casted_max_value(max_value);

  BigInt<coord_bits*2 + 5> expected = casted_min_value*casted_min_value -
                                        casted_min_value*casted_max_value;

  EXPECT_EQ(v1.Cross(v2), expected);

  auto allowed_max = decltype(v1.Cross(v2))::max_value();
  EXPECT_GE(allowed_max, expected);
}

}  // walnut
