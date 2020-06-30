#include "walnut/vertex.h"

#include "gtest/gtest.h"

namespace walnut {

TEST(Vector, IsSameDir) {
  Vector<> v1a(0, 0, 1);
  Vector<> v1b(0, 0, 2);
  Vector<> v2a(1, 0, 0);
  Vector<> v2b(10, 0, 0);

  EXPECT_TRUE(v1a.IsSameDir(v1b));
  EXPECT_TRUE(v1b.IsSameDir(v1a));
  EXPECT_FALSE(v1a.IsSameDir(v2a));

  EXPECT_TRUE(v2a.IsSameDir(v2b));
  EXPECT_TRUE(v2b.IsSameDir(v2a));
  EXPECT_FALSE(v2a.IsSameDir(v1a));
}

TEST(Vector, OppositeDirIsNotSame) {
  Vector<> v1(1, 1, 1);
  Vector<> v2(-1, -1, -1);
  Vector<> v3(-2, -2, -2);

  EXPECT_FALSE(v1.IsSameDir(v2));
  EXPECT_FALSE(v1.IsSameDir(v3));
  EXPECT_TRUE(v2.IsSameDir(v3));
}

TEST(Vector, IsSameOrOppositeDir) {
  Vector<> v1a(0, 0, -1);
  Vector<> v1b(0, 0, -2);
  Vector<> v1c(0, 0, 1);
  Vector<> v1d(0, 0, 2);
  Vector<> v2(1, 0, 0);

  EXPECT_TRUE(v1a.IsSameOrOppositeDir(v1b));
  EXPECT_TRUE(v1b.IsSameOrOppositeDir(v1c));
  EXPECT_TRUE(v1c.IsSameOrOppositeDir(v1d));
  EXPECT_TRUE(v1d.IsSameOrOppositeDir(v1a));

  EXPECT_FALSE(v1a.IsSameOrOppositeDir(v2));
  EXPECT_FALSE(v1c.IsSameOrOppositeDir(v2));
}

TEST(Vector, Minus) {
  Vector<> v1(1, 10, 100);
  Vector<> v2(2, 20, 200);

  EXPECT_EQ(v1 - v2, Vector<>(-1, -10, -100));
}

TEST(Vector, Dot) {
  Vector<> v1(5, 0, 0);
  Vector<> v2(10, 0, 0);

  EXPECT_EQ(v1.Dot(v2), 50);
}

TEST(Vector, GetScaleSquared) {
  Vector<> v1(1, 2, 3);

  EXPECT_EQ(v1.GetScaleSquared(), 1*1 + 2*2 + 3*3);
}

TEST(Vector, GetScaleSquaredMax) {
  static constexpr int coord_bits = Vertex3<>::BigIntRep::word_count *
                                        Vertex3<>::BigIntRep::bits_per_word;
  BigInt<coord_bits> min_value = BigInt<coord_bits>::min_value();
  // abs(int_min) > abs(int_max), so a vector with all int_min coordinates will
  // have the biggest scale.
  //
  // Note that since min_value uses all coord_bits, coord_bits must be
  // explicitly passed to Vector, because coord_bits could be larger than
  // Vector<>::coord_bits.
  Vector<coord_bits> min_vector(min_value, min_value, min_value);

  BigInt<coord_bits*2 + 5> expected_scale = min_value;
  expected_scale = expected_scale * min_value;
  expected_scale = expected_scale * 3;

  EXPECT_EQ(min_vector.GetScaleSquared(), expected_scale);
}

TEST(Vector, Scale) {
  Vector<> v1(0, 2, 1);
  EXPECT_EQ(v1, v1.Scale(1));

  EXPECT_TRUE(v1.IsSameDir(v1.Scale(2)));
  EXPECT_EQ(v1.Scale(2).GetScaleSquared(), v1.GetScaleSquared() * 2 * 2);

  EXPECT_TRUE(v1.IsSameDir(v1.Scale(10)));
  EXPECT_EQ(v1.Scale(10).GetScaleSquared(), v1.GetScaleSquared() * 10 * 10);
}

}  // walnut
