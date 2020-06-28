#include "walnut/vertex.h"

#include "gtest/gtest.h"

namespace walnut {

TEST(Vertex3, ConstructorAssertsOnOverflow) {
  static constexpr int big_coord_bits = Vertex3<32>::BigIntRep::max_bits*2;
  BigInt<big_coord_bits> big_coord = BigInt<big_coord_bits>::max_value();
  Vertex3<big_coord_bits> big_vertex(big_coord, big_coord, big_coord);

  ASSERT_DEBUG_DEATH(Vertex3<32> v(big_vertex), "overflow");
}

TEST(Vertex3, XYZIntConstructor) {
  Vertex3<> vertex(1, 2, 3);
  EXPECT_EQ(vertex.x(), 1);
  EXPECT_EQ(vertex.y(), 2);
  EXPECT_EQ(vertex.z(), 3);
}

TEST(Vector3, IsSameDir) {
  Vector3<> v1a(0, 0, 1);
  Vector3<> v1b(0, 0, 2);
  Vector3<> v2a(1, 0, 0);
  Vector3<> v2b(10, 0, 0);

  EXPECT_TRUE(v1a.IsSameDir(v1b));
  EXPECT_TRUE(v1b.IsSameDir(v1a));
  EXPECT_FALSE(v1a.IsSameDir(v2a));

  EXPECT_TRUE(v2a.IsSameDir(v2b));
  EXPECT_TRUE(v2b.IsSameDir(v2a));
  EXPECT_FALSE(v2a.IsSameDir(v1a));
}

TEST(Vector3, OppositeDirIsNotSame) {
  Vector3<> v1(1, 1, 1);
  Vector3<> v2(-1, -1, -1);
  Vector3<> v3(-2, -2, -2);

  EXPECT_FALSE(v1.IsSameDir(v2));
  EXPECT_FALSE(v1.IsSameDir(v3));
  EXPECT_TRUE(v2.IsSameDir(v3));
}

TEST(Vector3, Dot) {
  Vector3<> v1(5, 0, 0);
  Vector3<> v2(10, 0, 0);

  EXPECT_EQ(v1.Dot(v2), 50);
}

TEST(Vector3, GetScaleSquared) {
  Vector3<> v1(1, 2, 3);

  EXPECT_EQ(v1.GetScaleSquared(), 1*1 + 2*2 + 3*3);
}

TEST(Vector3, GetScaleSquaredMax) {
  static constexpr int coord_bits = Vector3<>::BigIntRep::max_bits;
  BigInt<coord_bits> min_value = BigInt<coord_bits>::min_value();
  // abs(int_min) > abs(int_max), so a vector with all int_min coordinates will
  // have the biggest scale.
  //
  // Note that since min_value uses all coord_bits, coord_bits must be
  // explicitly passed to Vector3, because coord_bits could be larger than
  // Vector3<>::coord_bits.
  Vector3<coord_bits> min_vector(min_value, min_value, min_value);

  BigInt<coord_bits*2 + 5> expected_scale = min_value;
  expected_scale = expected_scale * min_value;
  expected_scale = expected_scale * 3;

  EXPECT_EQ(min_vector.GetScaleSquared(), expected_scale);
}

TEST(Vector3, Scale) {
  Vector3<> v1(0, 2, 1);
  EXPECT_EQ(v1, v1.Scale(1));

  EXPECT_TRUE(v1.IsSameDir(v1.Scale(2)));
  EXPECT_EQ(v1.Scale(2).GetScaleSquared(), v1.GetScaleSquared() * 2 * 2);

  EXPECT_TRUE(v1.IsSameDir(v1.Scale(10)));
  EXPECT_EQ(v1.Scale(10).GetScaleSquared(), v1.GetScaleSquared() * 10 * 10);
}

}  // walnut
