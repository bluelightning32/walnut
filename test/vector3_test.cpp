#include "walnut/vector3.h"

#include "gtest/gtest.h"
#include "walnut/point3.h"

namespace walnut {

TEST(Vector3, IsSameDir) {
  Vector3 v1a(0, 0, 1);
  Vector3 v1b(0, 0, 2);
  Vector3 v2a(1, 0, 0);
  Vector3 v2b(10, 0, 0);

  EXPECT_TRUE(v1a.IsSameDir(v1b));
  EXPECT_TRUE(v1b.IsSameDir(v1a));
  EXPECT_FALSE(v1a.IsSameDir(v2a));

  EXPECT_TRUE(v2a.IsSameDir(v2b));
  EXPECT_TRUE(v2b.IsSameDir(v2a));
  EXPECT_FALSE(v2a.IsSameDir(v1a));
}

TEST(Vector3, OppositeDirIsNotSame) {
  Vector3 v1(1, 1, 1);
  Vector3 v2(-1, -1, -1);
  Vector3 v3(-2, -2, -2);

  EXPECT_FALSE(v1.IsSameDir(v2));
  EXPECT_FALSE(v1.IsSameDir(v3));
  EXPECT_TRUE(v2.IsSameDir(v3));
}

TEST(Vector3, IsSameOrOppositeDir) {
  Vector3 v1a(0, 0, -1);
  Vector3 v1b(0, 0, -2);
  Vector3 v1c(0, 0, 1);
  Vector3 v1d(0, 0, 2);
  Vector3 v2(1, 0, 0);

  EXPECT_TRUE(v1a.IsSameOrOppositeDir(v1b));
  EXPECT_TRUE(v1b.IsSameOrOppositeDir(v1c));
  EXPECT_TRUE(v1c.IsSameOrOppositeDir(v1d));
  EXPECT_TRUE(v1d.IsSameOrOppositeDir(v1a));

  EXPECT_FALSE(v1a.IsSameOrOppositeDir(v2));
  EXPECT_FALSE(v1c.IsSameOrOppositeDir(v2));
}

TEST(Vector3, Minus) {
  Vector3 v1(1, 10, 100);
  Vector3 v2(2, 20, 200);

  EXPECT_EQ(v1 - v2, Vector3(-1, -10, -100));
}

TEST(Vector3, MinusMax) {
  static constexpr int coord_bits = 640;
  BigInt min_value = BigInt::min_value(coord_bits - 1);
  BigInt max_value = BigInt::max_value(coord_bits - 1);
  Vector3 v1(min_value, min_value, min_value);
  Vector3 v2(max_value, max_value, max_value);

  BigInt expected = min_value - max_value;

  EXPECT_EQ(v1 - v2, Vector3(expected, expected, expected));
}

TEST(Vector3, Dot) {
  Vector3 v1(5, 0, 0);
  Vector3 v2(10, 0, 0);

  EXPECT_EQ(v1.Dot(v2), 50);
}

TEST(Vector3, GetScaleSquared) {
  Vector3 v1(1, 2, 3);

  EXPECT_EQ(v1.GetScaleSquared(), 1*1 + 2*2 + 3*3);
}

TEST(Vector3, GetScaleSquaredMax) {
  static constexpr int coord_bits = 640;
  BigInt min_value = BigInt::min_value(coord_bits - 1);
  // abs(int_min) > abs(int_max), so a vector with all int_min coordinates will
  // have the biggest scale.
  Vector3 min_vector(min_value, min_value, min_value);

  BigInt expected_scale = min_value;
  expected_scale = expected_scale * min_value;
  expected_scale = expected_scale * 3;

  EXPECT_EQ(min_vector.GetScaleSquared(), expected_scale);
}

TEST(Vector3, Scale) {
  Vector3 v1(0, 2, 1);
  EXPECT_EQ(v1, v1.Scale(1));

  EXPECT_TRUE(v1.IsSameDir(v1.Scale(2)));
  EXPECT_EQ(v1.Scale(2).GetScaleSquared(), v1.GetScaleSquared() * 2 * 2);

  EXPECT_TRUE(v1.IsSameDir(v1.Scale(10)));
  EXPECT_EQ(v1.Scale(10).GetScaleSquared(), v1.GetScaleSquared() * 10 * 10);
}

TEST(Vector3, DotNegMax) {
  static constexpr int coord_bits = 640;
  BigInt min_value = BigInt::min_value(coord_bits - 1);
  Vector3 min_vector(min_value, min_value, min_value);

  BigInt expected_scale = min_value;
  expected_scale = expected_scale * min_value;
  expected_scale = expected_scale * 3;

  EXPECT_EQ(min_vector.Dot(min_vector), expected_scale);
}

TEST(Vector3, DotPosMax) {
  static constexpr int coord_bits = 640;
  BigInt max_value = BigInt::max_value(coord_bits - 1);
  Vector3 max_vector(max_value, max_value, max_value);

  BigInt expected_scale = max_value;
  expected_scale = expected_scale * max_value;
  expected_scale = expected_scale * 3;

  EXPECT_EQ(max_vector.Dot(max_vector), expected_scale);
}

TEST(Vector3, CrossMax) {
  static constexpr int coord_bits = 640;
  BigInt min_value = BigInt::min_value(coord_bits - 1);
  BigInt max_value = BigInt::max_value(coord_bits - 1);
  Vector3 v1(min_value, min_value, BigInt(0));
  Vector3 v2(max_value, min_value, BigInt(0));

  // Calculate the z coordinate of the cross product using an extra large
  // integer type, then verify it produces the same value.
  BigInt casted_min_value(min_value);
  BigInt casted_max_value(max_value);

  BigInt expected_z = casted_min_value*casted_min_value -
                                        casted_min_value*casted_max_value;

  EXPECT_EQ(v1.Cross(v2).z(), expected_z);
}

TEST(Vector3, DropDimensionSideness) {
  Vector3 a3(1, 2, 1);
  Vector3 b3(2, 1, 3);

  Vector3 c3 = a3.Cross(b3);

  for (int dimension = 0; dimension < 3; ++dimension) {
    Vector2 a2 = a3.DropDimension(dimension);
    Vector2 b2 = b3.DropDimension(dimension);

    BigInt c = a2.Cross(b2);

    EXPECT_FALSE(c3.components()[dimension].IsZero())
      << "dimension=" << dimension;
    if (c3.components()[dimension] > 0) {
      EXPECT_GT(c, 0) << "dimension=" << dimension;
    } else {
      EXPECT_LT(c, 0) << "dimension=" << dimension;
    }
  }
}

}  // walnut
