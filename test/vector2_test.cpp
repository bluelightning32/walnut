#include "walnut/vector2.h"

#include <cmath>
#include <map>
#include <random>

#include "gtest/gtest.h"

namespace walnut {

TEST(Vector2, IsSameDir) {
  Vector2 v1a(0, 1);
  Vector2 v1b(0, 2);
  Vector2 v2a(1, 0);
  Vector2 v2b(10, 0);

  EXPECT_TRUE(v1a.IsSameDir(v1b));
  EXPECT_TRUE(v1b.IsSameDir(v1a));
  EXPECT_FALSE(v1a.IsSameDir(v2a));

  EXPECT_TRUE(v2a.IsSameDir(v2b));
  EXPECT_TRUE(v2b.IsSameDir(v2a));
  EXPECT_FALSE(v2a.IsSameDir(v1a));
}

TEST(Vector2, OppositeDirIsNotSame) {
  Vector2 v1(1, 1);
  Vector2 v2(-1, -1);
  Vector2 v3(-2, -2);

  EXPECT_FALSE(v1.IsSameDir(v2));
  EXPECT_FALSE(v1.IsSameDir(v3));
  EXPECT_TRUE(v2.IsSameDir(v3));
}

TEST(Vector2, IsSameOrOppositeDir) {
  Vector2 v1a(0, -1);
  Vector2 v1b(0, -2);
  Vector2 v1c(0, 1);
  Vector2 v1d(0, 2);
  Vector2 v2(1, 0);

  EXPECT_TRUE(v1a.IsSameOrOppositeDir(v1b));
  EXPECT_TRUE(v1b.IsSameOrOppositeDir(v1c));
  EXPECT_TRUE(v1c.IsSameOrOppositeDir(v1d));
  EXPECT_TRUE(v1d.IsSameOrOppositeDir(v1a));

  EXPECT_FALSE(v1a.IsSameOrOppositeDir(v2));
  EXPECT_FALSE(v1c.IsSameOrOppositeDir(v2));
}

TEST(Vector2, Minus) {
  Vector2 v1(1, 100);
  Vector2 v2(2, 200);

  EXPECT_EQ(v1 - v2, Vector2(-1, -100));
}

TEST(Vector3, MinusMax) {
  static constexpr int coord_bits = 640;
  BigInt min_value = BigInt::min_value(coord_bits - 1);
  BigInt max_value = BigInt::max_value(coord_bits - 1);
  Vector2 v1(min_value, min_value);
  Vector2 v2(max_value, max_value);

  BigInt expected = BigInt(min_value) - BigInt(max_value);

  EXPECT_EQ(v1 - v2, Vector2(expected, expected));
}

TEST(Vector2, Dot) {
  Vector2 v1(5, 0);
  Vector2 v2(10, 0);

  EXPECT_EQ(v1.Dot(v2), 50);
}

TEST(Vector2, GetScaleSquared) {
  Vector2 v1(1, 2);

  EXPECT_EQ(v1.GetScaleSquared(), 1*1 + 2*2);
}

TEST(Vector2, Scale) {
  Vector2 v1(0, 2);
  EXPECT_EQ(v1, v1.Scale(1));

  EXPECT_TRUE(v1.IsSameDir(v1.Scale(2)));
  EXPECT_EQ(v1.Scale(2).GetScaleSquared(), v1.GetScaleSquared() * 2 * 2);

  EXPECT_TRUE(v1.IsSameDir(v1.Scale(10)));
  EXPECT_EQ(v1.Scale(10).GetScaleSquared(), v1.GetScaleSquared() * 10 * 10);
}

TEST(Vector2, DotNegMax) {
  static constexpr int coord_bits = 64;
  BigInt min_value = BigInt::min_value(coord_bits - 1);
  // Note that since min_value uses all coord_bits, coord_bits must be
  // explicitly passed to Vector2, because coord_bits could be larger than
  // Vector2::coord_bits.
  Vector2 min_vector(min_value, min_value);

  BigInt expected_scale = min_value;
  expected_scale = expected_scale * min_value;
  expected_scale = expected_scale * 2;

  EXPECT_EQ(min_vector.Dot(min_vector), expected_scale);
}

TEST(Vector2, CrossMax) {
  static constexpr int coord_bits = 640;
  BigInt min_value = BigInt::min_value(coord_bits - 1);
  BigInt max_value = BigInt::max_value(coord_bits - 1);
  Vector2 v1(min_value, min_value);
  Vector2 v2(max_value, min_value);

  // Calculate the z coordinate of the cross product using an extra large
  // integer type, then verify it produces the same value.
  BigInt casted_min_value(min_value);
  BigInt casted_max_value(max_value);

  BigInt expected = casted_min_value*casted_min_value -
                    casted_min_value*casted_max_value;

  EXPECT_EQ(v1.Cross(v2), expected);
}

TEST(Vector2, IsHalfRotationLessThanXAxisAndYAxis) {
  Vector2 x(1, 0), y(0, 1);

  EXPECT_TRUE(x.IsHalfRotationLessThan(y));
  EXPECT_FALSE(y.IsHalfRotationLessThan(x));
}

TEST(Vector2, IsHalfRotationLessThanXAxisAndNegXAxisEquivalent) {
  Vector2 x(1, 0), neg_x(-1, 0);

  EXPECT_FALSE(x.IsHalfRotationLessThan(neg_x));
  EXPECT_FALSE(neg_x.IsHalfRotationLessThan(x));
}

TEST(Vector2, IsHalfRotationLessThanNegXAxisAndNegYAxis) {
  Vector2 neg_x(-1, 0), neg_y(0, -1);

  EXPECT_TRUE(neg_x.IsHalfRotationLessThan(neg_y));
  EXPECT_FALSE(neg_y.IsHalfRotationLessThan(neg_x));
}

TEST(Vector2, IsHalfRotationLessThanDifferentMagnitudes) {
  Vector2 u(2, 4), v(3, 6);

  EXPECT_FALSE(u.IsHalfRotationLessThan(v));
  EXPECT_FALSE(v.IsHalfRotationLessThan(u));
}

TEST(Vector2, HalfRotationCompareStoreInMap) {
  std::vector<std::pair<Vector2, int>> to_add;

  constexpr const double pi = 3.14159265358979323846;
  std::mt19937 gen;

  for (int i = 0; i < 32; i += 2) {
    const double angle = pi * i / 32;
    const double magnitude = gen() % 5 + 10;
    to_add.emplace_back(Vector2(static_cast<int>(cos(angle) * magnitude),
                                static_cast<int>(sin(angle) * magnitude)), i);
    const int extra_mult = gen() % 3 + 1;
    to_add.emplace_back(Vector2(-to_add.back().first.x() * extra_mult,
                                  -to_add.back().first.y() * extra_mult), i+1);
  }

  using Map = std::map<Vector2, int, Vector2::HalfRotationCompare>;
  Map sorted;
  while (!to_add.empty()) {
    size_t selected = gen() % to_add.size();
    sorted.emplace(to_add[selected].first, to_add[selected].second);
    to_add.erase(to_add.begin() + selected);
  }

  EXPECT_EQ(sorted.size(), 32/2);
  Map::iterator it = sorted.begin();
  for (int i = 0; it != sorted.end(); ++it, ++i) {
    EXPECT_EQ(it->second/2, i) << it->second;
  }
}

TEST(Vector2, IsRotationLessThanXAxisAndYAxis) {
  Vector2 x(1, 0), y(0, 1);

  EXPECT_TRUE(x.IsRotationLessThan(y));
  EXPECT_FALSE(y.IsRotationLessThan(x));
}

TEST(Vector2, IsRotationLessThanXAxisAndNegXAxis) {
  Vector2 x(1, 0), neg_x(-1, 0);

  EXPECT_TRUE(x.IsRotationLessThan(neg_x));
  EXPECT_FALSE(neg_x.IsRotationLessThan(x));
}

TEST(Vector2, IsRotationLessThanDifferentMagnitudes) {
  Vector2 u(2, 4), v(3, 6);

  EXPECT_FALSE(u.IsRotationLessThan(v));
  EXPECT_FALSE(v.IsRotationLessThan(u));
}

TEST(Vector2, RotationCompareStoreInMap) {
  constexpr int add_count = 32;
  constexpr const double pi = 3.14159265358979323846;
  std::mt19937 gen;

  std::vector<std::pair<Vector2, int>> to_add;
  for (int i = 0; i < add_count; ++i) {
    const double angle = 2 * pi * i / add_count;
    const double magnitude = gen() % 5 + 10;
    to_add.emplace_back(Vector2(static_cast<int>(cos(angle) * magnitude),
                                static_cast<int>(sin(angle) * magnitude)), i);
  }

  using Map = std::map<Vector2, int, Vector2::RotationCompare>;
  Map sorted;
  while (!to_add.empty()) {
    size_t selected = gen() % to_add.size();
    sorted.emplace(to_add[selected].first, to_add[selected].second);
    to_add.erase(to_add.begin() + selected);
  }

  EXPECT_EQ(sorted.size(), add_count);
  Map::iterator it = sorted.begin();
  for (int i = 0; it != sorted.end(); ++it, ++i) {
    EXPECT_EQ(it->second, i);
  }
}

}  // walnut
