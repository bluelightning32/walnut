#include "walnut/vector2_half_rotation_compare.h"

#include <cmath>
#include <map>
#include <random>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/convex_polygon_factory.h"

namespace walnut {

TEST(Vector2HalfRotationCompare, XAxisAndYAxis) {
  Vector2<> x(1, 0), y(0, 1);

  Vector2HalfRotationCompare<> compare;
  EXPECT_TRUE(compare(x, y));
  EXPECT_FALSE(compare(y, x));
}

TEST(Vector2HalfRotationCompare, XAxisAndNegXAxisEquivalent) {
  Vector2<> x(1, 0), neg_x(-1, 0);

  Vector2HalfRotationCompare<> compare;
  EXPECT_FALSE(compare(x, neg_x));
  EXPECT_FALSE(compare(neg_x, x));
}

TEST(Vector2HalfRotationCompare, DifferentMagnitudesEquivalent) {
  Vector2<> u(2, 4), v(3, 6);

  Vector2HalfRotationCompare<> compare;
  EXPECT_FALSE(compare(u, v));
  EXPECT_FALSE(compare(v, u));
}

TEST(Vector2HalfRotationCompare, StoreInMap) {
  std::vector<std::pair<Vector2<>, int>> to_add;

  constexpr const double pi = 3.14159265358979323846;
  std::mt19937 gen;

  for (int i = 0; i < 30; i += 2) {
    const double angle = pi * i / 32;
    const double magnitude = gen() % 5 + 10;
    to_add.emplace_back(Vector2<>(cos(angle) * magnitude,
                                  sin(angle) * magnitude), i);
    const int extra_mult = gen() % 3 + 1;
    to_add.emplace_back(Vector2<>(to_add.back().first.x() * extra_mult,
                                  to_add.back().first.y() * extra_mult), i+1);
  }

  using Map = std::map<Vector2<>, int, Vector2HalfRotationCompare<>>;
  Map sorted;
  while (!to_add.empty()) {
    size_t selected = gen() % to_add.size();
    sorted.emplace(to_add[selected].first, to_add[selected].second);
    to_add.erase(to_add.begin() + selected);
  }

  EXPECT_EQ(sorted.size(), 15);
  Map::iterator it = sorted.begin();
  for (int i = 0; it != sorted.end(); ++it, ++i) {
    EXPECT_EQ(it->second/2, i);
  }
}

}  // walnut
