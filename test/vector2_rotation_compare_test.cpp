#include "walnut/vector2_rotation_compare.h"

#include <cmath>
#include <map>
#include <random>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

TEST(Vector2RotationCompare, XAxisAndYAxis) {
  Vector2<> x(1, 0), y(0, 1);

  Vector2RotationCompare<> compare;
  EXPECT_TRUE(compare(x, y));
  EXPECT_FALSE(compare(y, x));
}

TEST(Vector2RotationCompare, XAxisAndNegXAxis) {
  Vector2<> x(1, 0), neg_x(-1, 0);

  Vector2RotationCompare<> compare;
  EXPECT_TRUE(compare(x, neg_x));
  EXPECT_FALSE(compare(neg_x, x));
}

TEST(Vector2RotationCompare, DifferentMagnitudesEquivalent) {
  Vector2<> u(2, 4), v(3, 6);

  Vector2RotationCompare<> compare;
  EXPECT_FALSE(compare(u, v));
  EXPECT_FALSE(compare(v, u));
}

TEST(Vector2RotationCompare, StoreInMap) {
  constexpr int add_count = 32;
  constexpr const double pi = 3.14159265358979323846;
  std::mt19937 gen;

  std::vector<std::pair<Vector2<>, int>> to_add;
  for (int i = 0; i < add_count; ++i) {
    const double angle = 2 * pi * i / add_count;
    const double magnitude = gen() % 5 + 10;
    to_add.emplace_back(Vector2<>(cos(angle) * magnitude,
                                  sin(angle) * magnitude), i);
  }

  using Map = std::map<Vector2<>, int, Vector2RotationCompare<>>;
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
