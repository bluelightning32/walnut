#include "walnut/nudging_plane_builder.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

TEST(NudgingPlaneBuilder, BuildFromUnconstrained) {
  std::array<HomoPoint3, 3> vertices{
    HomoPoint3(1, 2, 3, 1),
    HomoPoint3(7, 11, 13, 1),
    HomoPoint3(17, 19, 23, 1),
  };

  NudgingPlaneBuilder builder;
  for (const HomoPoint3& vertex : vertices) {
    builder.AddUnconstrained(&vertex);
  }
  EXPECT_EQ(builder.Build(), HalfSpace3(vertices[0], vertices[1], vertices[2]));
}

TEST(NudgingPlaneBuilder, BuildFromConstrained) {
  std::array<HomoPoint3, 4> vertices{
    HomoPoint3(1, 2, 3, 1),
    HomoPoint3(7, 11, 13, 1),
    HomoPoint3(17, 19, 23, 1),
    HomoPoint3(29, 31, 37, 1),
  };

  NudgingPlaneBuilder builder;
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_TRUE(builder.TryAddConstrained(&vertices[i]));
  }
  EXPECT_FALSE(builder.TryAddConstrained(&vertices[3]));
  EXPECT_EQ(builder.Build(), HalfSpace3(vertices[0], vertices[1], vertices[2]));
}

TEST(NudgingPlaneBuilder, BuildFromUnConstrainedAndConstrained) {
  std::array<HomoPoint3, 4> vertices{
    HomoPoint3(1, 2, 3, 1),
    HomoPoint3(7, 11, 13, 1),
    HomoPoint3(17, 19, 23, 1),
    HomoPoint3(29, 31, 37, 1),
  };

  NudgingPlaneBuilder builder;
  EXPECT_TRUE(builder.TryAddConstrained(&vertices[0]));
  builder.AddUnconstrained(&vertices[1]);
  builder.AddUnconstrained(&vertices[2]);
  EXPECT_TRUE(builder.TryAddConstrained(&vertices[3]));
  EXPECT_EQ(builder.Build(), HalfSpace3(vertices[0], vertices[1], vertices[3]));
}

TEST(NudgingPlaneBuilder, ExceptMoreConstrainedOnPlane) {
  std::array<HomoPoint3, 7> vertices{
    HomoPoint3(0, 0, 0, 1),
    HomoPoint3(1, 0, 0, 1),
    HomoPoint3(0, 1, 0, 1),
    HomoPoint3(0, 2, 0, 1),
    HomoPoint3(0, 3, 0, 1),
    HomoPoint3(0, 4, 0, 1),
    HomoPoint3(0, 5, 0, 1),
  };

  NudgingPlaneBuilder builder;
  for (const HomoPoint3& vertex : vertices) {
    EXPECT_TRUE(builder.TryAddConstrained(&vertex));
  }
  EXPECT_EQ(builder.Build(), HalfSpace3(vertices[0], vertices[1], vertices[2]));
}

}  // walnut
