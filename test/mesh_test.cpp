#include "walnut/mesh.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/aabb.h"

namespace walnut {

TEST(Mesh, GetCentroid1x1x1Cube) {
  AABB box(0, 0, 0, 1, 1, 1, /*denom=*/1);
  EXPECT_EQ(GetCentroid(box.GetWalls()), HomoPoint3(1, 1, 1, 2));
}

TEST(Mesh, GetCentroid1x2x1Cube) {
  AABB box(0, 0, 0, 1, 2, 1, /*denom=*/1);
  EXPECT_EQ(GetCentroid(box.GetWalls()), HomoPoint3(1, 2, 1, 2));
}

TEST(Mesh, GetCentroid1x1x4Cube) {
  AABB box(0, 0, 0, 1, 1, 4, /*denom=*/1);
  EXPECT_EQ(GetCentroid(box.GetWalls()), HomoPoint3(1, 1, 4, 2));
}

TEST(Mesh, GetCentroidOffset1x1x1Cube) {
  AABB box(10, 10, 10, 11, 11, 11, /*denom=*/1);
  EXPECT_EQ(GetCentroid(box.GetWalls()), HomoPoint3(21, 21, 21, 2));
}

TEST(Mesh, GetCentroidOffsetHalfCube) {
  AABB box(1, 1, 1, 2, 2, 2, /*denom=*/2);
  EXPECT_EQ(GetCentroid(box.GetWalls()), HomoPoint3(3, 3, 3, 4));
}

}  // walnut
