#include "walnut/r_transformation.h"

#include "gtest/gtest.h"
#include "walnut/homo_point3.h"
#include "walnut/half_space3.h"

namespace walnut {

TEST(RXYCompareBivector, DiagonalPlanes) {
  //
  // p3&p4   p1&p2            |
  //      \ /                 |
  //       p0                 |
  //
  //        n2                |
  //       /                  |
  //       \                  |
  //        n1                |
  //
  HomoPoint3<> p[5] = {
    HomoPoint3<>(1, 1, 5, 1),
    HomoPoint3<>(2, 2, 5, 1),
    HomoPoint3<>(2, 2, 6, 1),
    HomoPoint3<>(0, 2, 5, 1),
    HomoPoint3<>(0, 2, 6, 1),
  };

  HalfSpace3<> h1(p[0], p[1], p[2]);
  EXPECT_GT(h1.x(), 0);
  EXPECT_LT(h1.y(), 0);
  EXPECT_EQ(h1.z(), 0);

  HalfSpace3<> h2(p[0], p[3], p[4]);
  EXPECT_GT(h2.x(), 0);
  EXPECT_GT(h2.y(), 0);
  EXPECT_EQ(h2.z(), 0);

  EXPECT_GT(h1.normal().DropDimension(2).Cross(
        h2.normal().DropDimension(2)), 0);

  HomoPoint3<> p_transformed[5];
  for (int i = 0; i < 5; ++i) {
    p_transformed[i] = RTransform(p[i], HomoPoint3<>::DenomInt(10));
  }

  HalfSpace3<> h1_transformed(p_transformed[0], p_transformed[1],
                              p_transformed[2]);
  HalfSpace3<> h2_transformed(p_transformed[0], p_transformed[3],
                              p_transformed[4]);

  EXPECT_GT(h1_transformed.normal().DropDimension(2).Cross(
        h2_transformed.normal().DropDimension(2)), 0);

  EXPECT_EQ(RXYCompareBivector(h1.normal(), h2.normal()), 1);
}

TEST(RXYCompareBivector, Y0Normal) {
  //
  // p3 p2 p1
  //   \_ \| 
  //     - p0
  //
  //      n1 ->
  //      n2 ->
  //
  HomoPoint3<> p[4] = {
    HomoPoint3<>(1, 1, 5, 1),
    HomoPoint3<>(1, 2, 5, 1),
    HomoPoint3<>(0, 2, 6, 1),
    HomoPoint3<>(-1, 2, 6, 1),
  };

  HalfSpace3<> h1(p[0], p[1], p[2]);
  EXPECT_GT(h1.x(), 0);
  EXPECT_EQ(h1.y(), 0);
  EXPECT_GT(h1.z(), 0);

  HalfSpace3<> h2(p[0], p[1], p[3]);
  EXPECT_GT(h2.x(), 0);
  EXPECT_EQ(h2.y(), 0);
  EXPECT_GT(h2.z(), 0);

  EXPECT_GT(h2.z(), h1.z());
  EXPECT_NE(h1.normal(), h2.normal());

  HomoPoint3<> p_transformed[4];
  for (int i = 0; i < 4; ++i) {
    p_transformed[i] = RTransform(p[i], HomoPoint3<>::DenomInt(10));
  }

  HalfSpace3<> h1_transformed(p_transformed[0], p_transformed[1],
                              p_transformed[2]);
  HalfSpace3<> h2_transformed(p_transformed[0], p_transformed[1],
                              p_transformed[3]);

  EXPECT_LT(h1_transformed.normal().DropDimension(2).Cross(
        h2_transformed.normal().DropDimension(2)), 0);

  EXPECT_EQ(RXYCompareBivector(h1.normal(), h2.normal()), -1);
}

TEST(RXYCompareBivector, X0Normal) {
  //
  //   p1- p0
  //   p2/ |
  //   p3-/
  //
  //       ^
  //       |
  //      n1
  //
  //       ^
  //       |
  //      n2
  //
  HomoPoint3<> p[4] = {
    HomoPoint3<>(1, 1, 5, 1),
    HomoPoint3<>(0, 1, 5, 1),
    HomoPoint3<>(0, -1, 6, 1),
    HomoPoint3<>(0, -2, 6, 1),
  };

  HalfSpace3<> h1(p[0], p[1], p[2]);
  EXPECT_EQ(h1.x(), 0);
  EXPECT_GT(h1.y(), 0);
  EXPECT_GT(h1.z(), 0);

  HalfSpace3<> h2(p[0], p[1], p[3]);
  EXPECT_EQ(h2.x(), 0);
  EXPECT_GT(h2.y(), 0);
  EXPECT_GT(h2.z(), 0);

  EXPECT_GT(h2.z(), h1.z());
  EXPECT_NE(h1.normal(), h2.normal());

  HomoPoint3<> p_transformed[4];
  for (int i = 0; i < 4; ++i) {
    p_transformed[i] = RTransform(p[i], HomoPoint3<>::DenomInt(10));
  }

  HalfSpace3<> h1_transformed(p_transformed[0], p_transformed[1],
                              p_transformed[2]);
  HalfSpace3<> h2_transformed(p_transformed[0], p_transformed[1],
                              p_transformed[3]);

  EXPECT_GT(h1_transformed.normal().DropDimension(2).Cross(
        h2_transformed.normal().DropDimension(2)), 0);

  EXPECT_EQ(RXYCompareBivector(h1.normal(), h2.normal()), 1);
}

}  // walnut
