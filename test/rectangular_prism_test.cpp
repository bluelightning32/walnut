#include "walnut/rectangular_prism.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::ElementsAre;

TEST(RectangularPrism, IsOnBorder) {
  RectangularPrism<> prism(2);

  EXPECT_TRUE(prism.IsOnBorder(Point3<>(2, 0, 0)));
  EXPECT_TRUE(prism.IsOnBorder(Point3<>(0, 2, 0)));
  EXPECT_TRUE(prism.IsOnBorder(Point3<>(0, 0, 2)));
  EXPECT_TRUE(prism.IsOnBorder(Point3<>(-2, 0, 0)));
  EXPECT_TRUE(prism.IsOnBorder(Point3<>(0, -2, 0)));
  EXPECT_TRUE(prism.IsOnBorder(Point3<>(0, 0, -2)));

  EXPECT_FALSE(prism.IsOnBorder(Point3<>(0, 0, 0)));
  EXPECT_FALSE(prism.IsOnBorder(Point3<>(1, 1, 1)));
}

TEST(RectangularPrism, IsInside) {
  RectangularPrism<> prism(2);

  EXPECT_TRUE(prism.IsInside(Point3<>(-2, 0, 0)));
  EXPECT_TRUE(prism.IsInside(Point3<>(2, 0, 0)));
  EXPECT_TRUE(prism.IsInside(Point3<>(0, 0, 0)));
  EXPECT_TRUE(prism.IsInside(Point3<>(1, 1, 1)));

  EXPECT_FALSE(prism.IsInside(Point3<>(3, 1, 1)));
}

TEST(RectangularPrism, IntersectPlaneZUp) {
  RectangularPrism<> prism(5);

  ConvexPolygon<32> result = prism.IntersectPlane(HalfSpace3<>(/*x=*/0,
                                                               /*y=*/0,
                                                               /*z=*/1,
                                                               /*d=*/0));
  result.SortVertices();
  std::vector<HomoPoint3<>> vertices;
  for (int i = 0; i < result.vertex_count(); ++i) {
    vertices.push_back(result.vertex(i));
  }
  EXPECT_THAT(vertices, ElementsAre(
          Point3<32>{-5, -5, 0},
          Point3<32>{5, -5, 0},
          Point3<32>{5, 5, 0},
          Point3<32>{-5, 5, 0}
        ));
}

TEST(RectangularPrism, IntersectPlaneZDown) {
  RectangularPrism<> prism(5);

  ConvexPolygon<32> result = prism.IntersectPlane(HalfSpace3<>(/*x=*/0,
                                                               /*y=*/0,
                                                               /*z=*/-1,
                                                               /*d=*/0));
  result.SortVertices();
  std::vector<HomoPoint3<>> vertices;
  for (int i = 0; i < result.vertex_count(); ++i) {
    vertices.push_back(result.vertex(i));
  }
  EXPECT_THAT(vertices, ElementsAre(
          Point3<32>{-5, -5, 0},
          Point3<32>{-5, 5, 0},
          Point3<32>{5, 5, 0},
          Point3<32>{5, -5, 0}
        ));
}

TEST(RectangularPrism, IntersectPlaneDiagPos) {
  RectangularPrism<> prism(5);

  ConvexPolygon<32> result = prism.IntersectPlane(HalfSpace3<>(/*x=*/1,
                                                               /*y=*/1,
                                                               /*z=*/1,
                                                               /*d=*/12));
  result.SortVertices();
  std::vector<HomoPoint3<>> vertices;
  for (int i = 0; i < result.vertex_count(); ++i) {
    vertices.push_back(result.vertex(i));
  }
  EXPECT_THAT(vertices, ElementsAre(
          Point3<32>{2, 5, 5},
          Point3<32>{5, 2, 5},
          Point3<32>{5, 5, 2}
        ));
}

TEST(RectangularPrism, IntersectPlaneDiagNeg) {
  RectangularPrism<> prism(5);

  ConvexPolygon<32> result = prism.IntersectPlane(HalfSpace3<>(/*x=*/-1,
                                                               /*y=*/-1,
                                                               /*z=*/-1,
                                                               /*d=*/12));
  result.SortVertices();
  std::vector<HomoPoint3<>> vertices;
  for (int i = 0; i < result.vertex_count(); ++i) {
    vertices.push_back(result.vertex(i));
  }
  EXPECT_THAT(vertices, ElementsAre(
          Point3<32>{-5, -5, -2},
          Point3<32>{-5, -2, -5},
          Point3<32>{-2, -5, -5}
        ));
}

}  // walnut
