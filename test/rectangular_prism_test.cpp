#include "walnut/rectangular_prism.h"

#include <string>

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

TEST(RectangularPrism, IsOnBorderHomoPoint3) {
  RectangularPrism<> prism(Point3<>(-1, -1, -1), Point3<>(2, 2, 2));

  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3<>(2, 0, 0, 1)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3<>(-2, 0, 0, -1)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3<>(1, 0, 0, -1)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3<>(-1, 0, 0, 1)));

  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3<>(2, 0, 0, -1)));
  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3<>(-2, 0, 0, 1)));
  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3<>(1, 0, 0, 1)));
  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3<>(-1, 0, 0, -1)));
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

TEST(RectangularPrism, IntersectPlaneZUpFactional) {
  RectangularPrism<> prism(Point3<>(-8, -8, 0), Point3<>(8, 8, 10));
  auto result = prism.IntersectPlane(HalfSpace3<>(/*x=*/0, /*y=*/0, /*z=*/20,
                                                  /*d=*/19));
  result.SortVertices();
  std::vector<HomoPoint3<>> vertices;
  for (int i = 0; i < result.vertex_count(); ++i) {
    vertices.push_back(result.vertex(i));
  }
  EXPECT_THAT(vertices, ElementsAre(HomoPoint3<32>{-8*20, -8*20, 19, 20},
                                    HomoPoint3<32>{ 8*20, -8*20, 19, 20},
                                    HomoPoint3<32>{ 8*20,  8*20, 19, 20},
                                    HomoPoint3<32>{-8*20,  8*20, 19, 20}));
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

template <int point3_bits>
void TestIntersectPlaneLowSlopeMin() {
  const BigInt<point3_bits> min_int = BigInt<point3_bits>::min_value();
  const BigInt<point3_bits> min_plus_1 = min_int + BigInt<point3_bits>(1);
  const BigInt<point3_bits> max_int = BigInt<point3_bits>::max_value();
  RectangularPrism<point3_bits> prism(Point3<point3_bits>(min_int,
                                                          min_int,
                                                          min_int),
                                      Point3<point3_bits>(max_int,
                                                          max_int,
                                                          max_int));

  for (int i = 0; i < 3; ++i) {
    Point3<point3_bits> p[3] = {
      Point3<point3_bits>{min_int, min_int, min_int},
      Point3<point3_bits>{min_int, min_int, min_int},
      Point3<point3_bits>{min_int, min_int, min_int},
    };
    p[0].components()[i] = min_plus_1;
    p[1].components()[(i + 1) % 3] = max_int;
    p[2].components()[(i + 2) % 3] = max_int;

    using HalfSpace3Rep =
      typename HalfSpace3FromPoint3Builder<point3_bits>::HalfSpace3Rep;
    HalfSpace3Rep plane(p[0], p[1], p[2]);

    ConvexPolygon<point3_bits> result = prism.IntersectPlane(plane);
    EXPECT_EQ(result.vertex_count(), 3);
    for (int i = 0; i < result.vertex_count(); ++i) {
      EXPECT_TRUE(prism.IsOnBorder(result.vertex(i)));
      EXPECT_TRUE(plane.IsCoincident(result.vertex(i)));
    }
  }
}

TEST(RectangularPrism, IntersectPlaneLowSlopeMin64) {
  TestIntersectPlaneLowSlopeMin<64>();
}

TEST(RectangularPrism, IntersectPlaneLowSlopeMin128) {
  TestIntersectPlaneLowSlopeMin<128>();
}

TEST(RectangularPrism, IntersectPlaneLowSlopeMin256) {
  TestIntersectPlaneLowSlopeMin<256>();
}

struct StringVertexData : public std::string {
  StringVertexData() = default;

  template <int num_bits, int denom_bits>
  StringVertexData(const StringVertexData& parent,
                   const HomoPoint3<num_bits, denom_bits>& new_source) { }

  template <int d_bits, int m_bits>
  StringVertexData(const StringVertexData& parent,
                   const PluckerLine<d_bits, m_bits>& new_line) { }

  template <int num_bits, int denom_bits, int d_bits, int m_bits>
  StringVertexData(const StringVertexData& parent,
                   const HomoPoint3<num_bits, denom_bits>& new_source,
                   const PluckerLine<d_bits, m_bits>& new_line) { }

  using std::string::operator=;
};

TEST(RectangularPrism, IntersectPlaneZUpWithData) {
  RectangularPrism<> prism(5);

  using ConvexPolygonRep = ConvexPolygon<32, StringVertexData>;
  ConvexPolygonRep result =
    prism.IntersectPlane<ConvexPolygonRep>(HalfSpace3<>(/*x=*/0,
                                                        /*y=*/0,
                                                        /*z=*/1,
                                                        /*d=*/0));
  result.SortVertices();
  std::vector<HomoPoint3<>> vertices;
  for (int i = 0; i < result.vertex_count(); ++i) {
    vertices.push_back(result.vertex(i));
    EXPECT_EQ(result.vertex_data(i), "");
    result.vertex_data(i) = std::to_string(i);
  }
  for (int i = 0; i < result.vertex_count(); ++i) {
    EXPECT_EQ(result.vertex_data(i), std::to_string(i));
  }
  EXPECT_THAT(vertices, ElementsAre(
          Point3<32>{-5, -5, 0},
          Point3<32>{5, -5, 0},
          Point3<32>{5, 5, 0},
          Point3<32>{-5, 5, 0}
        ));
}

}  // walnut
