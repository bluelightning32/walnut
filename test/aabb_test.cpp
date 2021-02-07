#include "walnut/aabb.h"

#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::ElementsAre;

TEST(AABB, IsOnBorder) {
  AABB<> prism(2);

  EXPECT_TRUE(prism.IsOnBorder(Point3<>(2, 0, 0)));
  EXPECT_TRUE(prism.IsOnBorder(Point3<>(0, 2, 0)));
  EXPECT_TRUE(prism.IsOnBorder(Point3<>(0, 0, 2)));
  EXPECT_TRUE(prism.IsOnBorder(Point3<>(-2, 0, 0)));
  EXPECT_TRUE(prism.IsOnBorder(Point3<>(0, -2, 0)));
  EXPECT_TRUE(prism.IsOnBorder(Point3<>(0, 0, -2)));

  EXPECT_FALSE(prism.IsOnBorder(Point3<>(0, 0, 0)));
  EXPECT_FALSE(prism.IsOnBorder(Point3<>(1, 1, 1)));
}

TEST(AABB, IsOnBorderHomoPoint3) {
  AABB<> prism(Point3<>(-1, -1, -1), Point3<>(2, 2, 2));

  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3<>(2, 0, 0, 1)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3<>(-2, 0, 0, -1)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3<>(1, 0, 0, -1)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3<>(-1, 0, 0, 1)));

  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3<>(2, 0, 0, -1)));
  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3<>(-2, 0, 0, 1)));
  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3<>(1, 0, 0, 1)));
  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3<>(-1, 0, 0, -1)));
}

TEST(AABB, IsOnBorderHomoPoint3Denom2) {
  AABB<> prism(Vector3<>(-1, -1, -1), Vector3<>(2, 2, 2), 2);

  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3<>(2, 0, 0, 2)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3<>(1, 0, 0, 1)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3<>(-2, 0, 0, -2)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3<>(-1, 0, 0, -1)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3<>(1, 0, 0, -2)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3<>(-1, 0, 0, 2)));

  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3<>(2, 0, 0, -2)));
  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3<>(-2, 0, 0, 2)));
  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3<>(1, 0, 0, 2)));
  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3<>(-1, 0, 0, -2)));
}

TEST(AABB, IsInside) {
  AABB<> prism(2);

  EXPECT_TRUE(prism.IsInside(Point3<>(-2, 0, 0)));
  EXPECT_TRUE(prism.IsInside(Point3<>(2, 0, 0)));
  EXPECT_TRUE(prism.IsInside(Point3<>(0, 0, 0)));
  EXPECT_TRUE(prism.IsInside(Point3<>(1, 1, 1)));

  EXPECT_FALSE(prism.IsInside(Point3<>(3, 1, 1)));
}

TEST(AABB, IntersectPlaneZUp) {
  AABB<> prism(5);

  MutableConvexPolygon<32> result =
    prism.IntersectPlane(HalfSpace3<>(/*x=*/0, /*y=*/0, /*z=*/1, /*d=*/0));
  result.SortVertices();
  std::vector<HomoPoint3<>> vertices;
  for (size_t i = 0; i < result.vertex_count(); ++i) {
    vertices.push_back(result.vertex(i));
  }
  EXPECT_THAT(vertices, ElementsAre(
          Point3<32>{-5, -5, 0},
          Point3<32>{5, -5, 0},
          Point3<32>{5, 5, 0},
          Point3<32>{-5, 5, 0}
        ));
}

TEST(AABB, IntersectPlaneZUpFactional) {
  AABB<> prism(Point3<>(-8, -8, 0), Point3<>(8, 8, 10));
  auto result = prism.IntersectPlane(HalfSpace3<>(/*x=*/0, /*y=*/0, /*z=*/20,
                                                  /*d=*/19));
  result.SortVertices();
  std::vector<HomoPoint3<>> vertices;
  for (size_t i = 0; i < result.vertex_count(); ++i) {
    vertices.push_back(result.vertex(i));
  }
  EXPECT_THAT(vertices, ElementsAre(HomoPoint3<32>{-8*20, -8*20, 19, 20},
                                    HomoPoint3<32>{ 8*20, -8*20, 19, 20},
                                    HomoPoint3<32>{ 8*20,  8*20, 19, 20},
                                    HomoPoint3<32>{-8*20,  8*20, 19, 20}));
}

TEST(AABB, IntersectPlaneZDown) {
  AABB<> prism(5);

  MutableConvexPolygon<32> result =
    prism.IntersectPlane(HalfSpace3<>(/*x=*/0, /*y=*/0, /*z=*/-1, /*d=*/0));
  result.SortVertices();
  std::vector<HomoPoint3<>> vertices;
  for (size_t i = 0; i < result.vertex_count(); ++i) {
    vertices.push_back(result.vertex(i));
  }
  EXPECT_THAT(vertices, ElementsAre(
          Point3<32>{-5, -5, 0},
          Point3<32>{-5, 5, 0},
          Point3<32>{5, 5, 0},
          Point3<32>{5, -5, 0}
        ));
}

TEST(AABB, IntersectPlaneDiagPos) {
  AABB<> prism(5);

  MutableConvexPolygon<32> result =
    prism.IntersectPlane(HalfSpace3<>(/*x=*/1, /*y=*/1, /*z=*/1, /*d=*/12));
  result.SortVertices();
  std::vector<HomoPoint3<>> vertices;
  for (size_t i = 0; i < result.vertex_count(); ++i) {
    vertices.push_back(result.vertex(i));
  }
  EXPECT_THAT(vertices, ElementsAre(
          Point3<32>{2, 5, 5},
          Point3<32>{5, 2, 5},
          Point3<32>{5, 5, 2}
        ));
}

TEST(AABB, IntersectPlaneDiagNeg) {
  AABB<> prism(5);

  MutableConvexPolygon<32> result =
    prism.IntersectPlane(HalfSpace3<>(/*x=*/-1, /*y=*/-1, /*z=*/-1, /*d=*/12));
  result.SortVertices();
  std::vector<HomoPoint3<>> vertices;
  for (size_t i = 0; i < result.vertex_count(); ++i) {
    vertices.push_back(result.vertex(i));
  }
  EXPECT_THAT(vertices, ElementsAre(
          Point3<32>{-5, -5, -2},
          Point3<32>{-5, -2, -5},
          Point3<32>{-2, -5, -5}
        ));
}

template <size_t point3_bits>
void TestIntersectPlaneLowSlopeMin() {
  const BigInt<point3_bits> min_int = BigInt<point3_bits>::min_value();
  const BigInt<point3_bits> min_plus_1 = min_int + BigInt<point3_bits>(1);
  const BigInt<point3_bits> max_int = BigInt<point3_bits>::max_value();
  AABB<point3_bits> prism(Point3<point3_bits>(min_int,
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
    for (size_t i = 0; i < result.vertex_count(); ++i) {
      EXPECT_TRUE(prism.IsOnBorder(result.vertex(i)));
      EXPECT_TRUE(plane.IsCoincident(result.vertex(i)));
    }
  }
}

TEST(AABB, IntersectPlaneLowSlopeMin64) {
  TestIntersectPlaneLowSlopeMin<64>();
}

TEST(AABB, IntersectPlaneLowSlopeMin128) {
  TestIntersectPlaneLowSlopeMin<128>();
}

TEST(AABB, IntersectPlaneLowSlopeMin256) {
  TestIntersectPlaneLowSlopeMin<256>();
}

struct StringVertexData : public std::string {
  StringVertexData() = default;

  template <size_t num_bits, size_t denom_bits>
  StringVertexData(const StringVertexData& parent,
                   const HomoPoint3<num_bits, denom_bits>& new_source) { }

  template <size_t d_bits, size_t m_bits>
  StringVertexData(const StringVertexData& parent,
                   const PluckerLine<d_bits, m_bits>& new_line) { }

  template <size_t num_bits, size_t denom_bits, size_t d_bits, size_t m_bits>
  StringVertexData(const StringVertexData& parent,
                   const HomoPoint3<num_bits, denom_bits>& new_source,
                   const PluckerLine<d_bits, m_bits>& new_line) { }

  using std::string::operator=;
};

TEST(AABB, IntersectPlaneZUpWithData) {
  AABB<> prism(5);

  using ConvexPolygonRep = MutableConvexPolygon<32, StringVertexData>;
  ConvexPolygonRep result =
    prism.IntersectPlane<ConvexPolygonRep>(HalfSpace3<>(/*x=*/0,
                                                        /*y=*/0,
                                                        /*z=*/1,
                                                        /*d=*/0));
  result.SortVertices();
  std::vector<HomoPoint3<>> vertices;
  for (size_t i = 0; i < result.vertex_count(); ++i) {
    vertices.push_back(result.vertex(i));
    EXPECT_EQ(result.vertex_data(i), "");
    result.vertex_data(i) = std::to_string(i);
  }
  for (size_t i = 0; i < result.vertex_count(); ++i) {
    EXPECT_EQ(result.vertex_data(i), std::to_string(i));
  }
  EXPECT_THAT(vertices, ElementsAre(
          Point3<32>{-5, -5, 0},
          Point3<32>{5, -5, 0},
          Point3<32>{5, 5, 0},
          Point3<32>{-5, 5, 0}
        ));
}

class GetAABBPlaneSideTest : public testing::TestWithParam<int> {
 protected:
  AABB<> GetScaledAABB(const AABB<>& box) const {
    int mult = GetParam();
    return AABB<>(box.min_point_num()*mult, box.max_point_num()*mult, mult);
  }

  void ExpectNegativeSide(const HalfSpace3<>& plane, const AABB<>& box) {
    EXPECT_EQ(box.GetPlaneSide(plane), -1);

    EXPECT_EQ(GetScaledAABB(box).GetPlaneSide(plane), -1);

    EXPECT_EQ(box.GetPlaneSide(-plane), 1);

    EXPECT_EQ(GetScaledAABB(box).GetPlaneSide(-plane), 1);
  }

  void ExpectStraddle(const HalfSpace3<>& plane, const AABB<>& box) {
    EXPECT_EQ(box.GetPlaneSide(plane), 0);

    EXPECT_EQ(GetScaledAABB(box).GetPlaneSide(plane), 0);

    EXPECT_EQ(box.GetPlaneSide(-plane), 0);

    EXPECT_EQ(GetScaledAABB(box).GetPlaneSide(-plane), 0);
  }
};

TEST_P(GetAABBPlaneSideTest, SideOfXYZPosNormal) {
  HalfSpace3<> half_space(1, 1, 1, 10);
  AABB<> aabb(-1, -1, -1, 1, 1, 1, 1);
  ExpectNegativeSide(half_space, aabb);
}

TEST_P(GetAABBPlaneSideTest, SideOfXYZNegNormal) {
  HalfSpace3<> half_space(-1, -1, -1, 10);
  AABB<> aabb(-1, -1, -1, 1, 1, 1, 1);
  ExpectNegativeSide(half_space, aabb);
}

TEST_P(GetAABBPlaneSideTest, Touching) {
  HalfSpace3<> half_space(1, -10, 1, 30);
  AABB<> aabb(10, -2, 10, 11, -1, 11, 1);
  ExpectStraddle(half_space, aabb);
}

INSTANTIATE_TEST_SUITE_P(, GetAABBPlaneSideTest,
    testing::Values(-2, 1, -1, 2));

}  // walnut
