#include "walnut/aabb.h"

#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::ElementsAre;

TEST(AABB, IsOnBorder) {
  AABB<> prism(2);

  EXPECT_TRUE(prism.IsOnBorder(Point3(2, 0, 0)));
  EXPECT_TRUE(prism.IsOnBorder(Point3(0, 2, 0)));
  EXPECT_TRUE(prism.IsOnBorder(Point3(0, 0, 2)));
  EXPECT_TRUE(prism.IsOnBorder(Point3(-2, 0, 0)));
  EXPECT_TRUE(prism.IsOnBorder(Point3(0, -2, 0)));
  EXPECT_TRUE(prism.IsOnBorder(Point3(0, 0, -2)));

  EXPECT_FALSE(prism.IsOnBorder(Point3(0, 0, 0)));
  EXPECT_FALSE(prism.IsOnBorder(Point3(1, 1, 1)));
}

TEST(AABB, IsOnBorderHomoPoint3) {
  AABB<> prism(Point3(-1, -1, -1), Point3(2, 2, 2));

  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3(2, 0, 0, 1)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3(-2, 0, 0, -1)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3(1, 0, 0, -1)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3(-1, 0, 0, 1)));

  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3(2, 0, 0, -1)));
  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3(-2, 0, 0, 1)));
  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3(1, 0, 0, 1)));
  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3(-1, 0, 0, -1)));
}

TEST(AABB, IsOnBorderHomoPoint3Denom2) {
  AABB<> prism(Vector3(-1, -1, -1), Vector3(2, 2, 2), 2);

  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3(2, 0, 0, 2)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3(1, 0, 0, 1)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3(-2, 0, 0, -2)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3(-1, 0, 0, -1)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3(1, 0, 0, -2)));
  EXPECT_TRUE(prism.IsOnBorder(HomoPoint3(-1, 0, 0, 2)));

  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3(2, 0, 0, -2)));
  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3(-2, 0, 0, 2)));
  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3(1, 0, 0, 2)));
  EXPECT_FALSE(prism.IsOnBorder(HomoPoint3(-1, 0, 0, -2)));
}

TEST(AABB, IsInside) {
  AABB<> prism(2);

  EXPECT_TRUE(prism.IsInside(Point3(-2, 0, 0)));
  EXPECT_TRUE(prism.IsInside(Point3(2, 0, 0)));
  EXPECT_TRUE(prism.IsInside(Point3(0, 0, 0)));
  EXPECT_TRUE(prism.IsInside(Point3(1, 1, 1)));

  EXPECT_FALSE(prism.IsInside(Point3(3, 1, 1)));
}

TEST(AABB, IntersectPlaneZUp) {
  AABB<> prism(5);

  MutableConvexPolygon<> result =
    prism.IntersectPlane(HalfSpace3(/*x=*/0, /*y=*/0, /*z=*/1, /*d=*/0));
  result.SortVertices();
  std::vector<HomoPoint3> vertices;
  for (size_t i = 0; i < result.vertex_count(); ++i) {
    vertices.push_back(result.vertex(i));
  }
  EXPECT_THAT(vertices, ElementsAre(
          Point3{-5, -5, 0},
          Point3{5, -5, 0},
          Point3{5, 5, 0},
          Point3{-5, 5, 0}
        ));
}

TEST(AABB, IntersectPlaneZUpFactional) {
  AABB<> prism(Point3(-8, -8, 0), Point3(8, 8, 10));
  auto result = prism.IntersectPlane(HalfSpace3(/*x=*/0, /*y=*/0, /*z=*/20,
                                                  /*d=*/19));
  result.SortVertices();
  std::vector<HomoPoint3> vertices;
  for (size_t i = 0; i < result.vertex_count(); ++i) {
    vertices.push_back(result.vertex(i));
  }
  EXPECT_THAT(vertices, ElementsAre(HomoPoint3{-8*20, -8*20, 19, 20},
                                    HomoPoint3{ 8*20, -8*20, 19, 20},
                                    HomoPoint3{ 8*20,  8*20, 19, 20},
                                    HomoPoint3{-8*20,  8*20, 19, 20}));
}

TEST(AABB, IntersectPlaneZDown) {
  AABB<> prism(5);

  MutableConvexPolygon<> result =
    prism.IntersectPlane(HalfSpace3(/*x=*/0, /*y=*/0, /*z=*/-1, /*d=*/0));
  result.SortVertices();
  std::vector<HomoPoint3> vertices;
  for (size_t i = 0; i < result.vertex_count(); ++i) {
    vertices.push_back(result.vertex(i));
  }
  EXPECT_THAT(vertices, ElementsAre(
          Point3{-5, -5, 0},
          Point3{-5, 5, 0},
          Point3{5, 5, 0},
          Point3{5, -5, 0}
        ));
}

TEST(AABB, IntersectPlaneDiagPos) {
  AABB<> prism(5);

  MutableConvexPolygon<> result =
    prism.IntersectPlane(HalfSpace3(/*x=*/1, /*y=*/1, /*z=*/1, /*d=*/12));
  result.SortVertices();
  std::vector<HomoPoint3> vertices;
  for (size_t i = 0; i < result.vertex_count(); ++i) {
    vertices.push_back(result.vertex(i));
  }
  EXPECT_THAT(vertices, ElementsAre(
          Point3{2, 5, 5},
          Point3{5, 2, 5},
          Point3{5, 5, 2}
        ));
}

TEST(AABB, IntersectPlaneDiagNeg) {
  AABB<> prism(5);

  MutableConvexPolygon<> result =
    prism.IntersectPlane(HalfSpace3(/*x=*/-1, /*y=*/-1, /*z=*/-1, /*d=*/12));
  result.SortVertices();
  std::vector<HomoPoint3> vertices;
  for (size_t i = 0; i < result.vertex_count(); ++i) {
    vertices.push_back(result.vertex(i));
  }
  EXPECT_THAT(vertices, ElementsAre(
          Point3{-5, -5, -2},
          Point3{-5, -2, -5},
          Point3{-2, -5, -5}
        ));
}

template <size_t point3_bits>
void TestIntersectPlaneLowSlopeMin() {
  const BigInt<point3_bits> min_int = BigInt<point3_bits>::min_value();
  const BigInt<point3_bits> min_plus_1 = min_int + BigInt<point3_bits>(1);
  const BigInt<point3_bits> max_int = BigInt<point3_bits>::max_value();
  AABB<point3_bits> prism(Point3(min_int, min_int, min_int),
                          Point3(max_int, max_int, max_int));

  for (int i = 0; i < 3; ++i) {
    Point3 p[3] = {
      Point3{min_int, min_int, min_int},
      Point3{min_int, min_int, min_int},
      Point3{min_int, min_int, min_int},
    };
    p[0].components()[i] = min_plus_1;
    p[1].components()[(i + 1) % 3] = max_int;
    p[2].components()[(i + 2) % 3] = max_int;

    HalfSpace3 plane(p[0], p[1], p[2]);

    ConvexPolygon<> result = prism.IntersectPlane(plane);
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

TEST(AABB, Equality) {
  AABB<> prism1(-1, -2, -3, 4, 5, 6, /*denom=*/1);
  AABB<> prism2(-1, -2, -3, 4, 5, 6, /*denom=*/2);
  AABB<> prism3(-2, -4, -6, 8, 10, 12, /*denom=*/2);

  EXPECT_EQ(prism1, prism3);
  EXPECT_NE(prism1, prism2);
  EXPECT_NE(prism2, prism3);
}

TEST(AABB, Assignment) {
  AABB<> prism1(-1, -2, -3, 4, 5, 6, /*denom=*/1);
  AABB<> prism2(-1, -2, -3, 4, 5, 6, /*denom=*/2);
  EXPECT_NE(prism1, prism2);

  prism1 = prism2;
  EXPECT_EQ(prism1, prism2);
}

struct StringVertexData : public EdgeInfoRoot {
  StringVertexData() = default;

  StringVertexData(const StringVertexData& parent,
                   const HomoPoint3& new_source) :
    str(parent.str) { }

  StringVertexData(const StringVertexData& parent,
                   const PluckerLine& new_line) :
    str(parent.str) { }

  StringVertexData(const StringVertexData& parent,
                   const HomoPoint3& new_source, const PluckerLine& new_line) :
    str(parent.str) { }

  std::string str;
};

TEST(AABB, IntersectPlaneZUpWithData) {
  AABB<> prism(5);

  using ConvexPolygonRep = MutableConvexPolygon<StringVertexData>;
  ConvexPolygonRep result =
    prism.IntersectPlane<ConvexPolygonRep>(HalfSpace3(/*x=*/0,
                                                        /*y=*/0,
                                                        /*z=*/1,
                                                        /*d=*/0));
  result.SortVertices();
  std::vector<HomoPoint3> vertices;
  for (size_t i = 0; i < result.vertex_count(); ++i) {
    vertices.push_back(result.vertex(i));
    EXPECT_EQ(result.edge(i).str, "");
    result.edge(i).str = std::to_string(i);
  }
  for (size_t i = 0; i < result.vertex_count(); ++i) {
    EXPECT_EQ(result.edge(i).str, std::to_string(i));
  }
  EXPECT_THAT(vertices, ElementsAre(
          Point3{-5, -5, 0},
          Point3{5, -5, 0},
          Point3{5, 5, 0},
          Point3{-5, 5, 0}
        ));
}

class GetAABBPlaneSideTest : public testing::TestWithParam<int> {
 protected:
  AABB<> GetScaledAABB(const AABB<>& box) const {
    int mult = GetParam();
    return AABB<>(box.min_point_num()*mult, box.max_point_num()*mult, mult);
  }

  void ExpectNegativeSide(const HalfSpace3& plane, const AABB<>& box) {
    EXPECT_EQ(box.GetPlaneSide(plane), -1);

    EXPECT_EQ(GetScaledAABB(box).GetPlaneSide(plane), -1);

    EXPECT_EQ(box.GetPlaneSide(-plane), 1);

    EXPECT_EQ(GetScaledAABB(box).GetPlaneSide(-plane), 1);
  }

  void ExpectStraddle(const HalfSpace3& plane, const AABB<>& box) {
    EXPECT_EQ(box.GetPlaneSide(plane), 0);

    EXPECT_EQ(GetScaledAABB(box).GetPlaneSide(plane), 0);

    EXPECT_EQ(box.GetPlaneSide(-plane), 0);

    EXPECT_EQ(GetScaledAABB(box).GetPlaneSide(-plane), 0);
  }
};

TEST_P(GetAABBPlaneSideTest, SideOfXYZPosNormal) {
  HalfSpace3 half_space(1, 1, 1, 10);
  AABB<> aabb(-1, -1, -1, 1, 1, 1, 1);
  ExpectNegativeSide(half_space, aabb);
}

TEST_P(GetAABBPlaneSideTest, SideOfXYZNegNormal) {
  HalfSpace3 half_space(-1, -1, -1, 10);
  AABB<> aabb(-1, -1, -1, 1, 1, 1, 1);
  ExpectNegativeSide(half_space, aabb);
}

TEST_P(GetAABBPlaneSideTest, Touching) {
  HalfSpace3 half_space(1, -10, 1, 30);
  AABB<> aabb(10, -2, 10, 11, -1, 11, 1);
  ExpectStraddle(half_space, aabb);
}

INSTANTIATE_TEST_SUITE_P(, GetAABBPlaneSideTest,
    testing::Values(-2, 1, -1, 2));

}  // walnut
