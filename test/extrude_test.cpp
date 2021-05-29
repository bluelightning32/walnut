#include "walnut/extrude.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::UnorderedElementsAre;

TEST(Extrude, TriangularPrism) {
  std::vector<walnut::HomoPoint3> bottom_cap_vertices{
    walnut::HomoPoint3(0, 0, 0, 1),
    walnut::HomoPoint3(1, 1, 0, 1),
    walnut::HomoPoint3(1, 0, 0, 1),
  };

  MutableConvexPolygon<> bottom_cap(/*plane=*/HalfSpace3(0, 0, -1, 0),
                                    /*drop_dimension=*/2, bottom_cap_vertices);

  std::vector<MutableConvexPolygon<>> result =
    Extrude(bottom_cap_vertices, /*direction=*/Vector3(0, 0, 1),
            /*direction_denom=*/BigInt(1));

  std::vector<walnut::HomoPoint3> top_cap_vertices{
    walnut::HomoPoint3(0, 0, 1, 1),
    walnut::HomoPoint3(1, 0, 1, 1),
    walnut::HomoPoint3(1, 1, 1, 1),
  };

  MutableConvexPolygon<> top_cap(/*plane=*/HalfSpace3(0, 0, 1, 1),
                                 /*drop_dimension=*/2, top_cap_vertices);

  std::vector<walnut::HomoPoint3> side1_vertices{
    walnut::HomoPoint3(1, 1, 0, 1),
    walnut::HomoPoint3(0, 0, 0, 1),
    walnut::HomoPoint3(0, 0, 1, 1),
    walnut::HomoPoint3(1, 1, 1, 1),
  };

  MutableConvexPolygon<> side1(/*plane=*/HalfSpace3(-1, 1, 0, 0),
                               /*drop_dimension=*/0, side1_vertices);

  std::vector<walnut::HomoPoint3> side2_vertices{
    walnut::HomoPoint3(0, 0, 0, 1),
    walnut::HomoPoint3(1, 0, 0, 1),
    walnut::HomoPoint3(1, 0, 1, 1),
    walnut::HomoPoint3(0, 0, 1, 1),
  };

  MutableConvexPolygon<> side2(/*plane=*/HalfSpace3(0, -1, 0, 0),
                               /*drop_dimension=*/1, side2_vertices);

  std::vector<walnut::HomoPoint3> side3_vertices{
    walnut::HomoPoint3(1, 0, 0, 1),
    walnut::HomoPoint3(1, 1, 0, 1),
    walnut::HomoPoint3(1, 1, 1, 1),
    walnut::HomoPoint3(1, 0, 1, 1),
  };

  MutableConvexPolygon<> side3(/*plane=*/HalfSpace3(1, 0, 0, 1),
                               /*drop_dimension=*/0, side3_vertices);

  EXPECT_THAT(result, UnorderedElementsAre(MutableConvexPolygon<>(bottom_cap),
                                           MutableConvexPolygon<>(top_cap),
                                           MutableConvexPolygon<>(side1),
                                           MutableConvexPolygon<>(side2),
                                           MutableConvexPolygon<>(side3)));
}

}  // walnut
