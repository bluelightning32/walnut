#include "walnut/connected_polygon.h"

#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/aabb_convex_polygon.h"
#include "walnut/point3.h"

namespace walnut {

TEST(ConnectedPolygon, ConstructEmpty) {
  ConnectedPolygon<> polygon;

  EXPECT_TRUE(polygon.IsValidState());
}

ConvexPolygon<> MakeRectangle() {
  // p[3] <----- p[2]
  //  |           ^
  //  v           |
  // p[0] -----> p[1]

  std::vector<Point3> p = {
    Point3(0, 0, 10),
    Point3(2, 0, 10),
    Point3(2, 1, 10),
    Point3(0, 1, 10),
  };
  ConvexPolygon<>::HalfSpace3Rep plane(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10);
  return ConvexPolygon<>(plane, /*drop_dimension=*/2, p);
}

TEST(ConnectedPolygon, ConstructOnTopAABB) {
  ConnectedPolygon<AABBConvexPolygon<>> polygon(MakeRectangle());
  EXPECT_TRUE(polygon.IsValidState());
  EXPECT_EQ(polygon.aabb(), AABBConvexPolygon<>(MakeRectangle()).aabb());
}

TEST(ConnectedPolygon, ConstructUnderneathAABB) {
  AABBConvexPolygon<ConnectedPolygon<>> polygon(MakeRectangle());
  EXPECT_TRUE(polygon.IsValidState());
  EXPECT_EQ(polygon.aabb(), AABBConvexPolygon<>(MakeRectangle()).aabb());
}

TEST(ConnectedPolygon, CopyConstruct) {
  ConnectedPolygon<> polygon(MakeRectangle());

  ConnectedPolygon<> polygon2(polygon);
  ASSERT_TRUE(polygon2.IsValidState());
  EXPECT_EQ(polygon2, polygon);
}

TEST(ConnectedPolygon, VectorInitializerList) {
  std::vector<ConnectedPolygon<>> v{
    MakeRectangle(),
    MakeRectangle(),
  };
  EXPECT_EQ(v[0], v[1]);
}

TEST(ConnectedPolygon, MoveConstruct) {
  ConnectedPolygon<> polygon(MakeRectangle());

  ConnectedPolygon<> polygon2(polygon);
  ConnectedPolygon<> polygon3(std::move(polygon));
  ASSERT_TRUE(polygon3.IsValidState());
  EXPECT_EQ(polygon3, polygon2);
}

TEST(ConnectedPolygon, SplitInMiddle) {
  //       split
  //         |->
  //         |
  // p[3] <----- p[2]
  //  |      |    ^
  //  v      |    |
  // p[0] -----> p[1]
  //         |

  ConnectedPolygon<> polygon(MakeRectangle());

  HalfSpace3<> split(/*x=*/1, /*y=*/0, /*z=*/0, /*dist=*/1);
  ConnectedPolygon<>::SplitInfoRep split_info = polygon.GetSplitInfo(split);
  EXPECT_TRUE(split_info.IsValid(polygon.vertex_count()));
  EXPECT_TRUE(split_info.ShouldEmitNegativeChild());
  EXPECT_TRUE(split_info.ShouldEmitPositiveChild());

  std::pair<ConnectedPolygon<>, ConnectedPolygon<>> children =
    polygon.CreateSplitChildren(split_info);

  EXPECT_EQ(children.first.vertex_count(), 4);
  EXPECT_EQ(children.second.vertex_count(), 4);
  EXPECT_TRUE(children.first.IsValidState());
  EXPECT_TRUE(children.second.IsValidState());
}

TEST(ConnectedPolygon, edge_index) {
  AABBConvexPolygon<ConnectedPolygon<>> polygon(MakeRectangle());

  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    EXPECT_EQ(&polygon.edge(i).polygon(), &polygon);
    EXPECT_EQ(polygon.edge(i).edge_index(), i);
  }
}

TEST(ConnectedEdge, ReversePartnerList) {
  ConnectedPolygon<> polygon(MakeRectangle());
  using HomoPoint3Rep = ConnectedPolygon<>::HomoPoint3Rep;

  polygon.edge(0).ReversePartnerList();

  polygon.edge(1).partner_ = &polygon.edge(0);
  polygon.edge(1).ReversePartnerList();
  EXPECT_EQ(polygon.edge(1).partner(), &polygon.edge(0));
  EXPECT_EQ(polygon.edge(1).extra_partner_count(), 0);

  polygon.edge(2).partner_ = &polygon.edge(0);
  polygon.edge(2).extra_partners_.emplace_back(HomoPoint3Rep(1, 0, 0, 1),
                                               &polygon.edge(1));
  polygon.edge(2).ReversePartnerList();
  EXPECT_EQ(polygon.edge(2).partner(), &polygon.edge(1));
  ASSERT_EQ(polygon.edge(2).extra_partner_count(), 1);
  EXPECT_EQ(polygon.edge(2).extra_partner(0), &polygon.edge(0));
  EXPECT_EQ(polygon.edge(2).extra_partner_start(0), HomoPoint3Rep(1, 0, 0, 1));

  polygon.edge(3).partner_ = &polygon.edge(0);
  polygon.edge(3).extra_partners_.emplace_back(HomoPoint3Rep(1, 0, 0, 1),
                                               &polygon.edge(1));
  polygon.edge(3).extra_partners_.emplace_back(HomoPoint3Rep(2, 0, 0, 1),
                                               &polygon.edge(2));
  polygon.edge(3).ReversePartnerList();
  EXPECT_EQ(polygon.edge(3).partner(), &polygon.edge(2));
  ASSERT_EQ(polygon.edge(3).extra_partner_count(), 2);
  EXPECT_EQ(polygon.edge(3).extra_partner(0), &polygon.edge(1));
  EXPECT_EQ(polygon.edge(3).extra_partner_start(0), HomoPoint3Rep(2, 0, 0, 1));
  EXPECT_EQ(polygon.edge(3).extra_partner(1), &polygon.edge(0));
  EXPECT_EQ(polygon.edge(3).extra_partner_start(1), HomoPoint3Rep(1, 0, 0, 1));
}

}  // walnut
