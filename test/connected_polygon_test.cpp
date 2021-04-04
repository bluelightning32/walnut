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
  HalfSpace3 plane(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10);
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

  HalfSpace3 split(/*x=*/1, /*y=*/0, /*z=*/0, /*dist=*/1);
  ConvexPolygonSplitInfo split_info = polygon.GetSplitInfo(split);
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

TEST(ConnectedPolygon, SplitEdge) {
  ConnectedPolygon<> polygon(MakeRectangle());
  polygon.edge(0).partner_ = &polygon.edge(1);
  polygon.edge(1).partner_ = &polygon.edge(0);

  HomoPoint3 p0 = polygon.vertex(0);
  HomoPoint3 p1 = polygon.vertex(1);

  HomoPoint3 mid_point(p0.x()*p1.w() + p1.x()*p0.w(),
                       p0.y()*p1.w() + p1.y()*p0.w(),
                       p0.z()*p1.w() + p1.z()*p0.w(), p0.w()*p1.w()*2);

  using EdgeDeed = Deed<ConnectedPolygon<>::EdgeRep>;
  EdgeDeed e0_deed(&polygon.edge(0));
  EXPECT_EQ(e0_deed.get(), &polygon.edge(0));

  polygon.SplitEdge(/*split_index=*/0, mid_point);
  EXPECT_EQ(polygon.vertex(0), p0);
  EXPECT_EQ(polygon.vertex(1), mid_point);
  EXPECT_EQ(polygon.vertex(2), p1);

  // The deed should stick to the lower index.
  EXPECT_EQ(e0_deed.get(), &polygon.edge(0));

  // The moved edges should still be partnered with each other.
  EXPECT_EQ(polygon.edge(0).partner(), &polygon.edge(2));
  // The new edge should not be partnered.
  EXPECT_EQ(polygon.edge(1).partner(), nullptr);
  EXPECT_EQ(polygon.edge(2).partner(), &polygon.edge(0));

  EXPECT_EQ(&polygon.edge(0).polygon(), &polygon);
  EXPECT_EQ(&polygon.edge(1).polygon(), &polygon);
  EXPECT_EQ(&polygon.edge(2).polygon(), &polygon);
}

// Validates that the partner links are updated when a ConnectedEdge is move
// constructed to a new location.
TEST(ConnectedEdge, MoveConstruct) {
  ConnectedPolygon<> polygon(MakeRectangle());

  using EdgeRep = AssignableWrapper<ConnectedPolygon<>::EdgeRep>;
  EdgeRep e1(polygon.edge(0));
  EdgeRep e2(polygon.edge(1));

  e1.partner_ = &e2;
  e2.partner_ = &e1;

  EdgeRep e1_move(std::move(e1));
  EdgeRep e2_move(std::move(e2));

  EXPECT_EQ(e1_move.partner_, &e2_move);
  EXPECT_EQ(e2_move.partner_, &e1_move);

  EXPECT_EQ(&e1_move.polygon(), &polygon);
  EXPECT_EQ(&e2_move.polygon(), &polygon);
}

// Validates that the partner links are updated when a ConnectedEdge is move
// assigned to a new location.
TEST(ConnectedEdge, MoveAssign) {
  ConnectedPolygon<> polygon(MakeRectangle());

  using EdgeRep = AssignableWrapper<ConnectedPolygon<>::EdgeRep>;
  EdgeRep e1(polygon.edge(0));
  EdgeRep e2(polygon.edge(1));

  e1.partner_ = &e2;
  e2.partner_ = &e1;

  EdgeRep e1_move(polygon.edge(1));
  e1_move = std::move(std::move(e1));
  EdgeRep e2_move(polygon.edge(0));
  e2_move = std::move(std::move(e2));

  EXPECT_EQ(e1_move.partner_, &e2_move);
  EXPECT_EQ(e2_move.partner_, &e1_move);

  EXPECT_EQ(&e1_move.polygon(), &polygon);
  EXPECT_EQ(&e2_move.polygon(), &polygon);

  // Moving the already moved from e1 and e2 should not affect e1_move and
  // e2_move.
  EdgeRep e1_move2(polygon.edge(1));
  e1_move2 = std::move(std::move(e1));
  EdgeRep e2_move2(polygon.edge(0));
  e2_move2 = std::move(std::move(e2));
  EXPECT_EQ(e1_move.partner_, &e2_move);
  EXPECT_EQ(e2_move.partner_, &e1_move);
}

}  // walnut
