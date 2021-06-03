// Include gtest_prod first so that connected_polygon.h uses FRIEND_TEST
#include "gtest/gtest_prod.h"
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

ConvexPolygon<> MakeRectangle(int min_x, int min_y,
                              int max_x, int max_y) {
  // p[3] <----- p[2]
  //  |           ^
  //  v           |
  // p[0] -----> p[1]

  std::vector<Point3> p = {
    Point3(min_x, min_y, 10),
    Point3(max_x, min_y, 10),
    Point3(max_x, max_y, 10),
    Point3(min_x, max_y, 10),
  };
  HalfSpace3 plane(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10);
  // The ConvexPolygon constructor makes the last vertex the first. So rotate
  // the input vertices first to compensate.
  std::rotate(p.begin(), p.begin() + 1, p.end());
  return ConvexPolygon<>(plane, /*drop_dimension=*/2, p);
}

TEST(ConnectedPolygon, ConstructOnTopAABB) {
  ConnectedPolygon<AABBConvexPolygon<>> polygon(MakeRectangle(/*min_x=*/0,
                                                              /*min_y=*/0,
                                                              /*max_x=*/2,
                                                              /*max_y=*/1));
  EXPECT_TRUE(polygon.IsValidState());
  EXPECT_EQ(polygon.aabb(),
            AABBConvexPolygon<>(MakeRectangle(/*min_x=*/0,
                                              /*min_y=*/0,
                                              /*max_x=*/2,
                                              /*max_y=*/1)).aabb());
}

TEST(ConnectedPolygon, ConstructUnderneathAABB) {
  AABBConvexPolygon<ConnectedPolygon<>> polygon(MakeRectangle(/*min_x=*/0,
                                                              /*min_y=*/0, /*max_x=*/2, /*max_y=*/1));
  EXPECT_TRUE(polygon.IsValidState());
  EXPECT_EQ(polygon.aabb(), AABBConvexPolygon<>(MakeRectangle(/*min_x=*/0,
                                                              /*min_y=*/0, /*max_x=*/2, /*max_y=*/1)).aabb());
}

TEST(ConnectedPolygon, CopyConstruct) {
  ConnectedPolygon<> polygon(MakeRectangle(/*min_x=*/0, /*min_y=*/0,
                                           /*max_x=*/2, /*max_y=*/1));

  ConnectedPolygon<> polygon2(polygon);
  ASSERT_TRUE(polygon2.IsValidState());
  EXPECT_EQ(polygon2, polygon);
}

TEST(ConnectedPolygon, VectorInitializerList) {
  std::vector<ConnectedPolygon<>> v{
    MakeRectangle(/*min_x=*/0, /*min_y=*/0, /*max_x=*/2, /*max_y=*/1),
    MakeRectangle(/*min_x=*/0, /*min_y=*/0, /*max_x=*/2, /*max_y=*/1),
  };
  EXPECT_EQ(v[0], v[1]);
}

TEST(ConnectedPolygon, MoveConstruct) {
  ConnectedPolygon<> polygon(MakeRectangle(/*min_x=*/0, /*min_y=*/0,
                                           /*max_x=*/2, /*max_y=*/1));

  ConnectedPolygon<> polygon2(polygon);
  ConnectedPolygon<> polygon3(std::move(polygon));
  ASSERT_TRUE(polygon3.IsValidState());
  EXPECT_EQ(polygon3, polygon2);

  EXPECT_TRUE(std::is_nothrow_move_constructible<ConnectedPolygon<>>::value);

  EXPECT_TRUE(std::is_move_constructible<ConnectedPolygon<>>::value);

  EXPECT_TRUE((
      std::is_nothrow_constructible<
        ConnectedPolygon<>, RValueKey<ConnectedPolygon<>>
      >::value));
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

  ConnectedPolygon<> polygon(MakeRectangle(/*min_x=*/0, /*min_y=*/0,
                                           /*max_x=*/2, /*max_y=*/1));

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
  AABBConvexPolygon<ConnectedPolygon<>> polygon(MakeRectangle(/*min_x=*/0,
                                                              /*min_y=*/0,
                                                              /*max_x=*/2,
                                                              /*max_y=*/1));

  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    EXPECT_EQ(&polygon.edge(i).polygon(), &polygon);
    EXPECT_EQ(polygon.edge(i).edge_index(), i);
  }
}

TEST(ConnectedPolygon, SplitEdge) {
  ConnectedPolygon<> polygon(MakeRectangle(/*min_x=*/0, /*min_y=*/0,
                                           /*max_x=*/2, /*max_y=*/1));
  polygon.edge(0).partner_ = &polygon.edge(1);
  polygon.edge(1).partner_ = &polygon.edge(0);

  HomoPoint3 p0 = polygon.vertex(0);
  HomoPoint3 p1 = polygon.vertex(1);

  HomoPoint3 mid_point(p0.x()*p1.w() + p1.x()*p0.w(),
                       p0.y()*p1.w() + p1.y()*p0.w(),
                       p0.z()*p1.w() + p1.z()*p0.w(), p0.w()*p1.w()*2);

  using EdgeRep = AssignableWrapper<ConnectedPolygon<>::EdgeRep>;
  EXPECT_TRUE((std::is_nothrow_constructible<
                 EdgeInfoRoot,
                 RValueKey<EdgeInfoRoot>
               >::value));
  EXPECT_TRUE((std::is_nothrow_constructible<
                 ConnectedPolygon<>::EdgeRep::Parent,
                 RValueKey<ConnectedPolygon<>::EdgeRep::Parent>
               >::value));
  EXPECT_TRUE((std::is_nothrow_constructible<
                 ConnectedPolygon<>::EdgeRep,
                 RValueKey<ConnectedPolygon<>::EdgeRep>
               >::value));
  EXPECT_TRUE(std::is_nothrow_move_constructible<EdgeRep>::value);

  EXPECT_TRUE((std::is_nothrow_assignable<
                 EdgeInfoRoot,
                 RValueKey<EdgeInfoRoot>
               >::value));
  EXPECT_TRUE((std::is_nothrow_move_assignable<
                 AssignableWrapper<ConnectedPolygon<>::EdgeRep::Parent>
               >::value));
  EXPECT_TRUE((std::is_nothrow_move_assignable<
                 AssignableWrapper<ConnectedPolygon<>::EdgeRep>
               >::value));
  EXPECT_TRUE(std::is_nothrow_move_assignable<EdgeRep>::value);

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
  ConnectedPolygon<> polygon(MakeRectangle(/*min_x=*/0, /*min_y=*/0,
                                           /*max_x=*/2, /*max_y=*/1));

  using EdgeRep = AssignableWrapper<ConnectedPolygon<>::EdgeRep>;
  EdgeRep e1(polygon.edge(0));
  EdgeRep e2(polygon.edge(1));

  e1.partner_ = &e2;
  e2.partner_ = &e1;

  EdgeRep e1_move(std::move(e1));
  EdgeRep e2_move(std::move(e2));

  EXPECT_EQ(e1_move.partner_, &e2_move) << "e1.partner=" << e1.partner_;
  EXPECT_EQ(e2_move.partner_, &e1_move);

  EXPECT_EQ(&e1_move.polygon(), &polygon);
  EXPECT_EQ(&e2_move.polygon(), &polygon);
}

// Validates that the partner links are updated when a ConnectedEdge is move
// assigned to a new location.
TEST(ConnectedEdge, MoveAssign) {
  ConnectedPolygon<> polygon(MakeRectangle(/*min_x=*/0, /*min_y=*/0,
                                           /*max_x=*/2, /*max_y=*/1));

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

TEST(ConnectedPolygon, Merge) {
  // p[3] <- p[2] p[3] <- p[2]
  //  |       ^    |       ^
  //  v  r2a  |    v  r2b  |
  // p[0] -> p[1] p[0] -> p[1]
  //
  // p[3] <- p[2] p[3] <- p[2]
  //  |       ^    |       ^
  //  v  r1a  |    v  r1b  |
  // p[0] -> p[1] p[0] -> p[1]
  ConnectedPolygon<> r1a(MakeRectangle(/*min_x=*/0, /*min_y=*/0,
                                       /*max_x=*/1, /*max_y=*/1));
  ConnectedPolygon<> r1b(MakeRectangle(/*min_x=*/1, /*min_y=*/0,
                                       /*max_x=*/2, /*max_y=*/1));
  ConnectedPolygon<> r2a(MakeRectangle(/*min_x=*/0, /*min_y=*/1,
                                       /*max_x=*/1, /*max_y=*/2));
  ConnectedPolygon<> r2b(MakeRectangle(/*min_x=*/1, /*min_y=*/1,
                                       /*max_x=*/2, /*max_y=*/2));

  EXPECT_EQ(r1a.vertex(0).x(), 0);
  EXPECT_EQ(r1a.vertex(0).y(), 0);
  r1a.edge(1).partner_ = &r1b.edge(3);
  r1b.edge(3).partner_ = &r1a.edge(1);
  r2a.edge(1).partner_ = &r2b.edge(3);
  r2b.edge(3).partner_ = &r2a.edge(1);

  r1a.edge(2).partner_ = &r2a.edge(0);
  r2a.edge(0).partner_ = &r1a.edge(2);
  r1b.edge(2).partner_ = &r2b.edge(0);
  r2b.edge(0).partner_ = &r1b.edge(2);

  EXPECT_TRUE(r1a.TryMergePolygon(/*nonzero_edge_dimension=*/1,
                                  /*my_edge_index=*/1, /*other=*/r1b,
                                  /*other_edge_index=*/3));
  // p[3] <- p[2] p[3] <- p[2]
  //  |       ^    |       ^
  //  v  r2a  |    v  r2b  |
  // p[0] -> p[1] p[0] -> p[1]
  //
  // p[4] <--- p[3] <---- p[2]
  //  |                    ^
  //  v  r1a               |
  // p[0] --------------> p[1]
  //
  // r1b is emptied out since it was the other polygon in a merge.
  EXPECT_EQ(r1b.vertex_count(), 0);
  // The top edges cannot be merged together yet, because they refer to
  // different partner polygons.
  ASSERT_EQ(r1a.vertex_count(), 5);
  EXPECT_EQ(r1a.edge(0).partner(), nullptr);
  EXPECT_EQ(r1a.edge(1).partner(), nullptr);
  EXPECT_EQ(r1a.edge(2).partner(), &r2b.edge(0));
  EXPECT_EQ(r1a.edge(3).partner(), &r2a.edge(0));
  EXPECT_EQ(r1a.edge(4).partner(), nullptr);

  EXPECT_TRUE(r2a.TryMergePolygon(/*nonzero_edge_dimension=*/1,
                                  /*my_edge_index=*/1, /*other=*/r2b,
                                  /*other_edge_index=*/3));
  // p[3] <-------------- p[2]
  //  |                    ^
  //  v  r2a               |
  // p[0] --------------> p[1]
  //
  // p[3] <-------------- p[2]
  //  |                    ^
  //  v  r1a               |
  // p[0] --------------> p[1]
  //
  // r2b is emptied out since it was the other polygon in a merge.
  EXPECT_EQ(r2b.vertex_count(), 0);
  // The bottom edges can be merged because they both refer to r1a.
  ASSERT_EQ(r2a.vertex_count(), 4);
  // Merging the bottom edges of r2b triggered merging the top edges of r1a.
  ASSERT_EQ(r1a.vertex_count(), 4);
  EXPECT_EQ(r1a.edge(0).partner(), nullptr);
  EXPECT_EQ(r1a.edge(1).partner(), nullptr);
  EXPECT_EQ(r1a.edge(2).partner(), &r2a.edge(0));
  EXPECT_EQ(r1a.edge(3).partner(), nullptr);

  EXPECT_EQ(r2a.edge(0).partner(), &r1a.edge(2));
  EXPECT_EQ(r2a.edge(1).partner(), nullptr);
  EXPECT_EQ(r2a.edge(2).partner(), nullptr);
  EXPECT_EQ(r2a.edge(3).partner(), nullptr);
}

}  // walnut
