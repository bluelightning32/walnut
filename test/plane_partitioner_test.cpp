#include "walnut/plane_partitioner.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::ElementsAre;
using testing::IsEmpty;
using testing::UnorderedElementsAre;

TEST(PlanePartitioner, EmptyInput) {
  PlanePartitioner<BSPPolygon<>> partitioner;
  std::vector<BSPPolygon<>> input;

  CollectorVisitor<BSPPolygon<>, PolygonFilter> visitor(PolygonFilter(0));

  std::vector<BSPContentInfo> content_info;
  partitioner.Run(input.begin(), input.end(), /*drop_dimension=*/2,
                  /*pos_normal=*/true, content_info, visitor);

  EXPECT_THAT(visitor.polygons(), IsEmpty());
}

BSPPolygon<> MakeRectangle(BSPContentId id, int minx, int miny, int maxx,
                           int maxy) {
  std::vector<Point3> p = {
    Point3(minx, miny, 10),
    Point3(maxx, miny, 10),
    Point3(maxx, maxy, 10),
    Point3(minx, maxy, 10),
  };
  const static HalfSpace3 plane(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10);
  BSPPolygon<> result(id, /*on_node_plane=*/&plane, /*pos_side=*/false,
                      /*parent=*/ConvexPolygon<>(plane, /*drop_dimension=*/2,
                                                 p));
  result.SetBoundaryAngles(/*coincident_info=*/SplitSide{/*split=*/&plane,
                                                         /*pos_side=*/false},
                           /*coincident_begin=*/0,
                           /*coincident_end=*/result.vertex_count());
  return result;
}

TEST(PlanePartitioner, AcceptSinglePolygon) {
  PlanePartitioner<BSPPolygon<>> partitioner;
  std::vector<BSPPolygon<>> input{
    MakeRectangle(/*id=*/0, /*minx=*/0, /*miny=*/0, /*maxx=*/2, /*maxy=*/2),
  };

  CollectorVisitor<BSPPolygon<>, PolygonFilter> visitor(PolygonFilter(0));

  std::vector<BSPContentInfo> content_info{
    BSPContentInfo{/*has_interior_polygons=*/false,
                   /*has_border_polygons=*/true, /*pwn=*/0},
  };
  partitioner.Run(input.begin(), input.end(), /*drop_dimension=*/2,
                  /*pos_normal=*/true, content_info, visitor);

  ASSERT_THAT(visitor.polygons(), ElementsAre(input[0]));
  for (const BSPPolygon<>::EdgeRep& edge : visitor.polygons()[0].edges()) {
    EXPECT_NE(edge.edge_first_coincident.split, nullptr);
  }
}

TEST(PlanePartitioner, SplitAlongAllPolygons) {
  PlanePartitioner<BSPPolygon<>> partitioner;
  //
  // The first two inputs form a square that is split vertically.
  // +----------+----------+
  // |          |          |
  // |          |          |
  // |          |          |
  // | input[0] | input[1] |
  // |          |          |
  // |          |          |
  // |          |          |
  // +----------+----------+
  //
  // The next two inputs form a square that is split horizontally. Both squares
  // cover the same area.
  // +---------------------+
  // |                     |
  // |    input[3]         |
  // |                     |
  // +---------------------+
  // |                     |
  // |    input[2]         |
  // |                     |
  // +---------------------+
  //
  // The result should be a square divided into 4 subsquares. In the input,
  // each subsquare is covered twice, but the filter ensures that only one
  // subsquare is outputted for each area.
  std::vector<BSPPolygon<>> input{
    MakeRectangle(/*id=*/0, /*minx=*/0, /*miny=*/0, /*maxx=*/1, /*maxy=*/2),
    MakeRectangle(/*id=*/0, /*minx=*/1, /*miny=*/0, /*maxx=*/2, /*maxy=*/2),
    MakeRectangle(/*id=*/0, /*minx=*/0, /*miny=*/0, /*maxx=*/2, /*maxy=*/1),
    MakeRectangle(/*id=*/0, /*minx=*/0, /*miny=*/1, /*maxx=*/2, /*maxy=*/2),
  };

  CollectorVisitor<BSPPolygon<>, PolygonFilter> visitor(PolygonFilter(0));

  std::vector<BSPContentInfo> content_info{
    BSPContentInfo{/*has_interior_polygons=*/false,
                   /*has_border_polygons=*/true, /*pwn=*/0},
  };
  partitioner.Run(input.begin(), input.end(), /*drop_dimension=*/2,
                  /*pos_normal=*/true, content_info, visitor);

  EXPECT_THAT(visitor.polygons(),
              UnorderedElementsAre(MakeRectangle(/*id=*/0, /*minx=*/0,
                                                 /*miny=*/0, /*maxx=*/1,
                                                 /*maxy=*/1),
                                   MakeRectangle(/*id=*/0, /*minx=*/1,
                                                 /*miny=*/0, /*maxx=*/2,
                                                 /*maxy=*/1),
                                   MakeRectangle(/*id=*/0, /*minx=*/0,
                                                 /*miny=*/1, /*maxx=*/1,
                                                 /*maxy=*/2),
                                   MakeRectangle(/*id=*/0, /*minx=*/1,
                                                 /*miny=*/1, /*maxx=*/2,
                                                 /*maxy=*/2)));
  for (const BSPPolygon<>& polygon : visitor.polygons()) {
    for (const BSPPolygon<>::EdgeRep& edge : polygon.edges()) {
      EXPECT_NE(edge.edge_first_coincident.split, nullptr);
    }
  }
}

// Only the second input polygon has nullptr edge_first_coincident values.
TEST(PlanePartitioner, SplitOnlyFromSecondPolygon) {
  PlanePartitioner<BSPPolygon<>> partitioner;
  std::vector<BSPPolygon<>> input{
    MakeRectangle(/*id=*/0, /*minx=*/0, /*miny=*/0, /*maxx=*/2, /*maxy=*/2),
  };

  CollectorVisitor<BSPPolygon<>, PolygonFilter> visitor1(PolygonFilter(0));

  std::vector<BSPContentInfo> content_info{
    BSPContentInfo{/*has_interior_polygons=*/false,
                   /*has_border_polygons=*/true, /*pwn=*/0},
  };
  partitioner.Run(input.begin(), input.end(), /*drop_dimension=*/2,
                  /*pos_normal=*/true, content_info, visitor1);

  ASSERT_THAT(visitor1.polygons(), ElementsAre(input[0]));

  const BSPPolygon<>& visitor1_polygon = visitor1.polygons()[0];
  std::vector<BSPPolygon<>> input2{
    visitor1_polygon,
    MakeRectangle(/*id=*/1, /*minx=*/2, /*miny=*/2, /*maxx=*/4, /*maxy=*/4),
  };

  std::vector<BSPContentInfo> content_info2{
    BSPContentInfo{/*has_interior_polygons=*/false,
                   /*has_border_polygons=*/true, /*pwn=*/0},
    BSPContentInfo{/*has_interior_polygons=*/false,
                   /*has_border_polygons=*/true, /*pwn=*/0},
  };
  CollectorVisitor<BSPPolygon<>, PolygonFilter> visitor2(PolygonFilter(1));
  partitioner.Run(input2.begin(), input2.end(), /*drop_dimension=*/2,
                  /*pos_normal=*/true, content_info2, visitor2);

  EXPECT_EQ(visitor2.polygons().size(), 1);
  for (const BSPPolygon<>& polygon : visitor2.polygons()) {
    for (const BSPPolygon<>::EdgeRep& edge : polygon.edges()) {
      EXPECT_NE(edge.edge_first_coincident.split, nullptr);
    }
  }
}

// Verify that the edge_first_coincident is not changed if it is already
// non-null.
TEST(PlanePartitioner, NoResplit) {
  PlanePartitioner<BSPPolygon<>> partitioner;
  std::vector<BSPPolygon<>> input{
    MakeRectangle(/*id=*/0, /*minx=*/0, /*miny=*/0, /*maxx=*/2, /*maxy=*/2),
  };

  CollectorVisitor<BSPPolygon<>, PolygonFilter> visitor1(PolygonFilter(0));

  std::vector<BSPContentInfo> content_info{
    BSPContentInfo{/*has_interior_polygons=*/false,
                   /*has_border_polygons=*/true, /*pwn=*/0},
  };
  partitioner.Run(input.begin(), input.end(), /*drop_dimension=*/2,
                  /*pos_normal=*/true, content_info, visitor1);

  ASSERT_THAT(visitor1.polygons(), ElementsAre(input[0]));

  CollectorVisitor<BSPPolygon<>, PolygonFilter> visitor2(PolygonFilter(0));
  partitioner.Run(visitor1.polygons().begin(), visitor1.polygons().end(),
                  /*drop_dimension=*/2, /*pos_normal=*/true, content_info,
                  visitor2);

  ASSERT_EQ(visitor2.polygons().size(), 1);
  const BSPPolygon<>& visitor1_polygon = visitor1.polygons()[0];
  const BSPPolygon<>& visitor2_polygon = visitor2.polygons()[0];
  ASSERT_EQ(visitor1_polygon, visitor2_polygon);
  for (size_t i = 0; i < visitor1_polygon.vertex_count(); ++i) {
    EXPECT_EQ(visitor2_polygon.edge(i).edge_first_coincident.split,
              visitor1_polygon.edge(i).edge_first_coincident.split);
  }
}

TEST(PlanePartitioner, IntersectionFilter) {
  PlanePartitioner<BSPPolygon<>> partitioner;
  //
  // The two input rectangles intersect at a common square.
  // +----------+
  // | input[0] |
  // |          |
  // |          |
  // +----------+----------+
  // | input[0] |          |
  // | &&       | input[1] |
  // | input[1] |          |
  // +----------+----------+
  std::vector<BSPPolygon<>> input{
    MakeRectangle(/*id=*/0, /*minx=*/0, /*miny=*/0, /*maxx=*/1, /*maxy=*/2),
    MakeRectangle(/*id=*/1, /*minx=*/0, /*miny=*/0, /*maxx=*/2, /*maxy=*/1),
  };

  auto filter = MakeIntersectionFilter(PolygonFilter(0), PolygonFilter(1));
  CollectorVisitor<BSPPolygon<>, decltype(filter)> visitor(filter);

  std::vector<BSPContentInfo> content_info{
    BSPContentInfo{/*has_interior_polygons=*/false,
                   /*has_border_polygons=*/true, /*pwn=*/0},
    BSPContentInfo{/*has_interior_polygons=*/false,
                   /*has_border_polygons=*/true, /*pwn=*/0},
  };
  partitioner.Run(input.begin(), input.end(), /*drop_dimension=*/2,
                  /*pos_normal=*/true, content_info, visitor);

  // Either polygon with either id is allowed to be in the output. The
  // following check works because the equality check doesn't actually look at
  // the id.
  EXPECT_THAT(visitor.polygons(),
              UnorderedElementsAre(MakeRectangle(/*id=*/0, /*minx=*/0,
                                                 /*miny=*/0, /*maxx=*/1,
                                                 /*maxy=*/1)));
}

TEST(PlanePartitioner, UnionFilter) {
  PlanePartitioner<BSPPolygon<>> partitioner;
  //
  // The two input rectangles intersect at a common square.
  // +----------+
  // | input[0] |
  // |          |
  // |          |
  // +----------+----------+
  // | input[0] |          |
  // | &&       | input[1] |
  // | input[1] |          |
  // +----------+----------+
  std::vector<BSPPolygon<>> input{
    MakeRectangle(/*id=*/0, /*minx=*/0, /*miny=*/0, /*maxx=*/1, /*maxy=*/2),
    MakeRectangle(/*id=*/1, /*minx=*/0, /*miny=*/0, /*maxx=*/2, /*maxy=*/1),
  };

  auto filter = MakeUnionFilter(PolygonFilter(0), PolygonFilter(1));
  CollectorVisitor<BSPPolygon<>, decltype(filter)> visitor(filter);

  std::vector<BSPContentInfo> content_info{
    BSPContentInfo{/*has_interior_polygons=*/false,
                   /*has_border_polygons=*/true, /*pwn=*/0},
    BSPContentInfo{/*has_interior_polygons=*/false,
                   /*has_border_polygons=*/true, /*pwn=*/0},
  };
  partitioner.Run(input.begin(), input.end(), /*drop_dimension=*/2,
                  /*pos_normal=*/true, content_info, visitor);

  // The following check works because the equality check doesn't actually look
  // at the id, just the vertices.
  EXPECT_THAT(visitor.polygons(),
              UnorderedElementsAre(MakeRectangle(/*id=*/0, /*minx=*/0,
                                                 /*miny=*/0, /*maxx=*/1,
                                                 /*maxy=*/1),
                                   MakeRectangle(/*id=*/0, /*minx=*/1,
                                                 /*miny=*/0, /*maxx=*/2,
                                                 /*maxy=*/1),
                                   MakeRectangle(/*id=*/0, /*minx=*/0,
                                                 /*miny=*/1, /*maxx=*/1,
                                                 /*maxy=*/2)));
}

}  // walnut
