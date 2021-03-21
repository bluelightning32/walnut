#include "walnut/edge_line_connector.h"

#include <cmath>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

// Returns a triangle.
//
// l is a line that starts at (0, 0, 0) and goes in the <1, -1, 0> direction
// from that line. The first edge of the returned line segment goes through l.
// Specifically, triangle.v[0] = <start, -start, 0> and
// triangle.v[1] = <end, -end, 0>.
//
// The third vertex of the triangle is the plane that is perpendicular to l at
// <extra, -extra, 0>. That vertex is roughly 1 unit away in `angle` direction.
ConnectedPolygon<> MakeTriangle(int start, int end, int extra, double angle) {
  using HomoPoint3Rep = ConnectedPolygon<>::HomoPoint3Rep;
  std::vector<HomoPoint3Rep> p = {
    // The ConvexPolygon constructor shifts the edges by one as part of
    // calculating the edge direction. The last vertex in the list becomes the
    // first in the polygon. So start with the second vertex to compensate.
    HomoPoint3Rep(end, -end, 0, 1),
    HomoPoint3Rep(10 * (extra + std::cos(angle)),
                  10 * (-extra + std::cos(angle)),
                  10 * std::sin(angle), 10),
    HomoPoint3Rep(start, -start, 0, 1),
  };
  ConnectedPolygon<>::HalfSpace3Rep plane(p[0], p[1], p[2]);
  const int drop_dimension = plane.normal().GetFirstNonzeroDimension();
  ConnectedPolygon<> result(plane, drop_dimension, p);
  assert(result.IsValidState());
  return result;
}

constexpr const double pi = 3.14159265358979323846;

TEST(EdgeLineConnector, PairAdjacentFlat) {
  //
  //  p <- t1   |
  //   \        |
  //  ^ \  ^    |
  //  |  \ |    |
  //  |   \     |
  // t2 <- q    |
  //
  int p = 1;
  int q = 3;
  ConnectedPolygon<> t1 = MakeTriangle(/*start=*/p, /*end=*/q, /*extra=*/2,
                                       /*angle=*/0);
  ConnectedPolygon<> t2 = MakeTriangle(/*start=*/q, /*end=*/p, /*extra=*/2,
                                       /*angle=*/pi);

  using EdgeVector =
    std::vector<std::reference_wrapper<ConnectedPolygon<>::EdgeRep>>;
  EdgeVector connect_edges{
    t1.edge(0),
    t2.edge(0),
  };
  EdgeLineConnector<> connector;
  bool errored = false;
  auto error_handler = [&errored](const std::string& message) {
    std::cout << message << std::endl;
    errored = true;
  };
  connector(connect_edges.begin(), connect_edges.end(), /*sorted_dimension=*/0,
            error_handler);
  EXPECT_FALSE(errored);
  EXPECT_EQ(t1.edge(0).extra_partner_count(), 0);
  EXPECT_EQ(t1.edge(0).partner(), &t2.edge(0));
  EXPECT_EQ(t2.edge(0).extra_partner_count(), 0);
  EXPECT_EQ(t2.edge(0).partner(), &t1.edge(0));
}

TEST(EdgeLineConnector, EightShareEdge) {
  // 8 triangles share an edge.
  //
  // Top view of the triangles. The arrows after each triangle represent the
  // triangle normals. The inside-to-inside strategy that the edge connector
  // uses will search for the nearest triangle in the opposite direction of the
  // normal.
  //
  //          ^   <- t3(+)               |
  //         /t4(-)  |  t2(-)\           |
  //               \ | /      v          |
  //                \|/            ^     |
  //  | t5(+) -------+------ t1(+) |     |
  //  v             /|\                  |
  //         ^     / | \                 |
  //          \t6(-) |  t8(-) /          |
  //               t7(+) ->  v           |
  //
  // When the algorithm internally sorts the triangles by their normals, the
  // normals of the negative edges are flipped.
  //
  //               t1(+)                 |
  //          t2(-)  |  t8(-)            |
  //               \ | /                 |
  //                \|/                  |
  //    t3(+) -------+------ t7(+)       |
  //                /|\                  |
  //               / | \                 |
  //           t4(-) |  t6(-)            |
  //               t5(+)                 |
  //
  int p = -1;
  int q = 3;
  ConnectedPolygon<> t1 = MakeTriangle(/*start=*/p, /*end=*/q, /*extra=*/1,
                                       /*angle=*/0);
  ConnectedPolygon<> t2 = MakeTriangle(/*start=*/q, /*end=*/p, /*extra=*/1,
                                       /*angle=*/pi/4);
  ConnectedPolygon<> t3 = MakeTriangle(/*start=*/p, /*end=*/q, /*extra=*/2,
                                       /*angle=*/pi/2);
  ConnectedPolygon<> t4 = MakeTriangle(/*start=*/q, /*end=*/p, /*extra=*/2,
                                       /*angle=*/3*pi/4);
  ConnectedPolygon<> t5 = MakeTriangle(/*start=*/p, /*end=*/q, /*extra=*/3,
                                       /*angle=*/pi);
  ConnectedPolygon<> t6 = MakeTriangle(/*start=*/q, /*end=*/p, /*extra=*/3,
                                       /*angle=*/5*pi/4);
  ConnectedPolygon<> t7 = MakeTriangle(/*start=*/p, /*end=*/q, /*extra=*/4,
                                       /*angle=*/3*pi/2);
  ConnectedPolygon<> t8 = MakeTriangle(/*start=*/q, /*end=*/p, /*extra=*/4,
                                       /*angle=*/7*pi/4);

  using EdgeVector =
    std::vector<std::reference_wrapper<ConnectedPolygon<>::EdgeRep>>;
  EdgeVector connect_edges{
    t1.edge(0),
    t2.edge(0),
    t3.edge(0),
    t4.edge(0),
    t5.edge(0),
    t6.edge(0),
    t7.edge(0),
    t8.edge(0),
  };
  EdgeLineConnector<> connector;
  bool errored = false;
  auto error_handler = [&errored](const std::string& message) {
    std::cout << message << std::endl;
    errored = true;
  };
  connector(connect_edges.begin(), connect_edges.end(), /*sorted_dimension=*/0,
            error_handler);
  EXPECT_FALSE(errored);
  EXPECT_EQ(t1.edge(0).partner(), &t8.edge(0));
  EXPECT_EQ(t1.edge(0).extra_partner_count(), 0);

  EXPECT_EQ(t2.edge(0).partner(), &t3.edge(0));
  EXPECT_EQ(t2.edge(0).extra_partner_count(), 0);

  EXPECT_EQ(t3.edge(0).partner(), &t2.edge(0));
  EXPECT_EQ(t3.edge(0).extra_partner_count(), 0);

  EXPECT_EQ(t4.edge(0).partner(), &t5.edge(0));
  EXPECT_EQ(t4.edge(0).extra_partner_count(), 0);

  EXPECT_EQ(t5.edge(0).partner(), &t4.edge(0));
  EXPECT_EQ(t5.edge(0).extra_partner_count(), 0);

  EXPECT_EQ(t6.edge(0).partner(), &t7.edge(0));
  EXPECT_EQ(t6.edge(0).extra_partner_count(), 0);

  EXPECT_EQ(t7.edge(0).partner(), &t6.edge(0));
  EXPECT_EQ(t7.edge(0).extra_partner_count(), 0);

  EXPECT_EQ(t8.edge(0).partner(), &t1.edge(0));
  EXPECT_EQ(t8.edge(0).extra_partner_count(), 0);
}

}  // walnut
