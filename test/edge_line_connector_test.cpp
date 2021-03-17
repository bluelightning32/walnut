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

}  // walnut
