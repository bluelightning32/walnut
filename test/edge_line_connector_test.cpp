#include "walnut/edge_line_connector.h"

// For std::shuffle
#include <algorithm>
#include <cmath>
// For std::mt19937
#include <random>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::ElementsAre;

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

// Returns a triangle given 2 points of one edge.
//
// A 0 z component is added to p1 and p2. A non-zero z component is given to
// p3.
ConnectedPolygon<> MakeTriangle(const HomoPoint2& p1, const HomoPoint2& p2) {
  using HomoPoint3Rep = ConnectedPolygon<>::HomoPoint3Rep;
  std::vector<HomoPoint3Rep> p = {
    // The ConvexPolygon constructor shifts the edges by one as part of
    // calculating the edge direction. The last vertex in the list becomes the
    // first in the polygon. So start with the second vertex to compensate.
    HomoPoint3Rep(p2.x(), p2.y(), BigIntImpl(0), p2.w()),
    HomoPoint3Rep(p2.x(), p2.y(), BigIntImpl(1), p2.w()),
    HomoPoint3Rep(p1.x(), p1.y(), BigIntImpl(0), p1.w()),
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
  connector.Connect(connect_edges.begin(), connect_edges.end(),
                    /*sorted_dimension=*/0, error_handler);
  EXPECT_FALSE(errored);
  EXPECT_EQ(t1.edge(0).extra_partner_count(), 0);
  EXPECT_EQ(t1.edge(0).partner(), &t2.edge(0));
  EXPECT_EQ(t2.edge(0).extra_partner_count(), 0);
  EXPECT_EQ(t2.edge(0).partner(), &t1.edge(0));
}

TEST(EdgeLineConnector, SplitPairAdjacentFlat) {
  //
  //     p <- t1(+)    |
  //      \            |
  //     ^ \  ^        |
  //     |  \ |        |
  //     |   \         |
  // t2(-) <- q        |
  //
  int p = 1;
  int q = 3;
  int r = 7;
  ConnectedPolygon<> t1 = MakeTriangle(/*start=*/p, /*end=*/r, /*extra=*/2,
                                       /*angle=*/0);
  ConnectedPolygon<> t2_a = MakeTriangle(/*start=*/r, /*end=*/q, /*extra=*/2,
                                         /*angle=*/pi);
  ConnectedPolygon<> t2_b = MakeTriangle(/*start=*/q, /*end=*/p, /*extra=*/2,
                                         /*angle=*/pi);

  using EdgeVector =
    std::vector<std::reference_wrapper<ConnectedPolygon<>::EdgeRep>>;
  EdgeVector connect_edges{
    t1.edge(0),
    t2_b.edge(0),
    t2_a.edge(0),
  };
  EdgeLineConnector<> connector;
  bool errored = false;
  auto error_handler = [&errored](const std::string& message) {
    std::cout << message << std::endl;
    errored = true;
  };
  connector.Connect(connect_edges.begin(), connect_edges.end(),
                    /*sorted_dimension=*/0, error_handler);
  EXPECT_FALSE(errored);
  using ExtraConnection = ConnectedPolygon<>::EdgeRep::ExtraConnection;
  EXPECT_EQ(t1.edge(0).partner(), &t2_b.edge(0));
  EXPECT_THAT(t1.edge(0).extra_partners(),
              ElementsAre(ExtraConnection(t2_b.vertex(0), &t2_a.edge(0))));

  EXPECT_EQ(t2_a.edge(0).extra_partner_count(), 0);
  EXPECT_EQ(t2_a.edge(0).partner(), &t1.edge(0));

  EXPECT_EQ(t2_b.edge(0).extra_partner_count(), 0);
  EXPECT_EQ(t2_b.edge(0).partner(), &t1.edge(0));
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
  connector.Connect(connect_edges.begin(), connect_edges.end(),
                    /*sorted_dimension=*/0, error_handler);
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

TEST(EdgeLineConnector, TwoCoplanarPolygons) {
  //
  // Top view of the triangles. The arrows show the polygon normals.
  //
  // <- t4(+)  t3(-)->&<-t2(+)    t1(-) ->
  //  3*pi/4        pi/2         pi/4
  //       \          |          /
  //        \         |         /
  //         \        |        /
  //          \       |       /
  //           \      |      /
  //            \     |     /
  //             \    |    /
  //              \   |   /
  //               \  |  /
  //                \ | /
  //                 \|/
  int p = 1;
  int q = 7;
  ConnectedPolygon<> t1 = MakeTriangle(/*start=*/q, /*end=*/p, /*extra=*/1,
                                       /*angle=*/pi/4);
  ConnectedPolygon<> t2 = MakeTriangle(/*start=*/p, /*end=*/q, /*extra=*/2,
                                       /*angle=*/pi/2);
  ConnectedPolygon<> t3 = MakeTriangle(/*start=*/q, /*end=*/p, /*extra=*/3,
                                       /*angle=*/pi/2);
  ConnectedPolygon<> t4 = MakeTriangle(/*start=*/p, /*end=*/q, /*extra=*/4,
                                       /*angle=*/3*pi/4);

  using EdgeVector =
    std::vector<std::reference_wrapper<ConnectedPolygon<>::EdgeRep>>;
  EdgeVector connect_edges{
    t1.edge(0),
    t2.edge(0),
    t3.edge(0),
    t4.edge(0),
  };
  EdgeLineConnector<> connector;
  bool errored = false;
  auto error_handler = [&errored](const std::string& message) {
    std::cout << message << std::endl;
    errored = true;
  };
  connector.Connect(connect_edges.begin(), connect_edges.end(),
                    /*sorted_dimension=*/0, error_handler);
  EXPECT_FALSE(errored);
  EXPECT_EQ(t1.edge(0).partner(), &t2.edge(0));
  EXPECT_EQ(t1.edge(0).extra_partner_count(), 0);

  EXPECT_EQ(t2.edge(0).partner(), &t1.edge(0));
  EXPECT_EQ(t2.edge(0).extra_partner_count(), 0);

  EXPECT_EQ(t3.edge(0).partner(), &t4.edge(0));
  EXPECT_EQ(t3.edge(0).extra_partner_count(), 0);

  EXPECT_EQ(t4.edge(0).partner(), &t3.edge(0));
  EXPECT_EQ(t4.edge(0).extra_partner_count(), 0);
}

TEST(EdgeLineConnector, SplitCoplanarPolygons) {
  // This test has up to 2 coplanar polygons at any point on the edge. However,
  // the coplanar polygons are interrupted in the middle at different points.
  //
  // Top view of the triangles. The arrows show the polygon normals.
  //
  // <- t4(+)  t3(-)->&<-t2(+)    t1(-) ->
  //  3*pi/4        pi/2         pi/4
  //       \          |          /
  //        \         |         /
  //         \        |        /
  //          \       |       /
  //           \      |      /
  //            \     |     /
  //             \    |    /
  //              \   |   /
  //               \  |  /
  //                \ | /
  //                 \|/
  int p = 1;
  int q = 2;
  int r = 5;
  int s = 7;
  ConnectedPolygon<> t1 = MakeTriangle(/*start=*/s, /*end=*/p, /*extra=*/1,
                                       /*angle=*/pi/4);
  ConnectedPolygon<> t2_a = MakeTriangle(/*start=*/p, /*end=*/q, /*extra=*/2,
                                         /*angle=*/pi/2);
  ConnectedPolygon<> t2_b = MakeTriangle(/*start=*/q, /*end=*/s, /*extra=*/2,
                                         /*angle=*/pi/2);
  ConnectedPolygon<> t3_a = MakeTriangle(/*start=*/s, /*end=*/r, /*extra=*/3,
                                         /*angle=*/pi/2);
  ConnectedPolygon<> t3_b = MakeTriangle(/*start=*/r, /*end=*/p, /*extra=*/3,
                                         /*angle=*/pi/2);
  ConnectedPolygon<> t4 = MakeTriangle(/*start=*/p, /*end=*/s, /*extra=*/4,
                                       /*angle=*/3*pi/4);

  using EdgeVector =
    std::vector<std::reference_wrapper<ConnectedPolygon<>::EdgeRep>>;
  EdgeVector connect_edges{
    t1.edge(0),
    t2_a.edge(0),
    t3_b.edge(0),
    t4.edge(0),
    t2_b.edge(0),
    t3_a.edge(0),
  };
  EdgeLineConnector<> connector;
  bool errored = false;
  auto error_handler = [&errored](const std::string& message) {
    std::cout << message << std::endl;
    errored = true;
  };
  connector.Connect(connect_edges.begin(), connect_edges.end(),
                    /*sorted_dimension=*/0, error_handler);
  EXPECT_FALSE(errored);
  using ExtraConnection = ConnectedPolygon<>::EdgeRep::ExtraConnection;
  EXPECT_EQ(t1.edge(0).partner(), &t2_b.edge(0));
  EXPECT_THAT(t1.edge(0).extra_partners(),
              ElementsAre(ExtraConnection(t2_b.vertex(0), &t2_a.edge(0))));

  EXPECT_EQ(t2_a.edge(0).partner(), &t1.edge(0));
  EXPECT_EQ(t2_a.edge(0).extra_partner_count(), 0);

  EXPECT_EQ(t2_b.edge(0).partner(), &t1.edge(0));
  EXPECT_EQ(t2_b.edge(0).extra_partner_count(), 0);

  EXPECT_EQ(t4.edge(0).partner(), &t3_b.edge(0));
  EXPECT_THAT(t4.edge(0).extra_partners(),
              ElementsAre(ExtraConnection(t3_b.vertex(0), &t3_a.edge(0))));

  EXPECT_EQ(t3_a.edge(0).partner(), &t4.edge(0));
  EXPECT_EQ(t3_a.edge(0).extra_partner_count(), 0);

  EXPECT_EQ(t3_b.edge(0).partner(), &t4.edge(0));
  EXPECT_EQ(t3_b.edge(0).extra_partner_count(), 0);
}

void SortEdges(
    std::vector<std::reference_wrapper<ConnectedPolygon<>::EdgeRep>> &edges) {
  EdgeLineConnector<>::SortEdgesInPlane(edges.begin(), edges.end(),
                                        /*drop_dimension=*/2);

  using LineRep = ConnectedPolygon<>::LineRep;
  const LineRep* prev_line = nullptr;
  std::set<std::array<int, 3>> seen_lines;
  const ConnectedPolygon<>::EdgeRep* prev_edge = nullptr;
  using EdgeRef = std::reference_wrapper<ConnectedPolygon<>::EdgeRep>;
  for (const EdgeRef& edge : edges) {
    int sorted_dimension;
    if (!edge.get().line().d().x().IsZero()) {
      sorted_dimension = 0;
    } else {
      sorted_dimension = 1;
    }
    if (prev_line == nullptr || (edge.get().line() != *prev_line &&
                                 edge.get().line() != -*prev_line)) {
      prev_line = &edge.get().line();
      LineRep reduced;
      if (edge.get().line().d().components()[sorted_dimension] < 0) {
        reduced = -edge.get().line();
      } else {
        reduced = edge.get().line();
      }
      reduced.Reduce();
      auto projected = reduced.Project2D(/*drop_dimension=*/2);
      std::array<int, 3> reduced_array{projected.x().ToInt(),
                                       projected.y().ToInt(),
                                       projected.d().ToInt()};
      auto added = seen_lines.insert(reduced_array);
      ASSERT_TRUE(added.second);
    } else {
      if (prev_edge->GetBeginLocation(sorted_dimension) !=
          edge.get().GetBeginLocation(sorted_dimension)) {
        ASSERT_TRUE(
            EdgeLineConnector<>::IsLocationLessThan(
              prev_edge->GetBeginLocation(sorted_dimension),
              edge.get().GetBeginLocation(sorted_dimension),
              sorted_dimension));
      }
    }
    prev_edge = &edge.get();
  }
}

void ShuffleAndSortEdges(std::vector<ConnectedPolygon<>>& triangles) {
  using EdgeRef = std::reference_wrapper<ConnectedPolygon<>::EdgeRep>;
  std::vector<EdgeRef> edges;
  for (ConnectedPolygon<>& triangle : triangles) {
    ASSERT_TRUE(triangle.IsValidState());
    edges.push_back(triangle.edge(0));
  }
  std::mt19937 g(0);
  std::shuffle(edges.begin(), edges.end(), g);

  SortEdges(edges);
}

TEST(EdgeLineConnector, DifferentDistInMiddle) {
  std::vector<ConnectedPolygon<>> triangles{
    MakeTriangle(HomoPoint2(0, 0, 1), HomoPoint2(-1, 0, 1)),
    MakeTriangle(HomoPoint2(1, 1, 1), HomoPoint2(0, 1, 1)),
    MakeTriangle(HomoPoint2(0, 0, 1), HomoPoint2(1, 0, 1)),
  };

  using EdgeRef = std::reference_wrapper<ConnectedPolygon<>::EdgeRep>;
  std::vector<EdgeRef> edges{
    triangles[0].edge(0),
    triangles[1].edge(0),
    triangles[2].edge(0),
  };
  SortEdges(edges);
}

TEST(EdgeLineConnector, DifferentDirectionInMiddle) {
  std::vector<ConnectedPolygon<>> triangles{
    MakeTriangle(HomoPoint2(0, 0, 1), HomoPoint2(-1, 0, 1)),
    MakeTriangle(HomoPoint2(0, 0, 1), HomoPoint2(1, 1, 1)),
    MakeTriangle(HomoPoint2(0, 0, 1), HomoPoint2(1, 0, 1)),
  };

  using EdgeRef = std::reference_wrapper<ConnectedPolygon<>::EdgeRep>;
  std::vector<EdgeRef> edges{
    triangles[0].edge(0),
    triangles[1].edge(0),
    triangles[2].edge(0),
  };
  SortEdges(edges);
}

TEST(EdgeLineConnector, SortEdgesInPlaneSameDenom) {
  std::vector<ConnectedPolygon<>> triangles;
  for (int p1_x = -1; p1_x <= 1; ++p1_x) {
    for (int p1_y = -1; p1_y <= 1; ++p1_y) {
      HomoPoint2 p1(p1_x, p1_y, 1);
      for (int p2_x = -1; p2_x <= 1; ++p2_x) {
        for (int p2_y = -1; p2_y <= 1; ++p2_y) {
          if (p1_x == p2_x && p1_y == p2_y) {
            continue;
          }
          HomoPoint2 p2(p2_x, p2_y, 1);
          ConnectedPolygon<> triangle = MakeTriangle(p1, p2);
          ASSERT_TRUE(triangle.IsValidState());
          triangles.push_back(std::move(triangle));
        }
      }
    }
  }
  ShuffleAndSortEdges(triangles);
}

TEST(EdgeLineConnector, SortEdgesInPlaneDifferentDenoms) {
  std::vector<ConnectedPolygon<>> triangles;
  for (int p1_d = -2; p1_d <= 2; ++p1_d) {
    if (p1_d == 0) continue;
    for (int p1_x = -1; p1_x <= 1; ++p1_x) {
      for (int p1_y = -1; p1_y <= 1; ++p1_y) {
        HomoPoint2 p1(p1_x, p1_y, p1_d);
        for (int p2_d = -2; p2_d <= 2; ++p2_d) {
          if (p2_d == 0) continue;
          for (int p2_x = -1; p2_x <= 1; ++p2_x) {
            for (int p2_y = -1; p2_y <= 1; ++p2_y) {
              if (p1_x * p2_d == p2_x * p1_d &&
                  p1_y * p2_d == p2_y * p1_d) {
                continue;
              }
              HomoPoint2 p2(p2_x, p2_y, p2_d);
              ConnectedPolygon<> triangle = MakeTriangle(p1, p2);
              ASSERT_TRUE(triangle.IsValidState());
              triangles.push_back(std::move(triangle));
            }
          }
        }
      }
    }
  }
  ShuffleAndSortEdges(triangles);
}

}  // walnut
