#include "walnut/vertex_double_point3_mapper.h"

#include <cmath>
#include <unordered_set>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/edge_line_connector.h"

namespace walnut {

// Returns a vector of equivalent HomoPoint3s that each have a different
// DoublePoint3 representation.
//
// This can create up to 6 equivalent HomoPoint3s. Every coordinate (along with
// the denominator) is greater than 2 in the returned HomoPoint3s.
std::vector<HomoPoint3> MakePointsWithDifferentReps(size_t count) {
  std::vector<HomoPoint3> result;
  if (count == 0) return result;

  HomoPoint3 canonical((BigInt(1) << 53) - 1,
                       (BigInt(3) << 52) - 1,
                       (BigInt(5) << 50) - 1,
                       (BigInt(1) << 51) - 1);

  std::unordered_set<DoublePoint3> found;
  found.insert(canonical.GetDoublePoint3());
  result.push_back(canonical);

  BigInt multiple(7);
  while (result.size() < count) {
    HomoPoint3 newpoint(canonical.x() * multiple, canonical.y() * multiple,
                        canonical.z() * multiple, canonical.w() * multiple);
    if (found.insert(newpoint.GetDoublePoint3()).second) {
      result.push_back(newpoint);
    }
    multiple *= 7;
  }
  return result;
}

// Verifies that it is possible to have 3 equivalent `HomoPoint3`s with
// different double representations.
TEST(VertexDoublePoint3Mapper, ThreeRepsExist) {
  std::vector<HomoPoint3> found = MakePointsWithDifferentReps(3);
  for (const HomoPoint3& p : found) {
    EXPECT_TRUE(std::isfinite(p.GetDoublePoint3().x));
    EXPECT_TRUE(std::isfinite(p.GetDoublePoint3().y));
    EXPECT_TRUE(std::isfinite(p.GetDoublePoint3().z));
  }
}

TEST(VertexDoublePoint3Mapper, MapMerge) {
  // Creates a vertex where 4 triangles meet. The place where they meet has 2
  // different DoublePoint3 representations, labelled p and q in the diagram
  // below.
  //
  //          -|-           |
  //         / | \          |
  //       --  |  --        |
  //     /   e0|    \       |
  //   --      |     - -    |
  //  /       p|p  e1   \   |
  // +---------+---------+  |
  //  \   e3  q|q       /   |
  //   --      |      --    |
  //     \     |e2   /      |
  //      --   |   --       |
  //        \  |  /         |
  //         --|--          |

  std::vector<HomoPoint3> centers = MakePointsWithDifferentReps(2);
  const HomoPoint3& p = centers[0];
  const HomoPoint3& q = centers[1];

  HomoPoint3 left(-p.x(), p.y(), p.z(), p.w());
  HomoPoint3 top(p.x(), p.y() << 1, p.z(), p.w());
  HomoPoint3 right(p.x() << 1, p.y(), p.z(), p.w());
  HomoPoint3 bottom(p.x(), -p.y(), p.z(), p.w());

  HalfSpace3 plane(/*x=*/BigInt{0}, /*y=*/BigInt{0}, /*z=*/p.w(),
                   /*dist=*/p.z());

  // The ConnectedPolygon constructor makes the last vertex the first. So list
  // the center vertex last so that the edge with the center as the source
  // becomes the first edge.
  std::vector<ConnectedPolygon<>> triangles{
    ConnectedPolygon<>(plane, /*drop_dimension=*/2,
                       std::vector<HomoPoint3>{top, left, p}),
    ConnectedPolygon<>(plane, /*drop_dimension=*/2,
                       std::vector<HomoPoint3>{right, top, p}),
    ConnectedPolygon<>(plane, /*drop_dimension=*/2,
                       std::vector<HomoPoint3>{bottom, right, q}),
    ConnectedPolygon<>(plane, /*drop_dimension=*/2,
                       std::vector<HomoPoint3>{left, bottom, q}),
  };

  std::vector<Deed<ConnectedPolygon<>::EdgeRep>> connect_edges;
  for (ConnectedPolygon<>& triangle : triangles) {
    ASSERT_EQ(triangle.vertex_count(), 3);
    connect_edges.push_back(&triangle.edge(0));
    // Edge 1 is the outer edge that has no neighbors. Don't add it to the
    // vector, because the EdgeLineConnector would raise an error when it can't
    // find a partner for the edge.
    connect_edges.push_back(&triangle.edge(2));
  }
  bool errored = false;
  auto error_handler = [&errored](const std::string& message) {
    std::cout << message << std::endl;
    errored = true;
  };
  EdgeLineConnector<> connector;
  connector.ConnectUnsorted(connect_edges.begin(), connect_edges.end(),
                            /*drop_dimension=*/2, error_handler);
  EXPECT_FALSE(errored);

  int merged = 0;
  int created = 0;
  auto value_factory = [&created](const DoublePoint3& p) -> int {
    return ++created;
  };
  auto value_merger = [&merged](RedirectableValue<int>& a,
                                RedirectableValue<int>& b) {
    a.Redirect(b);
    ++merged;
  };
  VertexDoublePoint3Mapper<decltype(value_factory), decltype(value_merger)>
    mapper(value_factory, value_merger);
  mapper.Map(triangles[0].edge(0));
  mapper.Map(triangles[2].edge(0));
  mapper.Map(triangles[1].edge(0));
  mapper.Map(triangles[3].edge(0));

  EXPECT_GT(created, 0);
  EXPECT_GT(merged, 0);

  int primary = 0;
  for (const auto& p : mapper.map) {
    if (p.second.IsPrimary()) ++primary;
  }
  EXPECT_EQ(primary, 1);

  EXPECT_TRUE(mapper.map.find(p.GetDoublePoint3()) != mapper.map.end());
  EXPECT_TRUE(mapper.map.find(q.GetDoublePoint3()) != mapper.map.end());
}

}  // walnut
