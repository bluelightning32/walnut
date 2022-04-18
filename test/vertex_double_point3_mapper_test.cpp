#include "walnut/vertex_double_point3_mapper.h"

#include <cmath>
#include <unordered_set>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/edge_line_connector.h"

namespace walnut {

// Returns a new numerator that barely satisfies:
//   (long double)ret/(long double)denom == (long double)num/(long double)denom
BigInt GetCloseToBoundary(BigInt num, BigInt denom) {
  DoublePoint3 unadjusted = HomoPoint3(num, num, num, denom).GetDoublePoint3();
  for (ssize_t i = num.used_words() * BigInt::bits_per_word; i >= 0; --i) {
    if (unadjusted == HomoPoint3(num + (BigInt(1) << i), num, num,
                                 denom).GetDoublePoint3()) {
      num += BigInt(1) << i;
    }
  }
  return num;
}

// Returns a vector of equivalent HomoPoint3s that each have a different
// DoublePoint3 representation.
//
// This can create at least 3 equivalent HomoPoint3s. Every coordinate (along
// with the denominator) is greater than 2 in the returned HomoPoint3s.
std::vector<HomoPoint3> MakePointsWithDifferentReps(size_t count) {
  std::vector<HomoPoint3> result;
  if (count == 0) return result;

  BigInt w(BigInt(37) << 60);
  HomoPoint3 canonical(GetCloseToBoundary(BigInt(251) << 63, w),
                       GetCloseToBoundary(BigInt(251) << 62, w),
                       GetCloseToBoundary(BigInt(251) << 61, w),
                       w);

  std::unordered_set<DoublePoint3> found;
  found.insert(canonical.GetDoublePoint3());
  result.push_back(canonical);

  BigInt multiple(7);
  while (result.size() < count) {
    HomoPoint3 newpoint(canonical.x() * multiple, canonical.y() * multiple,
                        canonical.z() * multiple, canonical.w() * multiple);
    if (!std::isfinite(newpoint.GetDoublePoint3().x)) break;
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
  EXPECT_EQ(found.size(), 3);
  for (const HomoPoint3& p : found) {
    EXPECT_TRUE(std::isfinite(p.GetDoublePoint3().x))
      << "x=" << (long double)p.x() << " w=" << (long double)p.w();
    EXPECT_TRUE(std::isfinite(p.GetDoublePoint3().y))
      << "y=" << (long double)p.x() << " w=" << (long double)p.w();
    EXPECT_TRUE(std::isfinite(p.GetDoublePoint3().z))
      << "z=" << (long double)p.x() << " w=" << (long double)p.w();
  }
}

// Creates 4 triangles that meet at a central vertex. The caller can pass in
// different (but equivalent) values for the center vertex. All components of
// the center points (including the denominator) must be positive.
//
//          -|-           |
//         / | \          |
//       --  |  --        |
//     /   e0|    \       |
//   --      |     - -    |
//  /      c0|c1 e1   \   |
// +---------+---------+  |
//  \   e3 c3|c2      /   |
//   --      |      --    |
//     \     |e2   /      |
//      --   |   --       |
//        \  |  /         |
//         --|--          |
void Generate4Triangles(const HomoPoint3& center0,
                        const HomoPoint3& center1,
                        const HomoPoint3& center2,
                        const HomoPoint3& center3,
                        std::vector<ConnectedPolygon<>>& triangles) {
  triangles.clear();

  HomoPoint3 left(-center0.x(), center0.y(), center0.z(), center0.w());
  HomoPoint3 top(center0.x(), center0.y() << 1, center0.z(), center0.w());
  HomoPoint3 right(center0.x() << 1, center0.y(), center0.z(), center0.w());
  HomoPoint3 bottom(center0.x(), -center0.y(), center0.z(), center0.w());

  HalfSpace3 plane(/*x=*/BigInt{0}, /*y=*/BigInt{0}, /*z=*/center0.w(),
                   /*dist=*/center0.z());

  // The ConnectedPolygon constructor makes the last vertex the first. So list
  // the center vertex last so that the edge with the center as the source
  // becomes the first edge.
  triangles.emplace_back(plane, /*drop_dimension=*/2,
                         std::vector<HomoPoint3>{top, left, center0});
  triangles.emplace_back(plane, /*drop_dimension=*/2,
                         std::vector<HomoPoint3>{right, top, center1});
  triangles.emplace_back(plane, /*drop_dimension=*/2,
                         std::vector<HomoPoint3>{bottom, right, center2});
  triangles.emplace_back(plane, /*drop_dimension=*/2,
                         std::vector<HomoPoint3>{left, bottom, center3});

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
}

TEST(VertexDoublePoint3Mapper, MapMerge) {
  std::vector<HomoPoint3> centers = MakePointsWithDifferentReps(2);
  const HomoPoint3& p = centers[0];
  const HomoPoint3& q = centers[1];

  std::vector<ConnectedPolygon<>> triangles;
  Generate4Triangles(p, p, q, q, triangles);

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

// Creates 1000 different mappers, each with a different amount of existing
// entries, then adds a new entry each one and verifies that the references are
// valid.
TEST(VertexDoublePoint3Mapper, RehashDuringMerge) {
  auto value_factory = [](const DoublePoint3& p) -> int {
    return 0;
  };
  auto value_merger = [](RedirectableValue<int>& a,
                         RedirectableValue<int>& b) {
    a.Redirect(b);
  };
  std::vector<VertexDoublePoint3Mapper<decltype(value_factory),
                                       decltype(value_merger)>> mappers;
  for (size_t i = 0; i < 20; ++i) {
    mappers.emplace_back(value_factory, value_merger);

    HomoPoint3 center(i + 1, i + 1, i + 1, 1);
    std::vector<ConnectedPolygon<>> triangles;
    Generate4Triangles(center, center, center, center, triangles);
    for (size_t j = 0; j < i; ++j) {
      auto& result = mappers[j].Map(triangles[0].edge(0));
      EXPECT_EQ(result.first, triangles[0].vertex(0).GetDoublePoint3());
    }
  }

  std::vector<ConnectedPolygon<>> triangles;
  std::vector<HomoPoint3> centers = MakePointsWithDifferentReps(3);
  Generate4Triangles(centers[0], centers[1], centers[2], centers[2],
                     triangles);

  VertexDoublePoint3Mapper<decltype(value_factory), decltype(value_merger)>
    mapper(value_factory, value_merger);
  for (size_t j = 0; j < 4; ++j) {
    for (size_t i = 0; i < 20; ++i) {
      auto& result = mappers[i].Map(triangles[j].edge(0));
      auto& retry = mappers[i].Map(triangles[j].edge(0));
      EXPECT_EQ(result, retry);
    }
  }
}

}  // walnut
