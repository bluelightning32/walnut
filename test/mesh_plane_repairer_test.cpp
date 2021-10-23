#include "walnut/mesh_plane_repairer.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using ::testing::Contains;

void Nudge(HomoPoint3& p, int x, int y, int z) {
  p.w() *= 1000;
  p.x() = p.x()*1000 + x;
  p.y() = p.y()*1000 + y;
  p.z() = p.z()*1000 + z;
}

void NudgeAndExpectPerfectRepair(
    const std::vector<HomoPoint3>& vertices,
    const std::vector<std::vector<size_t>>& indices) {
  // Nudge up to 4 vertices
  for (size_t i = 0; i < vertices.size(); ++i) {
    for (size_t j = i; j < vertices.size(); ++j) {
      for (size_t k = j; k < vertices.size(); ++k) {
        for (size_t l = k; l < vertices.size(); ++l) {
          std::vector<HomoPoint3> nudged = vertices;
          Nudge(nudged[i], 2, 3, 5);
          Nudge(nudged[j], 7, 11, 13);
          Nudge(nudged[k], 17, 19, 23);
          Nudge(nudged[l], 29, 31, 37);

          MeshPlaneRepairer<> builder;
          for (const std::vector<size_t>& polygon : indices) {
            for (const size_t vertex_index : polygon) {
              builder.AddVertex(nudged[vertex_index]);
            }
            builder.FinishFacet();
          }
          MeshPlaneRepairerProducer<> output_polygons =
            std::move(builder).FinalizeMesh();

          for (const std::vector<size_t>& polygon : indices) {
            ASSERT_TRUE(output_polygons.HasMorePolygons());

            MeshPlaneRepairerProducer<>::VertexIterator pos, last_vertex;
            HalfSpace3 plane =
              output_polygons.GetNextPolygon(pos, last_vertex);
            for (const size_t vertex_index : polygon) {
              ASSERT_NE(pos, last_vertex);
              EXPECT_EQ(*pos, nudged[vertex_index]);
              EXPECT_TRUE(plane.IsCoincident(*pos));
              ++pos;
            }
          }
        }
      }
    }
  }
}

void NudgeAndExpectSamePolygonCount(
    const std::vector<HomoPoint3>& vertices,
    const std::vector<std::vector<size_t>>& indices) {
  // Nudge up to 4 vertices
  for (size_t i = 0; i < vertices.size(); ++i) {
    for (size_t j = i; j < vertices.size(); ++j) {
      for (size_t k = j; k < vertices.size(); ++k) {
        for (size_t l = k; l < vertices.size(); ++l) {
          std::vector<HomoPoint3> nudged = vertices;
          Nudge(nudged[i], 2, 3, 5);
          Nudge(nudged[j], 7, 11, 13);
          Nudge(nudged[k], 17, 19, 23);
          Nudge(nudged[l], 29, 31, 37);

          MeshPlaneRepairer<> builder;
          for (const std::vector<size_t>& polygon : indices) {
            for (const size_t vertex_index : polygon) {
              builder.AddVertex(nudged[vertex_index]);
            }
            builder.FinishFacet();
          }
          MeshPlaneRepairerProducer<> output_polygons =
            std::move(builder).FinalizeMesh();

          for (const std::vector<size_t>& polygon : indices) {
            ASSERT_TRUE(output_polygons.HasMorePolygons());

            MeshPlaneRepairerProducer<>::VertexIterator pos, last_vertex;
            HalfSpace3 plane =
              output_polygons.GetNextPolygon(pos, last_vertex);
            for (const size_t vertex_index : polygon) {
              ASSERT_NE(pos, last_vertex);
              BigInt denom;
              Vector3 difference = pos->Difference(nudged[vertex_index],
                                                   denom);
              EXPECT_LT(double(difference.GetScaleSquared())/
                        double(denom*denom), 0.1)
                << "before=" << nudged[vertex_index].GetDoublePoint3()
                << " fixed=" << pos->GetDoublePoint3();
              EXPECT_TRUE(plane.IsCoincident(*pos));
              ++pos;
            }
          }
        }
      }
    }
  }
}

TEST(MeshPlaneRepairer, Tetrahedron) {
  // The MeshPlaneRepairer should accept the tetrahedron sides without altering
  // them.
  std::vector<HomoPoint3> vertices{
    HomoPoint3(0, 0, 0, 1),
    HomoPoint3(1, 0, 0, 1),
    HomoPoint3(0, 1, 0, 1),
    HomoPoint3(0, 0, 1, 1),
  };

  std::vector<std::vector<size_t>> indices{
    /*bottom=*/std::vector<size_t>{0, 2, 1},
    /*south=*/std::vector<size_t>{0, 1, 3},
    /*northeast=*/std::vector<size_t>{1, 2, 3},
    /*west=*/std::vector<size_t>{2, 0, 3},
  };

  NudgeAndExpectPerfectRepair(vertices, indices);
}

TEST(MeshPlaneRepairer, Cube) {
  std::vector<HomoPoint3> vertices{
    HomoPoint3(0, 0, 0, 1),
    HomoPoint3(1, 0, 0, 1),
    HomoPoint3(1, 1, 0, 1),
    HomoPoint3(0, 1, 0, 1),
    HomoPoint3(0, 0, 1, 1),
    HomoPoint3(1, 0, 1, 1),
    HomoPoint3(1, 1, 1, 1),
    HomoPoint3(0, 1, 1, 1),
  };

  std::vector<std::vector<size_t>> indices{
    /*bottom=*/std::vector<size_t>{3, 2, 1, 0},
    /*south=*/std::vector<size_t>{0, 1, 5, 4},
    /*east=*/std::vector<size_t>{1, 2, 6, 5},
    /*north=*/std::vector<size_t>{2, 3, 7, 6},
    /*west=*/std::vector<size_t>{3, 0, 4, 7},
    /*top=*/std::vector<size_t>{4, 5, 6, 7},
  };

  NudgeAndExpectSamePolygonCount(vertices, indices);
}

static constexpr double tau = 6.28318;

TEST(MeshPlaneRepairer, SplitUnrepairableHexagon) {
  std::vector<HomoPoint3> top_ring;
  std::vector<HomoPoint3> bottom_ring;
  constexpr int ring_count = 6;
  for (int i = 0; i < ring_count; ++i) {
    double top_angle = i * tau / ring_count;
    double nudge = i == 3 ? 0.00001 : 0;
    top_ring.push_back(HomoPoint3::FromDoubles(/*min_exponent=*/-20,
                                               /*x=*/cos(top_angle),
                                               /*y=*/sin(top_angle),
                                               /*z=*/1 + nudge));
    // The bottom ring is staggered as compared to the top.
    double bottom_angle = i * tau / ring_count + 0.5 / ring_count;
    bottom_ring.push_back(HomoPoint3::FromDoubles(/*min_exponent=*/-20,
                                                  /*x=*/cos(bottom_angle),
                                                  /*y=*/sin(bottom_angle),
                                                  /*z=*/0));
  }

  MeshPlaneRepairer<> builder;
  // Add the bottom facet
  for (size_t i = ring_count; i > 0; ) {
    --i;
    builder.AddVertex(bottom_ring[i]);
  }
  builder.FinishFacet();

  // Add the sides with two edges on the bottom.
  for (size_t i = 0; i < ring_count; ++i) {
    builder.AddVertex(bottom_ring[i]);
    builder.AddVertex(bottom_ring[(i + 1) % ring_count]);
    builder.AddVertex(top_ring[(i + 1) % ring_count]);
    builder.FinishFacet();
  }

  // Add the sides with two edges on the top.
  for (size_t i = 0; i < ring_count; ++i) {
    builder.AddVertex(top_ring[(i + 1) % ring_count]);
    builder.AddVertex(top_ring[i]);
    builder.AddVertex(bottom_ring[i]);
    builder.FinishFacet();
  }

  // Add the top facet
  for (size_t i = 0; i < ring_count; ++i) {
    builder.AddVertex(top_ring[i]);
  }
  builder.FinishFacet();

  MeshPlaneRepairerProducer<> output_polygons =
    std::move(builder).FinalizeMesh();
  MeshPlaneRepairerProducer<>::VertexIterator pos, last_vertex;

  // Validate the bottom facet
  ASSERT_TRUE(output_polygons.HasMorePolygons());
  EXPECT_EQ(output_polygons.GetNextPolygon(pos, last_vertex),
            HalfSpace3(/*x=*/0, /*y=*/0, /*z=*/-1, /*dist=*/0));
  for (size_t i = 0; i < ring_count; ++i) {
    ASSERT_NE(pos, last_vertex);
    EXPECT_EQ(*pos, bottom_ring[ring_count - i - 1]);
    ++pos;
  }

  // Validate the side facets connected to the bottom
  for (size_t i = 0; i < ring_count; ++i) {
    ASSERT_TRUE(output_polygons.HasMorePolygons());
    HalfSpace3 plane = output_polygons.GetNextPolygon(pos, last_vertex);

    ASSERT_NE(pos, last_vertex);
    EXPECT_EQ(*pos, bottom_ring[i]);
    EXPECT_TRUE(plane.IsCoincident(*pos));
    ++pos;

    ASSERT_NE(pos, last_vertex);
    EXPECT_EQ(*pos, bottom_ring[(i + 1) % ring_count]);
    EXPECT_TRUE(plane.IsCoincident(*pos));
    ++pos;

    ASSERT_NE(pos, last_vertex);
    EXPECT_EQ(*pos, top_ring[(i + 1) % ring_count]);
    EXPECT_TRUE(plane.IsCoincident(*pos));
    ++pos;

    EXPECT_EQ(pos, last_vertex);
  }

  // Validate the side facets connected to the top
  for (size_t i = 0; i < ring_count; ++i) {
    ASSERT_TRUE(output_polygons.HasMorePolygons());
    HalfSpace3 plane = output_polygons.GetNextPolygon(pos, last_vertex);

    ASSERT_NE(pos, last_vertex);
    EXPECT_EQ(*pos, top_ring[(i + 1) % ring_count]);
    EXPECT_TRUE(plane.IsCoincident(*pos));
    ++pos;

    ASSERT_NE(pos, last_vertex);
    EXPECT_EQ(*pos, top_ring[i]);
    EXPECT_TRUE(plane.IsCoincident(*pos));
    ++pos;

    ASSERT_NE(pos, last_vertex);
    EXPECT_EQ(*pos, bottom_ring[i]);
    EXPECT_TRUE(plane.IsCoincident(*pos));
    ++pos;

    EXPECT_EQ(pos, last_vertex);
  }

  int top_vertex_count = 0;
  int top_polygon_count = 0;
  std::set<size_t> found_top_indices;
  while(output_polygons.HasMorePolygons()) {
    ++top_polygon_count;
    HalfSpace3 plane = output_polygons.GetNextPolygon(pos, last_vertex);

    while (pos != last_vertex) {
      auto found = std::find(top_ring.begin(), top_ring.end(), *pos);
      ASSERT_NE(found, top_ring.end());
      found_top_indices.insert(found - top_ring.begin());
      EXPECT_TRUE(plane.IsCoincident(*pos));
      ++top_vertex_count;
      ++pos;
    }
  }
  EXPECT_EQ(top_vertex_count - top_polygon_count*2, 4)
    << "top_vertex_count=" << top_vertex_count
    << " top_polygon_count=" << top_polygon_count;
  EXPECT_EQ(found_top_indices.size(), ring_count);
}

}  // walnut
