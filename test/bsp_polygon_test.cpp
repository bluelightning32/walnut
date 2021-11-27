#include "walnut/bsp_polygon.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/convex_polygon_factory.h"

namespace walnut {

template<typename Container>
BSPPolygon<> MakePolygon(const Container& vertices) {
  using Iterator = decltype(std::begin(vertices));
  using Point3Rep = typename std::iterator_traits<Iterator>::value_type;
  using Factory = ConvexPolygonFactory<Point3Rep>;
  using ConvexPolygonRep = typename Factory::ConvexPolygonRep;

  class CollectOne : public Factory {
   public:
    CollectOne() = default;

    ConvexPolygonRep&& GetResult() {
      EXPECT_EQ(received_, 1);
      return std::move(result_);
    }

   protected:
    void Emit(ConvexPolygonRep&& polygon) override {
      EXPECT_EQ(received_, 0);
      received_++;
      result_ = std::move(polygon);
    }

   private:
    int received_ = 0;
    ConvexPolygonRep result_;
  };

  CollectOne collector;
  collector.Build(std::begin(vertices), std::end(vertices));
  ConvexPolygonRep parent(collector.GetResult());
  size_t first_vertex = 0;
  while (parent.vertex(first_vertex) != vertices[0]) ++first_vertex;
  parent.RotateEdges(first_vertex);
  return BSPPolygon<>(/*id=*/0, /*on_node_plane=*/nullptr, /*pos_side=*/false,
                      std::move(parent));
}

TEST(BSPPolygon, SetAdjacentBoundaryAnglesSingleVertex) {
  const HalfSpace3 split_plane =
    HalfSpace3::GetAxisAligned(/*dimension=*/0, /*numerator=*/BigInt(1),
                               /*denominator=*/BigInt(1));
  BSPPolygon<> polygon = MakePolygon(std::vector<Point3>{Point3(0, 0, 0),
                                                         Point3(0, 1, 0),
                                                         Point3(1, 1, 0)});
  EXPECT_EQ(polygon.vertex(0), Point3(0, 0, 0));

  SplitSide side{&split_plane, /*pos_side=*/false};
  polygon.SetAdjacentBoundaryAngles(side, /*adjacent=*/2);
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    const auto& edge = polygon.const_edge(i);
    EXPECT_EQ(edge.vertex_last_coincident, i == 2 ? side : SplitSide());
    EXPECT_EQ(edge.edge_first_coincident, SplitSide());
    EXPECT_EQ(edge.edge_last_coincident, SplitSide());
    EXPECT_EQ(edge.edge_created_by, SplitSide());
  }
  EXPECT_EQ(polygon.on_node_plane, SplitSide());
}

TEST(BSPPolygon, SetAdjacentBoundaryAnglesAdditionalBefore) {
  const HalfSpace3 split_plane =
    HalfSpace3::GetAxisAligned(/*dimension=*/0, /*numerator=*/BigInt(1),
                               /*denominator=*/BigInt(1));
  BSPPolygon<> polygon = MakePolygon(std::vector<Point3>{Point3(0, 0, 0),
                                                         Point3(0, 1, 0),
                                                         Point3(1, 1, 0),
                                                         Point3(1, 0, 0)});
  EXPECT_EQ(polygon.vertex(0), Point3(0, 0, 0));

  SplitSide side{&split_plane, /*pos_side=*/false};
  polygon.SetAdjacentBoundaryAngles(side, /*adjacent=*/3);
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    const auto& edge = polygon.const_edge(i);
    EXPECT_EQ(edge.vertex_last_coincident, i >= 2 ? side : SplitSide());
    EXPECT_EQ(edge.edge_first_coincident, i == 2 ? side : SplitSide());
    EXPECT_EQ(edge.edge_last_coincident, i == 2 ? side : SplitSide());
    EXPECT_EQ(edge.edge_created_by, SplitSide());
  }
  EXPECT_EQ(polygon.on_node_plane, SplitSide());
}

TEST(BSPPolygon, SetAdjacentBoundaryAnglesAdditionalAfter) {
  const HalfSpace3 split_plane =
    HalfSpace3::GetAxisAligned(/*dimension=*/0, /*numerator=*/BigInt(1),
                               /*denominator=*/BigInt(1));
  BSPPolygon<> polygon = MakePolygon(std::vector<Point3>{Point3(0, 0, 0),
                                                         Point3(0, 1, 0),
                                                         Point3(1, 1, 0),
                                                         Point3(1, 0, 0)});
  EXPECT_EQ(polygon.vertex(0), Point3(0, 0, 0));

  SplitSide side{&split_plane, /*pos_side=*/false};
  polygon.SetAdjacentBoundaryAngles(side, /*adjacent=*/2);
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    const auto& edge = polygon.const_edge(i);
    EXPECT_EQ(edge.vertex_last_coincident, i >= 2 ? side : SplitSide());
    EXPECT_EQ(edge.edge_first_coincident, i == 2 ? side : SplitSide());
    EXPECT_EQ(edge.edge_last_coincident, i == 2 ? side : SplitSide());
    EXPECT_EQ(edge.edge_created_by, SplitSide());
  }
  EXPECT_EQ(polygon.on_node_plane, SplitSide());
}

TEST(BSPPolygon, SetAdjacentBoundaryAnglesTwoBeforeAndAfter) {
  const HalfSpace3 split_plane =
    HalfSpace3::GetAxisAligned(/*dimension=*/0, /*numerator=*/BigInt(1),
                               /*denominator=*/BigInt(1));
  BSPPolygon<> polygon = MakePolygon(std::vector<Point3>{Point3(0, 0, 0),
                                                         Point3(0, 4, 0),
                                                         Point3(1, 4, 0),
                                                         Point3(1, 3, 0),
                                                         Point3(1, 2, 0),
                                                         Point3(1, 1, 0),
                                                         Point3(1, 0, 0)});
  EXPECT_EQ(polygon.vertex(0), Point3(0, 0, 0));

  SplitSide side{&split_plane, /*pos_side=*/false};
  polygon.SetAdjacentBoundaryAngles(side, /*adjacent=*/4);
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    const auto& edge = polygon.const_edge(i);
    EXPECT_EQ(edge.vertex_last_coincident, i >= 2 ? side : SplitSide());
    EXPECT_EQ(edge.edge_first_coincident, (i >= 2 && i < 6) ?
                                          side : SplitSide());
    EXPECT_EQ(edge.edge_last_coincident, (i >= 2 && i < 6) ?
                                          side : SplitSide());
    EXPECT_EQ(edge.edge_created_by, SplitSide());
  }
  EXPECT_EQ(polygon.on_node_plane, SplitSide());
}

TEST(BSPPolygon, SetAdjacentBoundaryAnglesOnPlane) {
  const HalfSpace3 split_plane =
    HalfSpace3::GetAxisAligned(/*dimension=*/2, /*numerator=*/BigInt(0),
                               /*denominator=*/BigInt(1));
  BSPPolygon<> polygon = MakePolygon(std::vector<Point3>{Point3(0, 0, 0),
                                                         Point3(0, 1, 0),
                                                         Point3(1, 1, 0)});
  EXPECT_EQ(polygon.vertex(0), Point3(0, 0, 0));

  SplitSide side{&split_plane, /*pos_side=*/false};
  polygon.SetAdjacentBoundaryAngles(side, /*adjacent=*/2);
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    const auto& edge = polygon.const_edge(i);
    EXPECT_EQ(edge.vertex_last_coincident, side);
    EXPECT_EQ(edge.edge_first_coincident, side);
    EXPECT_EQ(edge.edge_last_coincident, side);
    EXPECT_EQ(edge.edge_created_by, SplitSide());
  }
  EXPECT_EQ(polygon.on_node_plane, side);
}

}  // walnut
