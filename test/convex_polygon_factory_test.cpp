#include "walnut/convex_polygon_factory.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::ElementsAre;

class ResultCollector : public ConvexPolygon<32>::Factory {
 public:
  using ConvexPolygonRep = ConvexPolygon<32>;
  using Vertex4Rep = ConvexPolygonRep::Vertex4Rep;
  using PlaneRep = ConvexPolygonRep::PlaneRep;

  std::vector<ConvexPolygonRep> GetSortedPolygonResult() {
    for (ConvexPolygonRep& polygon : result_) {
      polygon.SortVertices();
    }

    std::sort(result_.begin(), result_.end(), PolygonLt);
    return result_;
  }

  std::vector<std::vector<Vertex3<32>>> GetSortedPolygonVertices() {
    std::vector<ConvexPolygonRep> sorted = GetSortedPolygonResult();
    std::vector<std::vector<Vertex3<32>>> result;
    for (const ConvexPolygonRep& polygon : sorted) {
      std::vector<Vertex3<32>> vertices;
      for (const Vertex4Rep vertex4 : polygon.vertices()) {
        EXPECT_EQ(vertex4.dist_denom(), 1);
        vertices.emplace_back(vertex4.x(), vertex4.y(), vertex4.z());
      }
      result.push_back(std::move(vertices));
    }
    return result;
  }

  static bool PolygonLt(const ConvexPolygonRep& a,
                        const ConvexPolygonRep& b) {
    return std::lexicographical_compare(a.vertices().begin(), a.vertices().end(),
        b.vertices().begin(), b.vertices().end(),
        Vertex4Rep::LexicographicallyLt<>);
  }

  // Check that every vertex in every convex polygon really is a convex vertex.
  void VerifyAllConvexVertices() const {
    for (const ConvexPolygonRep& polygon : result_) {
      PlaneRep::VectorInt drop_dim_value = 
        polygon.plane().normal().coords()[polygon.drop_dimension()];
      ASSERT_NE(drop_dim_value, 0);
      const int orientation = drop_dim_value.GetSign();
      const size_t vertex_count = polygon.vertices().size();
      for (int i = 0; i < vertex_count; ++i) {
        const Vertex4Rep& prev = polygon.vertices()[
            (vertex_count + i - 1) % vertex_count];
        const Vertex4Rep& cur = polygon.vertices()[i];
        const Vertex4Rep& next = polygon.vertices()[(i + 1) % vertex_count];
        EXPECT_LE(cur.Get2DTwistDir(polygon.drop_dimension(), prev, next) *
                  orientation, 0);
      }
    }
  }

  int CountVertexAppearances(const Vertex3<32>& v) {
    size_t matching = 0;
    for (const ConvexPolygonRep& polygon : result_) {
      for (const Vertex4Rep& v4 : polygon.vertices()) {
        if (v == v4) {
          ++matching;
        }
      }
    }
    return matching;
  }

 protected:
  void Emit(ConvexPolygonRep&& polygon) override {
    result_.push_back(std::move(polygon));
  }

 private:
  std::vector<ConvexPolygonRep> result_;
};

TEST(ConvexPolygonFactory, Triangle) {
  Vertex3<32> input[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(1, 0, 10),
    Vertex3<32>(1, 1, 10),
  };

  ResultCollector collector;
  collector.Build(std::begin(input), std::end(input));
  ASSERT_THAT(collector.GetSortedPolygonVertices(), ElementsAre(
        std::vector<Vertex3<32>>{input[0], input[1], input[2]}
        ));
  EXPECT_EQ(collector.GetSortedPolygonResult()[0].plane(),
            ResultCollector::PlaneRep(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10));
}

TEST(ConvexPolygonFactory, CWTriangle) {
  Vertex3<32> input[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(1, 1, 10),
    Vertex3<32>(1, 0, 10),
  };

  ResultCollector collector;
  collector.Build(std::begin(input), std::end(input));
  ASSERT_THAT(collector.GetSortedPolygonVertices(), ElementsAre(
        std::vector<Vertex3<32>>{input[0], input[1], input[2]}
        ));
  EXPECT_EQ(collector.GetSortedPolygonResult()[0].plane(),
            ResultCollector::PlaneRep(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10));
}

TEST(ConvexPolygonFactory, Square) {
  Vertex3<32> input[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(1, 0, 10),
    Vertex3<32>(1, 1, 10),
    Vertex3<32>(0, 1, 10),
  };

  ResultCollector collector;
  collector.Build(std::begin(input), std::end(input));
  ASSERT_THAT(collector.GetSortedPolygonVertices(), ElementsAre(
        std::vector<Vertex3<32>>{input[0], input[1], input[2], input[3]}
        ));
  EXPECT_EQ(collector.GetSortedPolygonResult()[0].plane(),
            ResultCollector::PlaneRep(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10));
}

TEST(ConvexPolygonFactory, SplitSquareAtPlaneBreak) {
  Vertex3<32> input[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(1, 0, 10),
    Vertex3<32>(1, 1, 10),
    Vertex3<32>(0, 1, 20),
  };

  ResultCollector collector;
  collector.Build(std::begin(input), std::end(input));
  // The implementation is allowed to split the square on either diagonal. Even
  // though the code is deterministic, the test is written in a way that is
  // agnostic to that implementation detail.
  if (collector.CountVertexAppearances(input[0]) == 2) {
    ASSERT_THAT(collector.GetSortedPolygonVertices(), ElementsAre(
          std::vector<Vertex3<32>>{input[0], input[1], input[2]},
          std::vector<Vertex3<32>>{input[0], input[2], input[3]}
          ));
  } else {
    ASSERT_THAT(collector.GetSortedPolygonVertices(), ElementsAre(
          std::vector<Vertex3<32>>{input[0], input[1], input[3]},
          std::vector<Vertex3<32>>{input[3], input[1], input[2]}
          ));
  }
}

TEST(ConvexPolygonFactory, SplitSquareAtPlaneBreakCW) {
  Vertex3<32> input[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(0, 1, 20),
    Vertex3<32>(1, 1, 10),
    Vertex3<32>(1, 0, 10),
  };

  ResultCollector collector;
  collector.Build(std::begin(input), std::end(input));
  // The implementation is allowed to split the square on either diagonal. Even
  // though the code is deterministic, the test is written in a way that is
  // agnostic to that implementation detail.
  if (collector.CountVertexAppearances(input[0]) == 2) {
    ASSERT_THAT(collector.GetSortedPolygonVertices(), ElementsAre(
          std::vector<Vertex3<32>>{input[0], input[2], input[3]},
          std::vector<Vertex3<32>>{input[0], input[1], input[2]}
          ));
  } else {
    ASSERT_THAT(collector.GetSortedPolygonVertices(), ElementsAre(
          std::vector<Vertex3<32>>{input[0], input[1], input[3]},
          std::vector<Vertex3<32>>{input[1], input[2], input[3]}
          ));
  }
}

}  // walnut