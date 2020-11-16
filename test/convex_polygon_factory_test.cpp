#include "walnut/convex_polygon_factory.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::ElementsAre;
using testing::IsEmpty;

template <typename Factory = ConvexPolygon<32>::Factory>
class ResultCollector : public Factory {
 public:
  using ConvexPolygonRep = typename Factory::ConvexPolygonRep;
  using HomoPoint3Rep = typename ConvexPolygonRep::HomoPoint3Rep;
  using HalfSpace3Rep = typename ConvexPolygonRep::HalfSpace3Rep;

  std::vector<ConvexPolygonRep> GetSortedPolygonResult() {
    for (ConvexPolygonRep& polygon : result_) {
      polygon.SortVertices();
    }

    std::sort(result_.begin(), result_.end(), PolygonLt);
    return result_;
  }

  std::vector<std::vector<Point3<32>>> GetSortedPolygonVertices() {
    std::vector<ConvexPolygonRep> sorted = GetSortedPolygonResult();
    std::vector<std::vector<Point3<32>>> result;
    for (const ConvexPolygonRep& polygon : sorted) {
      std::vector<Point3<32>> vertices;
      for (size_t i = 0; i < polygon.vertex_count(); ++i) {
        const HomoPoint3Rep& vertex = polygon.vertex(i);
        EXPECT_EQ(vertex.dist_denom(), 1);
        vertices.emplace_back(vertex.x(), vertex.y(), vertex.z());
      }
      result.push_back(std::move(vertices));
    }
    return result;
  }

  static bool PolygonLt(const ConvexPolygonRep& a,
                        const ConvexPolygonRep& b) {
    return std::lexicographical_compare(a.edges().begin(),
        a.edges().end(),
        b.edges().begin(), b.edges().end(),
        ConvexPolygonRep::EdgeRep::LexicographicallyLt);
  }

  // Check that every vertex in every convex polygon really is a convex vertex.
  void VerifyAllConvexVertices() const {
    for (const ConvexPolygonRep& polygon : result_) {
      typename HalfSpace3Rep::VectorInt drop_dim_value =
        polygon.plane().normal().components()[polygon.drop_dimension()];
      ASSERT_NE(drop_dim_value, 0);
      const int orientation = drop_dim_value.GetSign();
      const size_t vertex_count = polygon.vertex_count();
      for (int i = 0; i < vertex_count; ++i) {
        const HomoPoint3Rep& prev = polygon.vertex(
            (vertex_count + i - 1) % vertex_count);
        const HomoPoint3Rep& cur = polygon.vertex(i);
        const HomoPoint3Rep& next = polygon.vertex((i + 1) % vertex_count);
        EXPECT_LE(cur.Get2DTwistDir(polygon.drop_dimension(), prev, next) *
                  orientation, 0);
      }
    }
  }

  int CountVertexAppearances(const Point3<32>& v) {
    size_t matching = 0;
    for (const ConvexPolygonRep& polygon : result_) {
      for (size_t i = 0; i < polygon.vertex_count(); ++i) {
        const HomoPoint3Rep& hv = polygon.vertex(i);
        if (v == hv) {
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
  Point3<32> input[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10),
    Point3<32>(1, 1, 10),
  };

  ResultCollector<> collector;
  collector.Build(std::begin(input), std::end(input));
  ASSERT_THAT(collector.GetSortedPolygonVertices(), ElementsAre(
        std::vector<Point3<32>>{input[0], input[1], input[2]}
        ));
  EXPECT_EQ(collector.GetSortedPolygonResult()[0].plane(),
            ResultCollector<>::HalfSpace3Rep(/*x=*/0, /*y=*/0, /*z=*/1,
                                             /*dist=*/10));
}

TEST(ConvexPolygonFactory, Collinear3Points) {
  Point3<32> input[] = {
    Point3<32>(0, 0, 0),
    Point3<32>(1, 1, 1),
    Point3<32>(2, 2, 2),
  };

  ResultCollector<> collector;
  collector.Build(std::begin(input), std::end(input));
  ASSERT_THAT(collector.GetSortedPolygonVertices(), IsEmpty());
}

TEST(ConvexPolygonFactory, CWTriangle) {
  Point3<32> input[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(1, 0, 10),
  };

  ResultCollector<> collector;
  collector.Build(std::begin(input), std::end(input));
  ASSERT_THAT(collector.GetSortedPolygonVertices(), ElementsAre(
        std::vector<Point3<32>>{input[0], input[1], input[2]}
        ));
  EXPECT_EQ(collector.GetSortedPolygonResult()[0].plane(),
            ResultCollector<>::HalfSpace3Rep(/*x=*/0, /*y=*/0, /*z=*/-1,
                                             /*dist=*/-10));
}

TEST(ConvexPolygonFactory, Square) {
  Point3<32> input[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(0, 1, 10),
  };

  ResultCollector<> collector;
  collector.Build(std::begin(input), std::end(input));
  ASSERT_THAT(collector.GetSortedPolygonVertices(), ElementsAre(
        std::vector<Point3<32>>{input[0], input[1], input[2], input[3]}
        ));
  EXPECT_EQ(collector.GetSortedPolygonResult()[0].plane(),
            ResultCollector<>::HalfSpace3Rep(/*x=*/0, /*y=*/0, /*z=*/1,
                                             /*dist=*/10));
}

TEST(ConvexPolygonFactory, SplitSquareAtPlaneBreak) {
  Point3<32> input[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(0, 1, 20),
  };

  ResultCollector<> collector;
  collector.Build(std::begin(input), std::end(input));
  // The implementation is allowed to split the square on either diagonal. Even
  // though the code is deterministic, the test is written in a way that is
  // agnostic to that implementation detail.
  if (collector.CountVertexAppearances(input[0]) == 2) {
    ASSERT_THAT(collector.GetSortedPolygonVertices(), ElementsAre(
          std::vector<Point3<32>>{input[0], input[1], input[2]},
          std::vector<Point3<32>>{input[0], input[2], input[3]}
          ));
  } else {
    ASSERT_THAT(collector.GetSortedPolygonVertices(), ElementsAre(
          std::vector<Point3<32>>{input[0], input[1], input[3]},
          std::vector<Point3<32>>{input[3], input[1], input[2]}
          ));
  }
}

TEST(ConvexPolygonFactory, SplitSquareAtPlaneBreakCW) {
  Point3<32> input[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(0, 1, 20),
    Point3<32>(1, 1, 10),
    Point3<32>(1, 0, 10),
  };

  ResultCollector<> collector;
  collector.Build(std::begin(input), std::end(input));
  // The implementation is allowed to split the square on either diagonal. Even
  // though the code is deterministic, the test is written in a way that is
  // agnostic to that implementation detail.
  if (collector.CountVertexAppearances(input[0]) == 2) {
    ASSERT_THAT(collector.GetSortedPolygonVertices(), ElementsAre(
          std::vector<Point3<32>>{input[0], input[2], input[3]},
          std::vector<Point3<32>>{input[0], input[1], input[2]}
          ));
  } else {
    ASSERT_THAT(collector.GetSortedPolygonVertices(), ElementsAre(
          std::vector<Point3<32>>{input[0], input[1], input[3]},
          std::vector<Point3<32>>{input[1], input[2], input[3]}
          ));
  }
}

TEST(ConvexPolygonFactory, SelfIntersecting) {
  //
  //   p4        p2
  //  /  \      /  \
  // p0 --\----/--> p1
  //       \  /
  //        p3
  //
  Point3<32> input[] = {
    /*p0=*/Point3<32>(0, 0, 10),
    /*p1=*/Point3<32>(4, 0, 10),
    /*p2=*/Point3<32>(3, 1, 10),
    /*p3=*/Point3<32>(2, -1, 10),
    /*p4=*/Point3<32>(1, 1, 10),
  };

  ResultCollector<> collector;
  collector.Build(std::begin(input), std::end(input));
  ASSERT_THAT(collector.GetSortedPolygonVertices(), ElementsAre(
        std::vector<Point3<32>>{input[0], input[3], input[4]},
        std::vector<Point3<32>>{input[0], input[1], input[3]},
        std::vector<Point3<32>>{input[3], input[1], input[2]}
        ));
  EXPECT_EQ(collector.GetSortedPolygonResult()[0].plane(),
            ResultCollector<>::HalfSpace3Rep(/*x=*/0, /*y=*/0, /*z=*/1,
                                             /*dist=*/10));
  EXPECT_EQ(collector.GetSortedPolygonResult()[1].plane(),
            ResultCollector<>::HalfSpace3Rep(/*x=*/0, /*y=*/0, /*z=*/-1,
                                             /*dist=*/-10));
  EXPECT_EQ(collector.GetSortedPolygonResult()[2].plane(),
            ResultCollector<>::HalfSpace3Rep(/*x=*/0, /*y=*/0, /*z=*/1,
                                             /*dist=*/10));
}

TEST(ConvexPolygonFactory, SelfIntersectingStartAtReflex) {
  //
  //   p1        p4
  //  /  \      /  \
  // p2 --\----/--> p3
  //       \  /
  //        p0
  //
  Point3<32> input[] = {
    /*p0=*/Point3<32>(2, -1, 10),
    /*p1=*/Point3<32>(1, 1, 10),
    /*p2=*/Point3<32>(0, 0, 10),
    /*p3=*/Point3<32>(4, 0, 10),
    /*p4=*/Point3<32>(3, 1, 10),
  };

  ResultCollector<> collector;
  collector.Build(std::begin(input), std::end(input));
  ASSERT_THAT(collector.GetSortedPolygonVertices(), ElementsAre(
        std::vector<Point3<32>>{input[(0 + 2)%5], input[(3 + 2)%5],
                                input[(4 + 2)%5]},
        std::vector<Point3<32>>{input[(0 + 2)%5], input[(1 + 2)%5],
                                input[(3 + 2)%5]},
        std::vector<Point3<32>>{input[(3 + 2)%5], input[(1 + 2)%5],
                                input[(2 + 2)%5]}
        ));
  EXPECT_EQ(collector.GetSortedPolygonResult()[0].plane(),
            ResultCollector<>::HalfSpace3Rep(/*x=*/0, /*y=*/0, /*z=*/1,
                                             /*dist=*/10));
  EXPECT_EQ(collector.GetSortedPolygonResult()[1].plane(),
            ResultCollector<>::HalfSpace3Rep(/*x=*/0, /*y=*/0, /*z=*/-1,
                                             /*dist=*/-10));
  EXPECT_EQ(collector.GetSortedPolygonResult()[2].plane(),
            ResultCollector<>::HalfSpace3Rep(/*x=*/0, /*y=*/0, /*z=*/1,
                                             /*dist=*/10));
}

TEST(ConvexPolygonFactory, VertexData) {
  struct NoDefaultConstructor : public std::string {
    using std::string::string;
    NoDefaultConstructor() = delete;
  };
  using ConvexPolygonRep = ConvexPolygon<32, NoDefaultConstructor>;
  using ConvexPolygonEdgeRep = ConvexPolygonRep::EdgeRep;
  using Point3WithString = ConvexPolygonEdgeRep::Point3WithVertexData;
  Point3WithString input[] = {
    Point3WithString(0, 0, 10, "p0"),
    Point3WithString(1, 0, 10, "p1"),
    Point3WithString(1, 1, 10, "p2"),
  };

  ResultCollector<ConvexPolygonRep::FactoryWithVertexData> collector;
  collector.Build(std::begin(input), std::end(input));
  ASSERT_THAT(collector.GetSortedPolygonVertices(), ElementsAre(
        std::vector<Point3<32>>{input[0], input[1], input[2]}
        ));

  ASSERT_EQ(collector.GetSortedPolygonResult().size(), 1);
  ConvexPolygonRep polygon = collector.GetSortedPolygonResult()[0];

  ASSERT_EQ(polygon.vertex_count(), 3);
  EXPECT_EQ(polygon.edges()[0].data, input[0].data);
  EXPECT_EQ(polygon.edges()[1].data, input[1].data);
  EXPECT_EQ(polygon.edges()[2].data, input[2].data);

  polygon.vertex_data(2) = "new p2";
  EXPECT_EQ(polygon.edges()[2].data, "new p2");
}

TEST(ConvexPolygonFactory, GetPlaneOrientationAfterProjection) {
  Point3<32> p[] = {
    Point3<32>(10, 0, 1),
    Point3<32>(0, 0, 0),
    Point3<32>(0, 10, 1),
  };

  for (int drop_dimension = 0; drop_dimension < 3; ++drop_dimension) {
    auto cross = HalfSpace3<>(p[0], p[1], p[2]).normal();

    EXPECT_NE(cross.components()[drop_dimension], 0);
    int orientation =
      ConvexPolygon<32>::Factory::GetPlaneOrientationAfterProjection(
        cross, drop_dimension);

    auto cross2 = (p[0] - p[1]).DropDimension(drop_dimension).Cross(
        (p[2] - p[1]).DropDimension(drop_dimension));
    EXPECT_GT(cross2 * orientation, 0)
      << " drop_dimension=" << drop_dimension
      << " orientation=" << orientation
      << " cross2=" << cross2;
  }
}

}  // walnut
