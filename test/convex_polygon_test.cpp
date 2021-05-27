#include "walnut/convex_polygon.h"

#include <iterator>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/convex_polygon_factory.h"

namespace walnut {

using testing::AnyOf;
using testing::Eq;

template<typename Container>
MutableConvexPolygon<> MakeUnsortedConvexPolygon(const Container& vertices) {
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
  return collector.GetResult();
}


template<typename Container>
MutableConvexPolygon<> MakeConvexPolygon(const Container& vertices) {
  auto result = MakeUnsortedConvexPolygon(vertices);
  result.SortVertices();
  return result;
}

TEST(ConvexPolygon, TrianglePlane) {
  Point3 input[] = {
    Point3(0, 0, 10),
    Point3(1, 0, 10),
    Point3(1, 1, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(input);
  EXPECT_EQ(polygon.plane(),
            HalfSpace3(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10));

  EXPECT_FALSE(polygon.plane().normal().IsZero());
  EXPECT_GT(polygon.plane().normal().z(), 0);
  EXPECT_EQ(polygon.drop_dimension(), 2);

  for (const Point3& v : input) {
    EXPECT_TRUE(polygon.plane().IsCoincident(v));
  }
}

TEST(ConvexPolygon, RotateEdges) {
  Point3 input[] = {
    Point3(0, 0, 10),
    Point3(1, 0, 10),
    Point3(1, 1, 10),
  };

  MutableConvexPolygon<> polygon = MakeConvexPolygon(input);
  EXPECT_EQ(polygon.vertex(0), input[0]);
  EXPECT_EQ(polygon.vertex(1), input[1]);
  EXPECT_EQ(polygon.vertex(2), input[2]);

  polygon.RotateEdges(1);
  EXPECT_EQ(polygon.vertex(0), input[1]);
  EXPECT_EQ(polygon.vertex(1), input[2]);
  EXPECT_EQ(polygon.vertex(2), input[0]);
}

TEST(ConvexPolygon, Triangle0DistPlane) {
  Point3 input[] = {
    Point3(0, 0, 0),
    Point3(10, 0, 0),
    Point3(10, 10, 0),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(input);
  EXPECT_EQ(polygon.plane(),
            HalfSpace3(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/0));

  EXPECT_EQ(polygon.drop_dimension(), 2);

  for (const Point3& v : input) {
    EXPECT_TRUE(polygon.plane().IsCoincident(v));
  }
}

TEST(ConvexPolygon, TriangleXZPlane) {
  Point3 input[] = {
    Point3(0 - 5, 0, 0 + 5),
    Point3(10 - 5, 0, 10 + 5),
    Point3(10 - 5, 10, 10 + 5),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(input);
  EXPECT_EQ(polygon.plane(),
            HalfSpace3(/*x=*/-1, /*y=*/0, /*z=*/1, /*dist=*/10));

  for (const Point3& v : input) {
    EXPECT_TRUE(polygon.plane().IsCoincident(v));
  }
}

TEST(ConvexPolygon, ClockwiseTrianglePlane) {
  Point3 input[] = {
    Point3(0, 0, 10),
    Point3(1, 1, 10),
    Point3(1, 0, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(input);
  EXPECT_EQ(polygon.plane(),
            HalfSpace3(/*x=*/0, /*y=*/0, /*z=*/-1, /*dist=*/-10));

  EXPECT_FALSE(polygon.plane().normal().IsZero());
  EXPECT_LT(polygon.plane().normal().z(), 0);
  EXPECT_EQ(polygon.drop_dimension(), 2);

  for (const Point3& v : input) {
    EXPECT_TRUE(polygon.plane().IsCoincident(v));
  }
}

TEST(ConvexPolygon, ClockwiseTriangleXZPlane) {
  Point3 input[] = {
    Point3(0 - 5, 0, 0 + 5),
    Point3(10 - 5, 10, 10 + 5),
    Point3(10 - 5, 0, 10 + 5),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(input);
  EXPECT_EQ(polygon.plane(),
            HalfSpace3(/*x=*/1, /*y=*/0, /*z=*/-1, /*dist=*/-10));

  EXPECT_FALSE(polygon.plane().normal().IsZero());
  EXPECT_LT(polygon.plane().normal().z(), 0);

  for (const Point3& v : input) {
    EXPECT_TRUE(polygon.plane().IsCoincident(v));
  }
}

TEST(ConvexPolygon, ClockwiseSquareYZPlane) {
  Point3 input[] = {
    Point3(0, 0, 0),
    Point3(0, 10, 1),
    Point3(10, 10, 1),
    Point3(10, 0, 0),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(input);
  EXPECT_EQ(polygon.plane(),
            HalfSpace3(/*x=*/0, /*y=*/1, /*z=*/-10, /*dist=*/0));

  EXPECT_FALSE(polygon.plane().normal().IsZero());

  for (const Point3& v : input) {
    EXPECT_TRUE(polygon.plane().IsCoincident(v));
  }
}

TEST(ConvexPolygon, CopyConstructor) {
  Point3 input[] = {
    Point3(0, 0, 10),
    Point3(1, 0, 10),
    Point3(1, 1, 10),
  };

  ConvexPolygon<> polygon1 = MakeConvexPolygon(input);
  ConvexPolygon<> polygon2(polygon1);
  EXPECT_EQ(polygon2, polygon1);
}

TEST(ConvexPolygon, EqualityOperator) {
  Point3 input1[] = {
    Point3(0, 0, 10),
    Point3(1, 0, 10),
    Point3(1, 1, 10),
  };
  ConvexPolygon<> polygon1 = MakeUnsortedConvexPolygon(input1);
  EXPECT_EQ(polygon1, polygon1);

  // different plane
  {
    Point3 input2[] = {
      Point3(0, 0, 11),
      Point3(1, 0, 11),
      Point3(1, 1, 11),
    };
    ConvexPolygon<> polygon2 = MakeUnsortedConvexPolygon(input2);
    EXPECT_NE(polygon1, polygon2);
    EXPECT_NE(polygon2, polygon1);
  }

  // Same plane as input1, but translated
  {
    Point3 input3[] = {
      Point3(1, 0, 10),
      Point3(2, 0, 10),
      Point3(2, 1, 10),
    };
    ConvexPolygon<> polygon3 = MakeUnsortedConvexPolygon(input3);
    EXPECT_NE(polygon1, polygon3);
    EXPECT_NE(polygon3, polygon1);
  }

  // First 2 points the same as input1, but different 3rd point
  {
    Point3 input4[] = {
      input1[0],
      input1[1],
      Point3(2, 1, 10),
    };
    ConvexPolygon<> polygon4 = MakeUnsortedConvexPolygon(input4);
    EXPECT_NE(polygon1, polygon4);
    EXPECT_NE(polygon4, polygon1);
  }

  // Flipped version of input1
  {
    Point3 input5[] = {
      input1[2],
      input1[1],
      input1[0],
    };
    ConvexPolygon<> polygon5 = MakeUnsortedConvexPolygon(input5);
    EXPECT_NE(polygon1, polygon5);
    EXPECT_NE(polygon5, polygon1);
  }

  // Same as input1, but with vertex indices rotated
  {
    Point3 compare_input[] = {
      input1[1],
      input1[2],
      input1[0],
    };
    MutableConvexPolygon<> compare_polygon =
      MakeUnsortedConvexPolygon(compare_input);
    EXPECT_EQ(polygon1, compare_polygon);
    EXPECT_EQ(compare_polygon, polygon1);

    compare_polygon.SortVertices();
    EXPECT_EQ(polygon1, compare_polygon);
    EXPECT_EQ(compare_polygon, polygon1);
  }
}

TEST(ConvexPolygon, CounterClockwiseTriangleEdges) {
  Point3 input[] = {
    Point3(0, 0, 10),
    Point3(1, 0, 10),
    Point3(1, 1, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(input);

  ASSERT_EQ(polygon.vertex_count(), std::end(input) - std::begin(input));
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    EXPECT_EQ(polygon.vertex(i), input[i]);
    const HomoPoint3& vertex = polygon.vertex(i);
    const HomoPoint3& next_vertex = polygon.vertex(
        (i + 1) % polygon.vertex_count());
    const Point3& input_vertex = input[i];
    const Point3& next_input_vertex = input[
        (i + 1) % polygon.vertex_count()];
    EXPECT_TRUE(polygon.const_edge(i).line().IsCoincident(vertex));
    EXPECT_TRUE(polygon.const_edge(i).line().IsCoincident(next_vertex));
    EXPECT_TRUE(polygon.const_edge(i).line().IsCoincident(input_vertex));
    EXPECT_TRUE(polygon.const_edge(i).line().IsCoincident(next_input_vertex));
    EXPECT_GT(
        (next_input_vertex - input_vertex).Dot(
          polygon.const_edge(i).line().d()), 0);
  }
}

TEST(ConvexPolygon, CounterClockwiseSquareEdges) {
  Point3 input[] = {
    Point3(0, 0, 10),
    Point3(1, 0, 10),
    Point3(1, 1, 10),
    Point3(0, 1, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(input);

  ASSERT_EQ(polygon.vertex_count(), std::end(input) - std::begin(input));
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    EXPECT_EQ(polygon.vertex(i), input[i]);
    const HomoPoint3& vertex = polygon.vertex(i);
    const HomoPoint3& next_vertex = polygon.vertex(
        (i + 1) % polygon.vertex_count());
    const Point3& input_vertex = input[i];
    const Point3& next_input_vertex = input[
        (i + 1) % polygon.vertex_count()];
    EXPECT_TRUE(polygon.const_edge(i).line().IsCoincident(vertex));
    EXPECT_TRUE(polygon.const_edge(i).line().IsCoincident(next_vertex));
    EXPECT_TRUE(polygon.const_edge(i).line().IsCoincident(input_vertex));
    EXPECT_TRUE(polygon.const_edge(i).line().IsCoincident(next_input_vertex));
    EXPECT_GT(
        (next_input_vertex - input_vertex).Dot(
          polygon.const_edge(i).line().d()), 0);
  }
}

TEST(ConvexPolygon, ClockwiseSquareEdges) {
  Point3 input[] = {
    Point3(0, 0, 10),
    Point3(0, 1, 10),
    Point3(1, 1, 10),
    Point3(1, 0, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(input);

  ASSERT_EQ(polygon.vertex_count(), std::end(input) - std::begin(input));
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    EXPECT_EQ(polygon.vertex(i), input[i]);
    const HomoPoint3& vertex = polygon.vertex(i);
    const HomoPoint3& next_vertex = polygon.vertex(
        (i + 1) % polygon.vertex_count());
    const Point3& input_vertex = input[i];
    const Point3& next_input_vertex = input[
        (i + 1) % polygon.vertex_count()];
    EXPECT_TRUE(polygon.const_edge(i).line().IsCoincident(vertex));
    EXPECT_TRUE(polygon.const_edge(i).line().IsCoincident(next_vertex));
    EXPECT_TRUE(polygon.const_edge(i).line().IsCoincident(input_vertex));
    EXPECT_TRUE(polygon.const_edge(i).line().IsCoincident(next_input_vertex));
    EXPECT_GT(
        (next_input_vertex - input_vertex).Dot(
          polygon.const_edge(i).line().d()), 0);
  }
}

TEST(ConvexPolygon, RedundantEdges) {
  Point3 input[] = {
    Point3(0, 0, 10),
    Point3(1, 0, 10), // collinear
    Point3(2, 0, 10), // collinear
    Point3(3, 0, 10),
    Point3(3, 1, 10),
    Point3(2, 1, 10), // collinear
    Point3(1, 1, 10), // collinear
    Point3(0, 1, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(input);
  EXPECT_TRUE(polygon.IsValidState());

  ASSERT_EQ(polygon.vertex_count(), std::end(input) - std::begin(input));
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    EXPECT_EQ(polygon.vertex(i), input[i]);
    const HomoPoint3& vertex = polygon.vertex(i);
    const HomoPoint3& next_vertex = polygon.vertex(
        (i + 1) % polygon.vertex_count());
    const Point3& input_vertex = input[i];
    const Point3& next_input_vertex = input[
        (i + 1) % polygon.vertex_count()];
    EXPECT_TRUE(polygon.const_edge(i).line().IsCoincident(vertex));
    EXPECT_TRUE(polygon.const_edge(i).line().IsCoincident(next_vertex));
    EXPECT_TRUE(polygon.const_edge(i).line().IsCoincident(input_vertex));
    EXPECT_TRUE(polygon.const_edge(i).line().IsCoincident(next_input_vertex));
    EXPECT_GT(
        (next_input_vertex - input_vertex).Dot(
          polygon.const_edge(i).line().d()), 0);
  }
}

TEST(ConvexPolygon, ClockwiseRedundantEdges) {
  Point3 input[] = {
    Point3(-3, 0, 10),
    Point3(-3, 1, 10),
    Point3(-2, 1, 10), // collinear
    Point3(-1, 1, 10), // collinear
    Point3(0, 1, 10),
    Point3(0, 0, 10),
    Point3(-1, 0, 10), // collinear
    Point3(-2, 0, 10), // collinear
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(input);
  EXPECT_TRUE(polygon.IsValidState());

  ASSERT_EQ(polygon.vertex_count(), std::end(input) - std::begin(input));
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    EXPECT_EQ(polygon.vertex(i), input[i]) << i;
  }
}

TEST(ConvexPolygon, GetOppositeEdgeIndicesBisectStartPerp) {
  // Test GetOppositeEdgeIndicesBisect on a convex polygon where the 0th edge
  // is perpendicular to the vector, and there are a few collinear edges around
  // the 0th edge.

  //
  // p[1] <- p[0] <- p[7] <- p[6]
  //                                ^
  //  |                       ^     | vector
  //  v                       |     |
  //
  // p[2] -> p[3] -> p[4] -> p[5]
  Point3 p[] = {
    /*p[0]=*/Point3(1, 1, 10),
    /*p[1]=*/Point3(0, 1, 10),
    /*p[2]=*/Point3(0, 0, 10),
    /*p[3]=*/Point3(1, 0, 10),
    /*p[4]=*/Point3(2, 0, 10),
    /*p[5]=*/Point3(3, 0, 10),
    /*p[6]=*/Point3(3, 1, 10),
    /*p[7]=*/Point3(2, 1, 10),
  };
  ConvexPolygon<> polygon = MakeConvexPolygon(p);

  Vector2 vector(0, 1);
  std::pair<size_t, size_t> opp_edges = polygon.GetOppositeEdgeIndicesBisect(
      vector, /*drop_dimension=*/2);

  EXPECT_GT(polygon.const_edge(opp_edges.first).line().d().DropDimension(2)
                   .Dot(vector), 0);
  EXPECT_LT(polygon.const_edge(opp_edges.second).line().d().DropDimension(2)
                   .Dot(vector), 0);

  vector.Negate();
  opp_edges = polygon.GetOppositeEdgeIndicesBisect(vector,
                                                   /*drop_dimension=*/2);
  EXPECT_GT(polygon.const_edge(opp_edges.first).line().d().DropDimension(2)
                   .Dot(vector), 0);
  EXPECT_LT(polygon.const_edge(opp_edges.second).line().d().DropDimension(2)
                   .Dot(vector), 0);
}

TEST(ConvexPolygon, GetOppositeEdgeIndicesBisect) {
  //
  // p[99] <- ... <- p[2] <- p[1]
  //
  //  \                     /     <-- vector
  //   ----       ----------
  //       \     /
  //         p[0]
  Point3 p[100] = {
    /*p[0]=*/Point3(1, 0, 10),
  };

  for (int i = 1; i < 100; ++i) {
    p[i] = Point3(99 - i, 1, 10);
  }

  const Vector2 vector(-1, 0);
  for (size_t offset = 0; offset < sizeof(p)/sizeof(p[0]); ++offset) {
    ConvexPolygon<> polygon = MakeConvexPolygon(p);

    std::pair<size_t, size_t> opp_edges = polygon.GetOppositeEdgeIndicesBisect(
        vector, /*drop_dimension=*/2);

    EXPECT_GT(polygon.const_edge(opp_edges.first).line().d().DropDimension(2)
                     .Dot(vector), 0);
    EXPECT_LT(polygon.const_edge(opp_edges.second).line().d().DropDimension(2)
                     .Dot(vector), 0);

    std::rotate(std::begin(p), std::begin(p) + 1, std::end(p));
  }
}

TEST(ConvexPolygon, GetOppositeEdgeIndicesCWMidSameDir) {
  //
  //  V |         |
  //    v         |
  //              |
  // p[0]         |
  // ^   \        |
  // |    p[1]    |
  //  \   |       |
  //  |    \      |
  //  |    p[2]   |
  //   \  /       |
  //    p[3]      |
  //
  std::vector<Point3> p{
    Point3(0, 0, 0),
    Point3(1, -1, 0),
    Point3(2, -3, 0),
    Point3(1, -4, 0),
  };
  ConvexPolygon<> polygon = MakeConvexPolygon(p);

  Vector2 vector(0, -1);
  std::pair<size_t, size_t> opp_edges = polygon.GetOppositeEdgeIndicesBisect(
      vector, /*drop_dimension=*/2);
  const auto& edge1_dir = polygon.const_edge(opp_edges.first).line().d();
  const auto& edge2_dir = polygon.const_edge(opp_edges.second).line().d();
  EXPECT_GT(vector.Dot(edge1_dir.DropDimension(2)), 0);
  EXPECT_LT(vector.Dot(edge2_dir.DropDimension(2)), 0);
}

TEST(ConvexPolygon, GetOppositeEdgeIndicesCCWMidSameDir) {
  //
  //  V ->                 |
  //                       |
  // p[0] <------          |
  //   \         \         |
  //   p[1]      p[3]      |
  //     \        /        |
  //      --> p[2]         |
  //
  std::vector<Point3> p{
    Point3(0, 0, 0),
    Point3(1, -1, 0),
    Point3(3, -2, 0),
    Point3(4, -1, 0),
  };
  ConvexPolygon<> polygon = MakeConvexPolygon(p);

  Vector2 vector(1, 0);
  std::pair<size_t, size_t> opp_edges = polygon.GetOppositeEdgeIndicesBisect(
      vector, /*drop_dimension=*/2);

  const auto& edge1_dir = polygon.const_edge(opp_edges.first).line().d();
  const auto& edge2_dir = polygon.const_edge(opp_edges.second).line().d();
  EXPECT_LT(edge1_dir.DropDimension(2).Dot(vector) *
            edge2_dir.DropDimension(2).Dot(vector).GetSign(), 0);
}

TEST(ConvexPolygon, GetOppositeEdgeIndicesLargeInts) {
  std::vector<HomoPoint3> p{
    HomoPoint3(98983800000, 478250000000, -496997400000, -165665800000),
    HomoPoint3(1237096350000, 1110633380000, 1422518250000, 474172750000),
    HomoPoint3(193810800000, 433923320000, 468159600000, 156053200000),
    HomoPoint3(0, 493174500000, 506610000000, 168870000000),
    HomoPoint3(-193810800000, 433923320000, 468159600000, 156053200000),
    HomoPoint3(1237096350000, -1110633380000, -1422518250000, -474172750000),
    HomoPoint3(1555715850000, 1413200000000, -1810284150000, -603428050000),
    HomoPoint3(281623200000, 361828700000, -421395600000, -140465200000),
  };

  HalfSpace3 plane(0, 0, 10000, 30000);
  {
    ConvexPolygon<> polygon(plane, 2, p);

    Vector2 vector(-92360000, -2450000);
    std::pair<size_t, size_t> opp_edges = polygon.GetOppositeEdgeIndicesBisect(
        vector, /*drop_dimension=*/2);
    EXPECT_EQ(opp_edges.first, 4);
    EXPECT_EQ(opp_edges.second, 0);
  }

  // Reduce everything to smaller coordinates and rerun the test, to double
  // check that the expected values are correct.
  for (HomoPoint3& point : p) {
    point.Reduce();
  }
  plane.Reduce();

  {
    ConvexPolygon<> polygon(plane, 2, p);

    Vector2 vector(-9236, -245);
    std::pair<size_t, size_t> opp_edges = polygon.GetOppositeEdgeIndicesBisect(
        vector, /*drop_dimension=*/2);
    EXPECT_EQ(opp_edges.first, 4);
    EXPECT_EQ(opp_edges.second, 0);
  }
}

TEST(ConvexPolygon, GetOppositeEdgeIndicesDenom) {
  // This test modifies one the HomoPoint3s at a time to include a
  // constant multiplier to all of the components. This constant
  // multiplier should have no effect on the result, because it is
  // applied to the both the x, y, and z components and the w component.
  //
  //  V ->               |
  //                     |
  // p[0] <------        |
  //   \         \       |
  //   p[1]      p[3]    |
  //     \        /      |
  //      --> p[2]       |
  //
  std::vector<HomoPoint3> p{
    HomoPoint3(101, -101, 0, 1),
    HomoPoint3(103, -102, 0, 1),
    HomoPoint3(104, -101, 0, 1),
    HomoPoint3(100, 100, 0, 1),
  };

  HalfSpace3 plane(0, 0, 1, 0);

  for (size_t i = 0; i < p.size(); ++i) {
    for (int multiple : {-1, 1000, -1000}) {
      std::vector<HomoPoint3> p_copy(p);
      p_copy[i].x() *= multiple;
      p_copy[i].y() *= multiple;
      p_copy[i].z() *= multiple;
      p_copy[i].w() *= multiple;

      ConvexPolygon<> polygon(plane, /*drop_dimension=*/2, p_copy);

      Vector2 vector(1, 0);
      std::pair<size_t, size_t> opp_edges =
        polygon.GetOppositeEdgeIndicesBisect(
            vector, /*drop_dimension=*/2);
      const auto& edge1_dir = polygon.const_edge(opp_edges.first).line().d();
      const auto& edge2_dir = polygon.const_edge(opp_edges.second).line().d();
      EXPECT_LT(edge1_dir.DropDimension(2).Dot(vector) *
                edge2_dir.DropDimension(2).Dot(vector).GetSign(), 0);
    }
  }
}

TEST(ConvexPolygon, GetGreaterCycleIndex) {
  Point3 input[] = {
    Point3(0, 0, 10),
    Point3(0, 1, 10),
    Point3(1, 1, 10),
    Point3(1, 0, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(input);

  for (size_t a = 0; a < polygon.vertex_count(); ++a) {
    for (size_t b = 0; b < polygon.vertex_count(); ++b) {
      size_t ret = polygon.GetGreaterCycleIndex(a, b);
      EXPECT_GE(ret, a);
      EXPECT_LT(ret, a + polygon.vertex_count());
      EXPECT_EQ(ret % polygon.vertex_count(), b);
    }
  }
}

TEST(ConvexPolygon, CounterClockwiseSquareGetExtremeIndexBisect) {
  // p[3] <- p[2]
  //  |       ^
  //  v       |
  // p[0] -> p[1]
  Point3 p[] = {
    Point3(0, 0, 10),
    Point3(1, 0, 10),
    Point3(1, 1, 10),
    Point3(0, 1, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(p);

  Vector2 to_corner[] = {
    Vector2(-1, -1),
    Vector2(1, -1),
    Vector2(1, 1),
    Vector2(-1, 1),
  };

  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(polygon.GetExtremeIndexBisect(to_corner[i],
                                            /*drop_dimension=*/2), i);
  }

  Vector2 along_edges[] = {
    Vector2(0, -1),
    Vector2(1, 0),
    Vector2(0, 1),
    Vector2(-1, 0),
  };

  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(polygon.GetExtremeIndexBisect(along_edges[i],
                                            /*drop_dimension=*/2), i);
  }
}

TEST(ConvexPolygon, CounterClockwiseSquareGetPosSideVertex) {
  // p[3] <- p[2]
  //  |       ^
  //  v       |
  // p[0] -> p[1]
  Point3 p[] = {
    Point3(0, 0, 10),
    Point3(1, 0, 10),
    Point3(1, 1, 10),
    Point3(0, 1, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(p);

  HalfSpace2 only_corner[] = {
    HalfSpace2(p[1].DropDimension(2), p[3].DropDimension(2)),
    HalfSpace2(p[2].DropDimension(2), p[0].DropDimension(2)),
    HalfSpace2(p[3].DropDimension(2), p[1].DropDimension(2)),
    HalfSpace2(p[0].DropDimension(2), p[2].DropDimension(2)),
  };

  for (int i = 0; i < 4; ++i) {
    std::pair<size_t, size_t> dir_indices =
      polygon.GetOppositeEdgeIndicesBisect(only_corner[i].normal(),
                                           /*drop_dimension=*/2);
    EXPECT_EQ(polygon.GetPosSideVertex(only_corner[i],
                                       /*drop_dimension=*/2,
                                       dir_indices.first,
                                       dir_indices.second).first, 1);
    EXPECT_EQ(polygon.GetPosSideVertex(only_corner[i],
                                       /*drop_dimension=*/2,
                                       dir_indices.first,
                                       dir_indices.second).second, i);
  }

  // p[3] <- p[2]
  //  |       ^
  //  v       |
  // p[0] -> p[1]
  HalfSpace2 on_plane[] = {
    HalfSpace2(p[1].DropDimension(2), p[0].DropDimension(2)),
    HalfSpace2(p[2].DropDimension(2), p[1].DropDimension(2)),
    HalfSpace2(p[3].DropDimension(2), p[2].DropDimension(2)),
    HalfSpace2(p[0].DropDimension(2), p[3].DropDimension(2)),
  };

  for (int i = 0; i < 4; ++i) {
    std::pair<size_t, size_t> dir_indices =
      polygon.GetOppositeEdgeIndicesBisect(only_corner[i].normal(),
                                           /*drop_dimension=*/2);
    EXPECT_EQ(polygon.GetPosSideVertex(on_plane[i],
                                       /*drop_dimension=*/2,
                                       dir_indices.first,
                                       dir_indices.second).first, 0);
    EXPECT_EQ(polygon.GetPosSideVertex(on_plane[i],
                                       /*drop_dimension=*/2,
                                       dir_indices.first,
                                       dir_indices.second).second, i);
  }
}

TEST(ConvexPolygon, CounterClockwiseSquareGetNegSideVertex) {
  // p[3] <- p[2]
  //  |       ^
  //  v       |
  // p[0] -> p[1]
  Point3 p[] = {
    Point3(0, 0, 10),
    Point3(1, 0, 10),
    Point3(1, 1, 10),
    Point3(0, 1, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(p);

  HalfSpace2 only_corner[] = {
    HalfSpace2(p[3].DropDimension(2), p[1].DropDimension(2)),
    HalfSpace2(p[0].DropDimension(2), p[2].DropDimension(2)),
    HalfSpace2(p[1].DropDimension(2), p[3].DropDimension(2)),
    HalfSpace2(p[2].DropDimension(2), p[0].DropDimension(2)),
  };

  for (int i = 0; i < 4; ++i) {
    std::pair<size_t, size_t> dir_indices =
      polygon.GetOppositeEdgeIndicesBisect(only_corner[i].normal(),
                                           /*drop_dimension=*/2);
    EXPECT_EQ(polygon.GetNegSideVertex(only_corner[i],
                                       /*drop_dimension=*/2,
                                       dir_indices.first,
                                       dir_indices.second).first, -1);
    EXPECT_EQ(polygon.GetNegSideVertex(only_corner[i],
                                       /*drop_dimension=*/2,
                                       dir_indices.first,
                                       dir_indices.second).second, i);
  }

  // p[3] <- p[2]
  //  |       ^
  //  v       |
  // p[0] -> p[1]
  HalfSpace2 on_plane[] = {
    HalfSpace2(p[0].DropDimension(2), p[1].DropDimension(2)),
    HalfSpace2(p[1].DropDimension(2), p[2].DropDimension(2)),
    HalfSpace2(p[2].DropDimension(2), p[3].DropDimension(2)),
    HalfSpace2(p[3].DropDimension(2), p[0].DropDimension(2)),
  };

  for (int i = 0; i < 4; ++i) {
    std::pair<size_t, size_t> dir_indices =
      polygon.GetOppositeEdgeIndicesBisect(only_corner[i].normal(),
                                           /*drop_dimension=*/2);
    EXPECT_EQ(polygon.GetNegSideVertex(on_plane[i],
                                       /*drop_dimension=*/2,
                                       dir_indices.first,
                                       dir_indices.second).first, 0);
    EXPECT_EQ(polygon.GetNegSideVertex(on_plane[i],
                                       /*drop_dimension=*/2,
                                       dir_indices.first,
                                       dir_indices.second).second, i);
  }
}

TEST(ConvexPolygon, ClockwiseSquareGetExtremeIndexBisect) {
  // p[1] -> p[2]
  //  ^       |
  //  |       v
  // p[0] <- p[3]
  Point3 p[] = {
    Point3(0, 0, 10),
    Point3(0, 1, 10),
    Point3(1, 1, 10),
    Point3(1, 0, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(p);

  Vector2 to_corner[] = {
    Vector2(-1, -1),
    Vector2(-1, 1),
    Vector2(1, 1),
    Vector2(1, -1),
  };

  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(polygon.GetExtremeIndexBisect(to_corner[i],
                                            /*drop_dimension=*/2), i);
  }

  Vector2 along_edges[] = {
    Vector2(-1, 0),
    Vector2(0, 1),
    Vector2(1, 0),
    Vector2(0, -1),
  };

  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(polygon.GetExtremeIndexBisect(along_edges[i],
                                            /*drop_dimension=*/2), i);
  }
}

TEST(ConvexPolygon, CounterClockwiseSquareGetExtremeIndexBisect3D) {
  // p[3] <- p[2]
  //  |       ^
  //  v       |
  // p[0] -> p[1]
  Point3 p[] = {
    Point3(0, 0, 0),
    Point3(1, 0, 10),
    Point3(1, 1, 10),
    Point3(0, 1, 0),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(p);

  Vector3 to_corner[] = {
    Vector3(-1, -1, -1),
    Vector3(1, -1, 1),
    Vector3(1, 1, 1),
    Vector3(-1, 1, -1),
  };

  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(polygon.GetExtremeIndexBisect(to_corner[i]), i);
  }
}

TEST(ConvexPolygon, ClockwiseSquareGetExtremeIndexBisect3D) {
  // p[1] -> p[2]
  //  ^       |
  //  |       v
  // p[0] <- p[3]
  Point3 p[] = {
    Point3(0, 0, 0),
    Point3(0, 1, 10),
    Point3(1, 1, 10),
    Point3(1, 0, 0),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(p);
  EXPECT_LT(polygon.plane().normal().z(), 0);

  Vector3 to_corner[] = {
    Vector3(-1, -1, -1),
    Vector3(-1, 1, 1),
    Vector3(1, 1, 1),
    Vector3(1, -1, -1),
  };

  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(polygon.GetExtremeIndexBisect(to_corner[i]), i);
  }
}

TEST(ConvexPolygon, GetExtremeIndexBisect3DIsNot2DProjection) {
  // This test case shows that the GetExtremeIndexBisect is not as simple as
  // projecting the vector to 2D.
  //
  // If the polygon was rotated to be flat on the XY plane, it would look like
  // this:
  //
  //      p[3]          |
  //   /       \        |
  // p[4]      p[2]     |
  //  |          ^      |
  //  |          |      |
  //  v          |      |
  // p[0] ---> p[1]     |
  //
  // However, dropping the X dimension (so that Y and Z are remaining, and Z
  // was a fraction of X) also effectively non-uniformly scales the polygon,
  // squishing horizontally so that it looks like this:
  //
  //      p[3]          |
  //    /     \         |
  //   /       \        |
  // p[4]      p[2]     |
  //  |          ^      |
  //  |          |      |
  //  |          |      |
  //  |          |      |
  //  |          |      |
  //  v          |      |
  // p[0] ---> p[1]     |
  //
  // So for a vector from p[0] to p[2], in the first polygon, p[2] is the
  // extreme index, but for the below diagram, p[3] is the extreme index. This
  // happens because distances lines are effectively drawn perpendicular to the
  // vector, and the non-uniform scaling does not update the 90 degree angle.
  // So whether the 90 degree angle is taken before or after the non-uniform
  // scaling affects the result.

  Point3 p[] = {
    Point3(0, 0, 0),
    Point3(40, 0, 4),
    Point3(40, 40, 4),
    Point3(20, 45, 2),
    Point3(0, 40, 0),
  };
  ConvexPolygon<> polygon = MakeConvexPolygon(p);

  EXPECT_EQ(polygon.GetExtremeIndexBisect(p[2] - p[0]), 2);

  EXPECT_EQ(polygon.GetExtremeIndexBisect((p[2] - p[0]).DropDimension(0),
                                          /*drop_dimension=*/0), 3);
}

TEST(ConvexPolygon, GetLastNegSideVertexOnPlane) {
  // p[8] <- p[7] <- p[6] <- p[5]
  //  |                       ^     ^
  //  v                       |     | pos half-space
  // p[9] ------------------ p[4] ------------------
  //  |                       ^
  //  v                       |
  // p[0] -> p[1] -> p[2] -> p[3]
  Point3 p[] = {
    /*p[0]=*/Point3(0, 0, 10),
    /*p[1]=*/Point3(1, 0, 10),
    /*p[2]=*/Point3(2, 0, 10),
    /*p[3]=*/Point3(3, 0, 10),
    /*p[4]=*/Point3(3, 1, 10),
    /*p[5]=*/Point3(3, 2, 10),
    /*p[6]=*/Point3(2, 2, 10),
    /*p[7]=*/Point3(1, 2, 10),
    /*p[8]=*/Point3(0, 2, 10),
    /*p[9]=*/Point3(0, 1, 10),
  };

  const ConvexPolygon<> polygon = MakeConvexPolygon(p);
  HalfSpace2 half_space(p[9].DropDimension(2), p[4].DropDimension(2));

  for (size_t neg_side_index = 0; neg_side_index < 4; neg_side_index++) {
    for (size_t pos_side_index = 5; pos_side_index <= 9; pos_side_index++) {
      auto result = polygon.GetLastNegSideVertex(half_space,
                                                 /*drop_dimension=*/2,
                                                 neg_side_index,
                                                 /*neg_side_type=*/-1,
                                                 pos_side_index);
      EXPECT_EQ(result.first, 0);
      EXPECT_EQ(result.second, 4);
    }
  }
  auto result = polygon.GetLastNegSideVertex(half_space, /*drop_dimension=*/2,
                                             /*neg_side_index=*/9,
                                             /*neg_side_type=*/0,
                                             /*pos_side_index=*/7);
  EXPECT_EQ(result.first, 0);
  EXPECT_EQ(result.second, 4);
  result = polygon.GetLastNegSideVertex(half_space, /*drop_dimension=*/2,
                                        /*neg_side_index=*/4,
                                        /*neg_side_type=*/0,
                                        /*pos_side_index=*/7);
  EXPECT_EQ(result.first, 0);
  EXPECT_EQ(result.second, 4);
}

TEST(ConvexPolygon, GetLastNegSideVertex) {
  // p[8] <- p[7] <- p[6] <- p[5]   ^
  //  |                       ^     | pos half-space
  //  | --------------------- | --------------------
  //  v                       |
  // p[9]                    p[4]
  //  |                       ^
  //  v                       |
  // p[0] -> p[1] -> p[2] -> p[3]
  Point3 p[] = {
    /*p[0]=*/Point3(0, 0, 10),
    /*p[1]=*/Point3(1, 0, 10),
    /*p[2]=*/Point3(2, 0, 10),
    /*p[3]=*/Point3(3, 0, 10),
    /*p[4]=*/Point3(3, 1, 10),
    /*p[5]=*/Point3(3, 3, 10),
    /*p[6]=*/Point3(2, 3, 10),
    /*p[7]=*/Point3(1, 3, 10),
    /*p[8]=*/Point3(0, 3, 10),
    /*p[9]=*/Point3(0, 1, 10),
  };

  const ConvexPolygon<> polygon = MakeConvexPolygon(p);
  HalfSpace2 half_space(Point2(0, 2), Point2(1, 2));

  for (size_t neg_side_index = 9; neg_side_index <= 9 + 4; neg_side_index++) {
    for (size_t pos_side_index = 5; pos_side_index <= 8; pos_side_index++) {
      auto result = polygon.GetLastNegSideVertex(half_space,
          /*drop_dimension=*/2, neg_side_index % polygon.vertex_count(),
          /*neg_side_type=*/-1, pos_side_index);
      EXPECT_EQ(result.first, -1);
      EXPECT_EQ(result.second, 4);
    }
  }
}

struct TestEdgeInfo : public EdgeInfoRoot {
  TestEdgeInfo() = default;
  explicit TestEdgeInfo(const EdgeInfoRoot&) { }

  bool operator!=(const EdgeInfoRoot&) const {
    return false;
  }

  bool operator!=(const TestEdgeInfo& other) const {
    return on_split != other.on_split;
  }

  bool on_split = false;
};

std::ostream& operator<<(std::ostream& out, const TestEdgeInfo& data) {
  out << data.on_split;
  return out;
}

TEST(ConvexPolygon, ConvertVertexData) {
  Point3 input[] = {
    Point3(0, 0, 10),
    Point3(1, 0, 10),
    Point3(1, 1, 10),
  };

  ConvexPolygon<> polygon_raw = MakeConvexPolygon(input);
  MutableConvexPolygon<TestEdgeInfo> polygon(polygon_raw);

  EXPECT_EQ(polygon, polygon_raw);
  ASSERT_EQ(polygon.vertex_count(), 3);

  EXPECT_FALSE(polygon.edge(0).on_split);
  polygon.edge(0).on_split = true;
  EXPECT_TRUE(polygon.edge(0).on_split);
}

using FindSplitRangesFunc = ConvexPolygonSplitRanges (ConvexPolygon<>::*)(
    const HalfSpace2&, int) const;

// Overload the << operator for FindSplitRangesFunc so that Google Test doesn't
// pick very long test name.
std::ostream& operator<<(std::ostream& out, FindSplitRangesFunc func) {
  if (func == &ConvexPolygon<>::FindSplitRangesBisect) {
    out << "Bisect";
  } else if (func == &ConvexPolygon<>::FindSplitRangesLinear) {
    out << "Linear";
  } else {
    out << "Unknown";
  }
  return out;
}

class ConvexPolygonFindSplitRanges :
  public testing::TestWithParam<FindSplitRangesFunc> {
};

TEST_P(ConvexPolygonFindSplitRanges, OnPosSide) {
  Point3 input[2][3] = {
    // counter-clockwise
    {
      Point3(0, 0, 10),
      Point3(1, 0, 10),
      Point3(1, 1, 10),
    },
    // clockwise
    {
      Point3(0, 0, 10),
      Point3(1, 1, 10),
      Point3(1, 0, 10),
    }
  };

  for (int i = 0; i < 2; ++i) {
    MutableConvexPolygon<> polygon(MakeConvexPolygon(input[i]));

    // Test where the polygon is completely in the positive half-space.
    {
      // All points with x>-1 are on the positive side of the half-space from
      // projecting this line to 2D by dropping the Z coordinate.
      Point3 p1(-1, 1, 0);
      Point3 p2(-1, 0, 0);
      PluckerLine line(p1, p2);

      ConvexPolygonSplitRanges indices =
        (polygon.*GetParam())(line.Project2D(/*drop_dimension=*/2),
                              /*drop_dimension=*/2);
      EXPECT_FALSE(indices.ShouldEmitNegativeChild());
      EXPECT_TRUE(indices.ShouldEmitPositiveChild());
      // There are 0 vertices on the plane.
      EXPECT_EQ(indices.pos_range.second,
                indices.pos_range.first + polygon.vertex_count());
    }

    // Test where the polygon is in the positive half-space, but vertex 0
    // touches the line.
    {
      // All points with x>0 are on the positive side of the half-space from
      // projecting this line to 2D by dropping the Z coordinate.
      Point3 p1(0, 1, 0);
      Point3 p2(0, 0, 0);
      PluckerLine line(p1, p2);

      ConvexPolygonSplitRanges indices =
        (polygon.*GetParam())(line.Project2D(/*drop_dimension=*/2),
                              /*drop_dimension=*/2);
      EXPECT_FALSE(indices.ShouldEmitNegativeChild());
      EXPECT_TRUE(indices.ShouldEmitPositiveChild());
      polygon.SortVertices();
      // pos_range.second points to the vertex on the plane, which is index 0.
      EXPECT_EQ(indices.pos_range.second % polygon.vertex_count(), 0);
      // There is 1 vertex on the plane.
      EXPECT_EQ(indices.pos_range.second,
                indices.pos_range.first + polygon.vertex_count() - 1);
    }
  }
}

TEST_P(ConvexPolygonFindSplitRanges, AtExistingVertices) {
  //
  // p[3] <--- p[2]
  //  | pos -/   ^
  //  |   -/     |
  //  v  /       |
  // p[0] ---> p[1]
  //
  Point3 p[] = {
    /*p[0]=*/Point3(0, 0, 10),
    /*p[1]=*/Point3(1, 0, 10),
    /*p[2]=*/Point3(1, 1, 10),
    /*p[3]=*/Point3(0, 1, 10),
  };

  PluckerLine line(p[0], p[2]);

  ConvexPolygon<> polygon(MakeConvexPolygon(p));

  // Run it in a loop a few times to get a sense of how fast the
  // function is from the overall test time.
  ConvexPolygonSplitRanges indices;
  for (int i = 0; i < 10000; ++i) {
    indices =
      (polygon.*GetParam())(line.Project2D(/*drop_dimension=*/2),
                            /*drop_dimension=*/2);
  }
  EXPECT_TRUE(indices.ShouldEmitNegativeChild());
  EXPECT_TRUE(indices.ShouldEmitPositiveChild());
  EXPECT_EQ(indices.neg_range.first, 1);
  EXPECT_EQ(indices.neg_range.second, 2);
  EXPECT_EQ(indices.pos_range.first, 3);
  EXPECT_EQ(indices.pos_range.second, 4);
}

TEST_P(ConvexPolygonFindSplitRanges, AtNewVertices) {
  //
  // p[3] <--- p[2]  pos
  //  |          ^    ^
  //  v          |    |
  // n[0] ---- n[1] -----
  //  |          ^
  //  v          |
  // p[0] ---> p[1]
  //
  Point3 p[] = {
    /*p[0]=*/Point3(0, 0, 0),
    /*p[1]=*/Point3(1, 0, 1),
    /*p[2]=*/Point3(1, 2, 3),
    /*p[3]=*/Point3(0, 2, 2),
  };

  Point3 n[] = {
    /*n[0]=*/Point3(0, 1, 1),
    /*n[1]=*/Point3(1, 1, 2),
  };

  ConvexPolygon<> polygon(MakeConvexPolygon(p));

  for (int drop_dimension = 0; drop_dimension < 3; ++drop_dimension) {
    PluckerLine line;
    EXPECT_FALSE(
        polygon.plane().normal().components()[drop_dimension].IsZero());
    if (polygon.plane().normal().components()[drop_dimension] < 0) {
      line = PluckerLine(n[1], n[0]);
    } else {
      line = PluckerLine(n[0], n[1]);
    }

    ConvexPolygonSplitRanges indices =
      (polygon.*GetParam())(line.Project2D(drop_dimension),
                            drop_dimension);
    EXPECT_TRUE(indices.ShouldEmitNegativeChild());
    EXPECT_TRUE(indices.ShouldEmitPositiveChild());
    EXPECT_EQ(indices.neg_range.first % polygon.vertex_count(), 0);
    EXPECT_EQ(indices.neg_range.second % polygon.vertex_count(), 2);
    EXPECT_EQ(indices.pos_range.first, 2);
    EXPECT_EQ(indices.pos_range.second, 4);
  }
}

TEST_P(ConvexPolygonFindSplitRanges, AtNewVerticesXPlane) {
  //
  // p[3] <--------- p[2]
  //  |       |       ^
  //  |       |pos->  |
  //  v       |       |
  // p[0] ---------> p[1]
  //
  Point3 p[4] = {
    Point3(0, 0, 10),
    Point3(2, 0, 10),
    Point3(2, 1, 10),
    Point3(0, 1, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(p);
  HalfSpace3 half_space(/*x=*/1, /*y=*/0, /*z=*/0, /*dist=*/1);
  PluckerLine line(half_space, polygon.plane());

  ConvexPolygonSplitRanges indices =
    (polygon.*GetParam())(line.Project2D(polygon.drop_dimension()),
                          polygon.drop_dimension());
  EXPECT_TRUE(indices.ShouldEmitNegativeChild());
  EXPECT_TRUE(indices.ShouldEmitPositiveChild());
  EXPECT_EQ(indices.neg_range.first % polygon.vertex_count(), 3);
  EXPECT_EQ(indices.neg_range.second % polygon.vertex_count(), 1);
  EXPECT_EQ(indices.pos_range.first, 1);
  EXPECT_EQ(indices.pos_range.second, 3);
  EXPECT_GT(line.Project2D(polygon.drop_dimension()).Compare(
        polygon.vertex(indices.pos_range.first).DropDimension(
          polygon.drop_dimension())), 0);
}

INSTANTIATE_TEST_SUITE_P(, ConvexPolygonFindSplitRanges,
    testing::Values(&ConvexPolygon<>::FindSplitRangesBisect,
                    &ConvexPolygon<>::FindSplitRangesLinear));

TEST(ConvexPolygon, SplitOnPlane) {
  Point3 input[] = {
    Point3(0, 0, 10),
    Point3(1, 0, 11),
    Point3(1, 1, 12),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(input);
  {
    auto info = polygon.GetSplitInfo(polygon.plane());
    EXPECT_FALSE(info.ShouldEmitNegativeChild());
    EXPECT_FALSE(info.ShouldEmitPositiveChild());
    EXPECT_TRUE(info.ShouldEmitOnPlane());
  }

  {
    auto info = polygon.GetSplitInfo(-polygon.plane());
    EXPECT_FALSE(info.ShouldEmitNegativeChild());
    EXPECT_FALSE(info.ShouldEmitPositiveChild());
    EXPECT_TRUE(info.ShouldEmitOnPlane());
  }
}

// Helper for a polygon that is expected to split into 2 pieces
void SplitHelper(const ConvexPolygon<>& polygon,
                 const HalfSpace3& half_space,
                 MutableConvexPolygon<>& neg_side,
                 MutableConvexPolygon<>& pos_side) {
  auto info = polygon.GetSplitInfo(half_space);
  ASSERT_TRUE(info.ShouldEmitNegativeChild());
  ASSERT_TRUE(info.ShouldEmitPositiveChild());

  std::pair<ConvexPolygon<>, ConvexPolygon<>> children =
    polygon.CreateSplitChildren(info);
  neg_side = std::move(children.first);
  pos_side = std::move(children.second);
  EXPECT_EQ(neg_side.plane(), polygon.plane());
  EXPECT_EQ(pos_side.plane(), polygon.plane());

  ASSERT_GE(neg_side.vertex_count(), 3);
  EXPECT_TRUE(half_space.IsCoincident(neg_side.vertex(
          neg_side.vertex_count() - 2)));
  EXPECT_TRUE(half_space.IsCoincident(neg_side.vertex(
          neg_side.vertex_count() - 1)));
  for (size_t i = 0; i < neg_side.vertex_count() - 2; ++i) {
    EXPECT_FALSE(half_space.IsCoincident(neg_side.vertex(i)));
  }

  ASSERT_GE(pos_side.vertex_count(), 3);
  EXPECT_TRUE(half_space.IsCoincident(pos_side.vertex(0)));
  EXPECT_TRUE(half_space.IsCoincident(pos_side.vertex(
          pos_side.vertex_count() - 1)));
  for (size_t i = 1; i < pos_side.vertex_count() - 1; ++i) {
    EXPECT_FALSE(half_space.IsCoincident(pos_side.vertex(i)));
  }

  for (const ConvexPolygon<>* output : {&neg_side, &pos_side}) {
    for (size_t i = 0; i < output->vertex_count(); ++i) {
      PluckerLine expected_line(
          output->vertex(i), output->vertex((i + 1) % output->vertex_count()));
      EXPECT_EQ(output->const_edge(i).line(), expected_line);
      EXPECT_TRUE(
          output->const_edge(i).line().d().IsSameDir(expected_line.d()));
    }
  }
}

// Helper for a polygon that is expected to split into 2 pieces
void SplitHelper(MutableConvexPolygon<>&& polygon,
                 const HalfSpace3& half_space,
                 MutableConvexPolygon<>& neg_side,
                 MutableConvexPolygon<>& pos_side) {
  auto info = polygon.GetSplitInfo(half_space);
  ASSERT_TRUE(info.ShouldEmitNegativeChild());
  ASSERT_TRUE(info.ShouldEmitPositiveChild());

  std::pair<ConvexPolygon<>, ConvexPolygon<>> children =
    ConvexPolygon<>::CreateSplitChildren(std::move(polygon).GetRValueKey(),
                                         std::move(info));
  neg_side = std::move(children.first);
  pos_side = std::move(children.second);

  ASSERT_GE(neg_side.vertex_count(), 3);
  EXPECT_TRUE(half_space.IsCoincident(neg_side.vertex(
          neg_side.vertex_count() - 2)));
  EXPECT_TRUE(half_space.IsCoincident(neg_side.vertex(
          neg_side.vertex_count() - 1)));
  for (size_t i = 0; i < neg_side.vertex_count() - 2; ++i) {
    EXPECT_FALSE(half_space.IsCoincident(neg_side.vertex(i)));
  }

  ASSERT_GE(pos_side.vertex_count(), 3);
  EXPECT_TRUE(half_space.IsCoincident(pos_side.vertex(0)));
  EXPECT_TRUE(half_space.IsCoincident(pos_side.vertex(
          pos_side.vertex_count() - 1)));
  for (size_t i = 1; i < pos_side.vertex_count() - 1; ++i) {
    EXPECT_FALSE(half_space.IsCoincident(pos_side.vertex(i)));
  }

  for (const ConvexPolygon<>* output : {&neg_side, &pos_side}) {
    for (size_t i = 0; i < output->vertex_count(); ++i) {
      PluckerLine expected_line(
          output->vertex(i), output->vertex((i + 1) % output->vertex_count()));
      EXPECT_EQ(output->const_edge(i).line(), expected_line);
      EXPECT_TRUE(
          output->const_edge(i).line().d().IsSameDir(expected_line.d()));
    }
  }
}

TEST(ConvexPolygon, SplitAtExistingVertices) {
  //
  // p[3] <--- p[2]
  //  | pos -/   ^
  //  |   -/     |
  //  v  /       |
  // p[0] ---> p[1]
  //
  Point3 p[] = {
    /*p[0]=*/Point3(0, 0, 10),
    /*p[1]=*/Point3(1, 0, 10),
    /*p[2]=*/Point3(1, 1, 10),
    /*p[3]=*/Point3(0, 1, 10),
  };

  Point3 neg_side_p[] = { p[0], p[1], p[2] };
  Point3 pos_side_p[] = { p[0], p[2], p[3] };

  Point3 above(0, 0, 11);
  HalfSpace3 half_space(p[0], above, p[2]);

  MutableConvexPolygon<> polygon(MakeConvexPolygon(p));
  ConvexPolygon<> expected_neg_side(MakeConvexPolygon(neg_side_p));
  ConvexPolygon<> expected_pos_side(MakeConvexPolygon(pos_side_p));

  MutableConvexPolygon<> neg_side;
  MutableConvexPolygon<> pos_side;
  SplitHelper(polygon, half_space, neg_side, pos_side);
  EXPECT_EQ(neg_side, expected_neg_side);
  EXPECT_EQ(pos_side, expected_pos_side);

  SplitHelper(polygon, -half_space, pos_side, neg_side);
  EXPECT_EQ(neg_side, expected_neg_side);
  EXPECT_EQ(pos_side, expected_pos_side);

  {
    HalfSpace3 polygon_plane =  polygon.plane();
    ConvexPolygonSplitInfo info = polygon.GetSplitInfo(-half_space);
    std::pair<ConvexPolygon<>, ConvexPolygon<>> children =
      std::move(polygon).CreateSplitChildren(std::move(info));
    EXPECT_EQ(children.first.plane(), polygon_plane);
    EXPECT_EQ(children.second.plane(), polygon_plane);
  }
}

TEST(ConvexPolygon, SplitAtExistingVerticesCW) {
  //
  // p[1] ---> p[2]
  //  ^ pos -/   |
  //  |   -/     |
  //  |  /       v
  // p[0] <--- p[3]
  //
  Point3 p[] = {
    /*p[0]=*/Point3(0, 0, 10),
    /*p[1]=*/Point3(0, 1, 10),
    /*p[2]=*/Point3(1, 1, 10),
    /*p[3]=*/Point3(1, 0, 10),
  };

  Point3 neg_side_p[] = { p[0], p[2], p[3] };
  Point3 pos_side_p[] = { p[0], p[1], p[2] };

  Point3 above(0, 0, 11);
  HalfSpace3 half_space(p[0], above, p[2]);

  ConvexPolygon<> polygon(MakeConvexPolygon(p));
  ConvexPolygon<> expected_neg_side(MakeConvexPolygon(neg_side_p));
  ConvexPolygon<> expected_pos_side(MakeConvexPolygon(pos_side_p));

  MutableConvexPolygon<> neg_side;
  MutableConvexPolygon<> pos_side;
  SplitHelper(polygon, half_space, neg_side, pos_side);
  EXPECT_EQ(neg_side, expected_neg_side);
  EXPECT_EQ(pos_side, expected_pos_side);

  SplitHelper(polygon, -half_space, pos_side, neg_side);
  EXPECT_EQ(neg_side, expected_neg_side);
  EXPECT_EQ(pos_side, expected_pos_side);
}

TEST(ConvexPolygon, SplitAtNewVertices) {
  //
  // p[3] <--------- p[2]
  //  |       |       ^
  //  |       |pos->  |
  //  v       |       |
  // p[0] ---------> p[1]
  //
  Point3 p[4] = {
    Point3(0, 0, 10),
    Point3(2, 0, 10),
    Point3(2, 1, 10),
    Point3(0, 1, 10),
  };

  Point3 neg_side_p[] = {
    p[0],
    Point3(1, 0, 10),
    Point3(1, 1, 10),
    p[3],
  };
  Point3 pos_side_p[] = {
    Point3(1, 0, 10),
    p[1],
    p[2],
    Point3(1, 1, 10),
  };

  HalfSpace3 half_space(/*x=*/1, /*y=*/0, /*z=*/0, /*dist=*/1);

  ConvexPolygon<> polygon(MakeConvexPolygon(p));
  ConvexPolygon<> expected_neg_side(MakeConvexPolygon(neg_side_p));
  ConvexPolygon<> expected_pos_side(MakeConvexPolygon(pos_side_p));

  MutableConvexPolygon<> neg_side;
  MutableConvexPolygon<> pos_side;
  SplitHelper(polygon, half_space, neg_side, pos_side);
  EXPECT_EQ(neg_side, expected_neg_side);
  EXPECT_EQ(pos_side, expected_pos_side);

  SplitHelper(polygon, -half_space, pos_side, neg_side);
  EXPECT_EQ(neg_side, expected_neg_side);
  EXPECT_EQ(pos_side, expected_pos_side);
}

TEST(ConvexPolygon, SplitAtNewVerticesLargeValues) {
  // This is the same as SplitAtNewVertices, except the coordinates are shifted
  // to the left by 64 bits.
  //
  // p[3] <--------- p[2]
  //  |       |       ^
  //  |       |pos->  |
  //  v       |       |
  // p[0] ---------> p[1]
  //
  BigInt big0(0);
  BigInt big1 = BigInt(1) << 64;
  BigInt big2 = BigInt(2) << 64;
  BigInt big10 = BigInt(10) << 64;
  Point3 p[4] = {
    Point3(big0, big0, big10),
    Point3(big2, big0, big10),
    Point3(big2, big1, big10),
    Point3(big0, big1, big10),
  };

  Point3 neg_side_p[] = {
    p[0],
    Point3(big1, big0, big10),
    Point3(big1, big1, big10),
    p[3],
  };
  Point3 pos_side_p[] = {
    Point3(big1, big0, big10),
    p[1],
    p[2],
    Point3(big1, big1, big10),
  };

  HalfSpace3 half_space(/*x=*/BigInt(1), /*y=*/big0, /*z=*/big0,
                        /*dist=*/big1);

  ConvexPolygon<> expected_neg_side(MakeConvexPolygon(neg_side_p));
  ConvexPolygon<> expected_pos_side(MakeConvexPolygon(pos_side_p));

  {
    MutableConvexPolygon<> polygon(MakeConvexPolygon(p));
    MutableConvexPolygon<> neg_side;
    MutableConvexPolygon<> pos_side;
    SplitHelper(std::move(polygon), half_space, neg_side, pos_side);
    EXPECT_EQ(neg_side, expected_neg_side);
    EXPECT_EQ(pos_side, expected_pos_side);
  }

  {
    MutableConvexPolygon<> polygon(MakeConvexPolygon(p));
    MutableConvexPolygon<> neg_side;
    MutableConvexPolygon<> pos_side;
    SplitHelper(std::move(polygon), -half_space, pos_side, neg_side);
    EXPECT_EQ(neg_side, expected_neg_side);
    EXPECT_EQ(pos_side, expected_pos_side);
  }
}

TEST(ConvexPolygon, SplitOnParallelPlane) {
  Point3 ccw_input[] = {
    Point3(0, 0, 10),
    Point3(1, 0, 10),
    Point3(1, 1, 10),
  };

  std::vector<Point3> cw_input(std::begin(ccw_input), std::end(ccw_input));
  std::reverse(cw_input.begin(), cw_input.end());

  ConvexPolygon<> ccw_polygon = MakeConvexPolygon(ccw_input);
  ConvexPolygon<> cw_polygon = MakeConvexPolygon(cw_input);

  HalfSpace3 above_up(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/11);
  EXPECT_TRUE(above_up.normal().IsSameDir(ccw_polygon.plane().normal()));
  {
    auto info = ccw_polygon.GetSplitInfo(above_up);
    EXPECT_TRUE(info.ShouldEmitNegativeChild());
    EXPECT_FALSE(info.ShouldEmitPositiveChild());

    EXPECT_EQ(info.neg_range().first, 0);
    EXPECT_EQ(info.neg_range().second, ccw_polygon.vertex_count());
  }
  {
    auto info = cw_polygon.GetSplitInfo(above_up);
    EXPECT_TRUE(info.ShouldEmitNegativeChild());
    EXPECT_FALSE(info.ShouldEmitPositiveChild());

    EXPECT_EQ(info.neg_range().first, 0);
    EXPECT_EQ(info.neg_range().second, cw_polygon.vertex_count());
  }

  HalfSpace3 above_down(/*x=*/0, /*y=*/0, /*z=*/-1, /*dist=*/-11);
  EXPECT_TRUE(above_down.normal().IsSameDir(-above_up.normal()));

  {
    auto info = ccw_polygon.GetSplitInfo(above_down);
    EXPECT_FALSE(info.ShouldEmitNegativeChild());
    EXPECT_TRUE(info.ShouldEmitPositiveChild());

    EXPECT_EQ(info.pos_range().first, 0);
    EXPECT_EQ(info.pos_range().second, ccw_polygon.vertex_count());
  }
  {
    auto info = cw_polygon.GetSplitInfo(above_down);
    EXPECT_FALSE(info.ShouldEmitNegativeChild());
    EXPECT_TRUE(info.ShouldEmitPositiveChild());

    EXPECT_EQ(info.pos_range().first, 0);
    EXPECT_EQ(info.pos_range().second, cw_polygon.vertex_count());
  }

  HalfSpace3 below_up(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/9);
  {
    auto info = ccw_polygon.GetSplitInfo(below_up);
    EXPECT_FALSE(info.ShouldEmitNegativeChild());
    EXPECT_TRUE(info.ShouldEmitPositiveChild());

    EXPECT_EQ(info.pos_range().first, 0);
    EXPECT_EQ(info.pos_range().second, ccw_polygon.vertex_count());
  }
  {
    auto info = cw_polygon.GetSplitInfo(below_up);
    EXPECT_FALSE(info.ShouldEmitNegativeChild());
    EXPECT_TRUE(info.ShouldEmitPositiveChild());

    EXPECT_EQ(info.pos_range().first, 0);
    EXPECT_EQ(info.pos_range().second, cw_polygon.vertex_count());
  }
}

TEST(ConvexPolygon, SplitOnFractionalParallelPlane) {
  Point3 ccw_input[] = {
    Point3(0, 0, 1),
    Point3(1, 0, 1),
    Point3(1, 1, 1),
  };

  ConvexPolygon<> triangle = MakeConvexPolygon(ccw_input);

  HalfSpace3 below_up(/*x=*/0, /*y=*/0, /*z=*/10, /*dist=*/9);
  {
    auto info = triangle.GetSplitInfo(below_up);
    EXPECT_FALSE(info.ShouldEmitNegativeChild());
    EXPECT_TRUE(info.ShouldEmitPositiveChild());

    EXPECT_EQ(info.pos_range().first, 0);
    EXPECT_EQ(info.pos_range().second, triangle.vertex_count());
  }
}

TEST(ConvexPolygon, VerticesIterator) {
  Point3 input[] = {
    Point3(0, 0, 10),
    Point3(1, 0, 11),
    Point3(1, 1, 12),
  };

  ConvexPolygon<> triangle = MakeConvexPolygon(input);

  EXPECT_EQ(&*triangle.vertices_begin(), &triangle.vertex(0));
  EXPECT_EQ(triangle.vertices_end() - triangle.vertices_begin(),
            triangle.vertex_count());
}

TEST(ConvexPolygon, FailMergeReflexVertex) {
  //                          |
  //      p[3]       p[2]     |
  //      +-----------+       |
  //      | polygon1 /        |
  //      |         /         |
  // p[0] |________/ p[1]     |
  // q[3] |        \ q[2]     |
  //      |         \         |
  //      | polygon2 \        |
  //      +-----------+       |
  //      q[0]       q[1]     |
  //                          |
  Point3 p[] = {
    Point3(0, 2, 10),
    Point3(2, 2, 10),
    Point3(3, 4, 10),
    Point3(0, 4, 10),
  };
  MutableConvexPolygon<> polygon1 = MakeConvexPolygon(p);
  EXPECT_EQ(polygon1.vertex(0), p[0]);

  Point3 q[] = {
    Point3(0, 0, 10),
    Point3(3, 0, 10),
    Point3(2, 2, 10),
    Point3(0, 2, 10),
  };
  MutableConvexPolygon<> polygon2 = MakeConvexPolygon(q);
  EXPECT_EQ(polygon2.vertex(0), q[0]);

  ASSERT_FALSE(polygon1.TryMergePolygon(/*nonzero_edge_dimension=*/0,
                                        /*my_edge_index=*/0,
                                        /*other=*/polygon2,
                                        /*other_edge_index=*/2));
  ASSERT_FALSE(polygon2.TryMergePolygon(/*nonzero_edge_dimension=*/0,
                                        /*my_edge_index=*/2,
                                        /*other=*/polygon1,
                                        /*other_edge_index=*/0));
}

TEST(ConvexPolygon, MergeConvexVertex) {
  //                              |
  //      p[3]       p[2]         |
  //      +-----------+           |
  //      | polygon1   \          |
  //      |             \         |
  // p[0] |______________\ p[1]   |
  // q[3] |              / q[2]   |
  //      |             /         |
  //      | polygon2   /          |
  //      +-----------+           |
  //      q[0]       q[1]         |
  //                              |
  Point3 p[] = {
    Point3(0, 2, 10),
    Point3(4, 2, 10),
    Point3(3, 4, 10),
    Point3(0, 4, 10),
  };
  MutableConvexPolygon<> polygon1a = MakeConvexPolygon(p);
  MutableConvexPolygon<> polygon1b = MakeConvexPolygon(p);
  EXPECT_EQ(polygon1a.vertex(0), p[0]);

  Point3 q[] = {
    Point3(0, 0, 10),
    Point3(3, 0, 10),
    p[1],
    p[0],
  };
  MutableConvexPolygon<> polygon2a = MakeConvexPolygon(q);
  MutableConvexPolygon<> polygon2b = MakeConvexPolygon(q);
  EXPECT_EQ(polygon2a.vertex(0), q[0]);

  Point3 merged_points[] = {
    q[0],
    q[1],
    q[2],
    p[2],
    p[3],
  };
  ConvexPolygon<> expected_merged = MakeConvexPolygon(merged_points);

  EXPECT_TRUE(polygon1a.TryMergePolygon(/*nonzero_edge_dimension=*/0,
                                        /*my_edge_index=*/0,
                                        /*other=*/polygon2a,
                                        /*other_edge_index=*/2));
  EXPECT_EQ(polygon1a, expected_merged);
  EXPECT_EQ(polygon2a.vertex_count(), 0);
  EXPECT_TRUE(polygon2b.TryMergePolygon(/*nonzero_edge_dimension=*/0,
                                        /*my_edge_index=*/2,
                                        /*other=*/polygon1b,
                                        /*other_edge_index=*/0));
  EXPECT_EQ(polygon2b, expected_merged);
  EXPECT_EQ(polygon1b.vertex_count(), 0);
}

TEST(ConvexPolygon, FailMergeExtraVertex) {
  //                          |
  //      p[4]       p[3]     |
  //      +-----------+       |
  //      | polygon1 /        |
  //      |  p[1]   /         |
  // p[0] |____+___/ p[2]     |
  // q[4] |  q[3]  \ q[2]     |
  //      |         \         |
  //      | polygon2 \        |
  //      +-----------+       |
  //      q[0]       q[1]     |
  //                          |
  Point3 p[] = {
    Point3(0, 2, 10),
    Point3(1, 2, 10),
    Point3(2, 2, 10),
    Point3(3, 4, 10),
    Point3(0, 4, 10),
  };
  MutableConvexPolygon<> polygon1 = MakeConvexPolygon(p);
  EXPECT_EQ(polygon1.vertex(0), p[0]);

  Point3 q[] = {
    Point3(0, 0, 10),
    Point3(3, 0, 10),
    Point3(2, 2, 10),
    Point3(1, 2, 10),
    Point3(0, 2, 10),
  };
  MutableConvexPolygon<> polygon2 = MakeConvexPolygon(q);
  EXPECT_EQ(polygon2.vertex(0), q[0]);

  ASSERT_FALSE(polygon1.TryMergePolygon(/*nonzero_edge_dimension=*/0,
                                        /*my_edge_index=*/0,
                                        /*other=*/polygon2,
                                        /*other_edge_index=*/3));
  ASSERT_FALSE(polygon1.TryMergePolygon(/*nonzero_edge_dimension=*/0,
                                        /*my_edge_index=*/1,
                                        /*other=*/polygon2,
                                        /*other_edge_index=*/2));

  ASSERT_FALSE(polygon2.TryMergePolygon(/*nonzero_edge_dimension=*/0,
                                        /*my_edge_index=*/3,
                                        /*other=*/polygon1,
                                        /*other_edge_index=*/0));
  ASSERT_FALSE(polygon2.TryMergePolygon(/*nonzero_edge_dimension=*/0,
                                        /*my_edge_index=*/2,
                                        /*other=*/polygon1,
                                        /*other_edge_index=*/1));
}

TEST(ConvexPolygon, FailMergeDifferentNormal) {
  //                              |
  //      p[3]       p[2]         |
  //      +-----------+           |
  //      | polygon1   \          |
  //      |             \         |
  // p[0] |______________\ p[1]   |
  // q[3] |              / q[2]   |
  //      |             /         |
  //      | polygon2   /          |
  //      +-----------+           |
  //      q[0]       q[1]         |
  //                              |
  // polygon1's normal is completely vertical. polygon2's normal points a
  // little down.
  Point3 p[] = {
    Point3(0, 2, 10),
    Point3(4, 2, 10),
    Point3(3, 4, 10),
    Point3(0, 4, 10),
  };
  MutableConvexPolygon<> polygon1 = MakeConvexPolygon(p);
  EXPECT_EQ(polygon1.vertex(0), p[0]);

  Point3 q[] = {
    Point3(0, 0, 0),
    Point3(3, 0, 0),
    Point3(4, 2, 10),
    Point3(0, 2, 10),
  };
  MutableConvexPolygon<> polygon2 = MakeConvexPolygon(q);
  EXPECT_EQ(polygon2.vertex(0), q[0]);

  ASSERT_FALSE(polygon1.TryMergePolygon(/*nonzero_edge_dimension=*/0,
                                        /*my_edge_index=*/0,
                                        /*other=*/polygon2,
                                        /*other_edge_index=*/2));
  ASSERT_FALSE(polygon2.TryMergePolygon(/*nonzero_edge_dimension=*/0,
                                        /*my_edge_index=*/2,
                                        /*other=*/polygon1,
                                        /*other_edge_index=*/0));
}

TEST(ConvexPolygon, MergeLastVertex) {
  //
  // p[3]    p[2] q[3]     q[2]
  //  +----------+----------+
  //  |          |          |
  //  | polygon1 | polygon2 |
  //  |          |          |
  //  |          |          |
  //  +----------+----------+
  // p[0]    p[1] q[0]     q[1]
  //
  Point3 p[] = {
    Point3(0, 0, 10),
    Point3(2, 0, 10),
    Point3(2, 2, 10),
    Point3(0, 2, 10),
  };
  MutableConvexPolygon<> polygon1a = MakeConvexPolygon(p);
  MutableConvexPolygon<> polygon1b = MakeConvexPolygon(p);
  EXPECT_EQ(polygon1a.vertex(0), p[0]);

  Point3 q[] = {
    p[1],
    Point3(4, 0, 10),
    Point3(4, 2, 10),
    p[2],
  };
  MutableConvexPolygon<> polygon2a = MakeConvexPolygon(q);
  MutableConvexPolygon<> polygon2b = MakeConvexPolygon(q);
  EXPECT_EQ(polygon2a.vertex(0), q[0]);

  Point3 merged_points[] = {
    p[0],
    q[1],
    q[2],
    p[3],
  };
  ConvexPolygon<> expected_merged = MakeConvexPolygon(merged_points);

  EXPECT_TRUE(polygon1a.TryMergePolygon(/*nonzero_edge_dimension=*/1,
                                        /*my_edge_index=*/1,
                                        /*other=*/polygon2a,
                                        /*other_edge_index=*/3));
  EXPECT_EQ(polygon1a, expected_merged);
  EXPECT_EQ(polygon2a.vertex_count(), 0);
  EXPECT_TRUE(polygon2b.TryMergePolygon(/*nonzero_edge_dimension=*/1,
                                        /*my_edge_index=*/3,
                                        /*other=*/polygon1b,
                                        /*other_edge_index=*/1));
  EXPECT_EQ(polygon2b, expected_merged);
  EXPECT_EQ(polygon1b.vertex_count(), 0);
}

}  // walnut
