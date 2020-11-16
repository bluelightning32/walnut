#include "walnut/convex_polygon.h"

#include <iterator>

#include "gtest/gtest.h"
#include "walnut/convex_polygon_factory.h"

namespace walnut {

template<typename Container>
auto
MakeUnsortedConvexPolygon(const Container& vertices) ->
ConvexPolygon<std::iterator_traits<
    decltype(std::begin(vertices))>::value_type::component_bits> {
  using Iterator = decltype(std::begin(vertices));
  using Point3Rep = typename std::iterator_traits<Iterator>::value_type;
  using ConvexPolygonRep = ConvexPolygon<Point3Rep::component_bits>;

  class CollectOne : public ConvexPolygonRep::Factory {
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
auto
MakeConvexPolygon(const Container& vertices) ->
ConvexPolygon<std::iterator_traits<
    decltype(std::begin(vertices))>::value_type::component_bits> {
  auto result = MakeUnsortedConvexPolygon(vertices);
  result.SortVertices();
  return result;
}

TEST(ConvexPolygon, TrianglePlane) {
  Point3<32> input[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10),
    Point3<32>(1, 1, 10),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);
  EXPECT_EQ(polygon.plane(),
            HalfSpace3<>(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10));

  EXPECT_FALSE(polygon.plane().normal().IsZero());
  EXPECT_GT(polygon.plane().normal().z(), 0);
  EXPECT_EQ(polygon.drop_dimension(), 2);

  for (const Point3<32>& v : input) {
    EXPECT_TRUE(polygon.plane().IsCoincident(v));
  }
}

TEST(ConvexPolygon, Triangle0DistPlane) {
  Point3<32> input[] = {
    Point3<32>(0, 0, 0),
    Point3<32>(10, 0, 0),
    Point3<32>(10, 10, 0),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);
  EXPECT_EQ(polygon.plane(),
            HalfSpace3<>(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/0));

  EXPECT_EQ(polygon.drop_dimension(), 2);

  for (const Point3<32>& v : input) {
    EXPECT_TRUE(polygon.plane().IsCoincident(v));
  }
}

TEST(ConvexPolygon, TriangleXZPlane) {
  Point3<32> input[] = {
    Point3<32>(0 - 5, 0, 0 + 5),
    Point3<32>(10 - 5, 0, 10 + 5),
    Point3<32>(10 - 5, 10, 10 + 5),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);
  EXPECT_EQ(polygon.plane(),
            HalfSpace3<>(/*x=*/-1, /*y=*/0, /*z=*/1, /*dist=*/10));

  for (const Point3<32>& v : input) {
    EXPECT_TRUE(polygon.plane().IsCoincident(v));
  }
}

TEST(ConvexPolygon, ClockwiseTrianglePlane) {
  Point3<32> input[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(1, 0, 10),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);
  EXPECT_EQ(polygon.plane(),
            HalfSpace3<>(/*x=*/0, /*y=*/0, /*z=*/-1, /*dist=*/-10));

  EXPECT_FALSE(polygon.plane().normal().IsZero());
  EXPECT_LT(polygon.plane().normal().z(), 0);
  EXPECT_EQ(polygon.drop_dimension(), 2);

  for (const Point3<32>& v : input) {
    EXPECT_TRUE(polygon.plane().IsCoincident(v));
  }
}

TEST(ConvexPolygon, ClockwiseTriangleXZPlane) {
  Point3<32> input[] = {
    Point3<32>(0 - 5, 0, 0 + 5),
    Point3<32>(10 - 5, 10, 10 + 5),
    Point3<32>(10 - 5, 0, 10 + 5),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);
  EXPECT_EQ(polygon.plane(),
            HalfSpace3<>(/*x=*/1, /*y=*/0, /*z=*/-1, /*dist=*/-10));

  EXPECT_FALSE(polygon.plane().normal().IsZero());
  EXPECT_LT(polygon.plane().normal().z(), 0);

  for (const Point3<32>& v : input) {
    EXPECT_TRUE(polygon.plane().IsCoincident(v));
  }
}

TEST(ConvexPolygon, ClockwiseSquareYZPlane) {
  Point3<32> input[] = {
    Point3<32>(0, 0, 0),
    Point3<32>(0, 10, 1),
    Point3<32>(10, 10, 1),
    Point3<32>(10, 0, 0),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);
  EXPECT_EQ(polygon.plane(),
            HalfSpace3<>(/*x=*/0, /*y=*/1, /*z=*/-10, /*dist=*/0));

  EXPECT_FALSE(polygon.plane().normal().IsZero());

  for (const Point3<32>& v : input) {
    EXPECT_TRUE(polygon.plane().IsCoincident(v));
  }
}

TEST(ConvexPolygon, CopyConstructor) {
  Point3<32> input[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10),
    Point3<32>(1, 1, 10),
  };

  ConvexPolygon<32> polygon1 = MakeConvexPolygon(input);
  ConvexPolygon<32> polygon2(polygon1);
  EXPECT_EQ(polygon2, polygon1);
}

TEST(ConvexPolygon, EqualityOperator) {
  Point3<32> input1[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10),
    Point3<32>(1, 1, 10),
  };
  ConvexPolygon<32> polygon1 = MakeUnsortedConvexPolygon(input1);
  EXPECT_EQ(polygon1, polygon1);

  // different plane
  {
    Point3<32> input2[] = {
      Point3<32>(0, 0, 11),
      Point3<32>(1, 0, 11),
      Point3<32>(1, 1, 11),
    };
    ConvexPolygon<32> polygon2 = MakeUnsortedConvexPolygon(input2);
    EXPECT_NE(polygon1, polygon2);
    EXPECT_NE(polygon2, polygon1);
  }

  // Same plane as input1, but translated
  {
    Point3<32> input3[] = {
      Point3<32>(1, 0, 10),
      Point3<32>(2, 0, 10),
      Point3<32>(2, 1, 10),
    };
    ConvexPolygon<32> polygon3 = MakeUnsortedConvexPolygon(input3);
    EXPECT_NE(polygon1, polygon3);
    EXPECT_NE(polygon3, polygon1);
  }

  // First 2 points the same as input1, but different 3rd point
  {
    Point3<32> input4[] = {
      input1[0],
      input1[1],
      Point3<32>(2, 1, 10),
    };
    ConvexPolygon<32> polygon4 = MakeUnsortedConvexPolygon(input4);
    EXPECT_NE(polygon1, polygon4);
    EXPECT_NE(polygon4, polygon1);
  }

  // Flipped version of input1
  {
    Point3<32> input5[] = {
      input1[2],
      input1[1],
      input1[0],
    };
    ConvexPolygon<32> polygon5 = MakeUnsortedConvexPolygon(input5);
    EXPECT_NE(polygon1, polygon5);
    EXPECT_NE(polygon5, polygon1);
  }

  // Same as input1, but with vertex indices rotated
  {
    Point3<32> compare_input[] = {
      input1[1],
      input1[2],
      input1[0],
    };
    ConvexPolygon<32> compare_polygon =
      MakeUnsortedConvexPolygon(compare_input);
    EXPECT_EQ(polygon1, compare_polygon);
    EXPECT_EQ(compare_polygon, polygon1);

    compare_polygon.SortVertices();
    EXPECT_EQ(polygon1, compare_polygon);
    EXPECT_EQ(compare_polygon, polygon1);
  }
}

TEST(ConvexPolygon, CounterClockwiseTriangleEdges) {
  Point3<32> input[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10),
    Point3<32>(1, 1, 10),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);

  ASSERT_EQ(polygon.vertex_count(), std::end(input) - std::begin(input));
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    EXPECT_EQ(polygon.vertex(i), input[i]);
    const ConvexPolygon<32>::HomoPoint3Rep& vertex = polygon.vertex(i);
    const ConvexPolygon<32>::HomoPoint3Rep& next_vertex = polygon.vertex(
        (i + 1) % polygon.vertex_count());
    const Point3<32>& input_vertex = input[i];
    const Point3<32>& next_input_vertex = input[
        (i + 1) % polygon.vertex_count()];
    EXPECT_TRUE(polygon.edge(i).line.IsCoincident(vertex));
    EXPECT_TRUE(polygon.edge(i).line.IsCoincident(next_vertex));
    EXPECT_TRUE(polygon.edge(i).line.IsCoincident(input_vertex));
    EXPECT_TRUE(polygon.edge(i).line.IsCoincident(next_input_vertex));
    EXPECT_GT((next_input_vertex - input_vertex).Dot(polygon.edge(i).line.d()),
              0);
  }
}

TEST(ConvexPolygon, CounterClockwiseSquareEdges) {
  Point3<32> input[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(0, 1, 10),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);

  ASSERT_EQ(polygon.vertex_count(), std::end(input) - std::begin(input));
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    EXPECT_EQ(polygon.vertex(i), input[i]);
    const ConvexPolygon<32>::HomoPoint3Rep& vertex = polygon.vertex(i);
    const ConvexPolygon<32>::HomoPoint3Rep& next_vertex = polygon.vertex(
        (i + 1) % polygon.vertex_count());
    const Point3<32>& input_vertex = input[i];
    const Point3<32>& next_input_vertex = input[
        (i + 1) % polygon.vertex_count()];
    EXPECT_TRUE(polygon.edge(i).line.IsCoincident(vertex));
    EXPECT_TRUE(polygon.edge(i).line.IsCoincident(next_vertex));
    EXPECT_TRUE(polygon.edge(i).line.IsCoincident(input_vertex));
    EXPECT_TRUE(polygon.edge(i).line.IsCoincident(next_input_vertex));
    EXPECT_GT((next_input_vertex - input_vertex).Dot(polygon.edge(i).line.d()),
              0);
  }
}

TEST(ConvexPolygon, ClockwiseSquareEdges) {
  Point3<32> input[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(0, 1, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(1, 0, 10),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);

  ASSERT_EQ(polygon.vertex_count(), std::end(input) - std::begin(input));
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    EXPECT_EQ(polygon.vertex(i), input[i]);
    const ConvexPolygon<32>::HomoPoint3Rep& vertex = polygon.vertex(i);
    const ConvexPolygon<32>::HomoPoint3Rep& next_vertex = polygon.vertex(
        (i + 1) % polygon.vertex_count());
    const Point3<32>& input_vertex = input[i];
    const Point3<32>& next_input_vertex = input[
        (i + 1) % polygon.vertex_count()];
    EXPECT_TRUE(polygon.edge(i).line.IsCoincident(vertex));
    EXPECT_TRUE(polygon.edge(i).line.IsCoincident(next_vertex));
    EXPECT_TRUE(polygon.edge(i).line.IsCoincident(input_vertex));
    EXPECT_TRUE(polygon.edge(i).line.IsCoincident(next_input_vertex));
    EXPECT_GT((next_input_vertex - input_vertex).Dot(polygon.edge(i).line.d()),
              0);
  }
}

TEST(ConvexPolygon, RedundantEdges) {
  Point3<32> input[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10), // collinear
    Point3<32>(2, 0, 10), // collinear
    Point3<32>(3, 0, 10),
    Point3<32>(3, 1, 10),
    Point3<32>(2, 1, 10), // collinear
    Point3<32>(1, 1, 10), // collinear
    Point3<32>(0, 1, 10),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);

  ASSERT_EQ(polygon.vertex_count(), std::end(input) - std::begin(input));
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    EXPECT_EQ(polygon.vertex(i), input[i]);
    const ConvexPolygon<32>::HomoPoint3Rep& vertex = polygon.vertex(i);
    const ConvexPolygon<32>::HomoPoint3Rep& next_vertex = polygon.vertex(
        (i + 1) % polygon.vertex_count());
    const Point3<32>& input_vertex = input[i];
    const Point3<32>& next_input_vertex = input[
        (i + 1) % polygon.vertex_count()];
    EXPECT_TRUE(polygon.edge(i).line.IsCoincident(vertex));
    EXPECT_TRUE(polygon.edge(i).line.IsCoincident(next_vertex));
    EXPECT_TRUE(polygon.edge(i).line.IsCoincident(input_vertex));
    EXPECT_TRUE(polygon.edge(i).line.IsCoincident(next_input_vertex));
    EXPECT_GT((next_input_vertex - input_vertex).Dot(polygon.edge(i).line.d()),
              0);
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
  Point3<32> p[] = {
    /*p[0]=*/Point3<32>(1, 1, 10),
    /*p[1]=*/Point3<32>(0, 1, 10),
    /*p[2]=*/Point3<32>(0, 0, 10),
    /*p[3]=*/Point3<32>(1, 0, 10),
    /*p[4]=*/Point3<32>(2, 0, 10),
    /*p[5]=*/Point3<32>(3, 0, 10),
    /*p[6]=*/Point3<32>(3, 1, 10),
    /*p[7]=*/Point3<32>(2, 1, 10),
  };
  ConvexPolygon<32> polygon = MakeConvexPolygon(p);

  Vector2<> vector(0, 1);
  std::pair<size_t, size_t> opp_edges = polygon.GetOppositeEdgeIndicesBisect(
      vector, /*drop_dimension=*/2);

  EXPECT_GT(polygon.edge(opp_edges.first).line.d().DropDimension(2)
                   .Dot(vector), 0);
  EXPECT_LT(polygon.edge(opp_edges.second).line.d().DropDimension(2)
                   .Dot(vector), 0);

  vector.Negate();
  opp_edges = polygon.GetOppositeEdgeIndicesBisect(vector,
                                                   /*drop_dimension=*/2);
  EXPECT_GT(polygon.edge(opp_edges.first).line.d().DropDimension(2)
                   .Dot(vector), 0);
  EXPECT_LT(polygon.edge(opp_edges.second).line.d().DropDimension(2)
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
  Point3<32> p[100] = {
    /*p[0]=*/Point3<32>(1, 0, 10),
  };

  for (int i = 1; i < 100; ++i) {
    p[i] = Point3<32>(99 - i, 1, 10);
  }

  const Vector2<> vector(-1, 0);
  for (size_t offset = 0; offset < sizeof(p)/sizeof(p[0]); ++offset) {
    ConvexPolygon<32> polygon = MakeConvexPolygon(p);

    std::pair<size_t, size_t> opp_edges = polygon.GetOppositeEdgeIndicesBisect(
        vector, /*drop_dimension=*/2);

    EXPECT_GT(polygon.edge(opp_edges.first).line.d().DropDimension(2)
                     .Dot(vector), 0);
    EXPECT_LT(polygon.edge(opp_edges.second).line.d().DropDimension(2)
                     .Dot(vector), 0);

    std::rotate(std::begin(p), std::begin(p) + 1, std::end(p));
  }
}

TEST(ConvexPolygon, GetGreaterCycleIndex) {
  Point3<32> input[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(0, 1, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(1, 0, 10),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);

  for (int a = 0; a < polygon.vertex_count(); ++a) {
    for (int b = 0; b < polygon.vertex_count(); ++b) {
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
  Point3<32> p[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(0, 1, 10),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(p);

  Vector2<> to_corner[] = {
    Vector2<>(-1, -1),
    Vector2<>(1, -1),
    Vector2<>(1, 1),
    Vector2<>(-1, 1),
  };

  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(polygon.GetExtremeIndexBisect(to_corner[i],
                                            /*drop_dimension=*/2), i);
  }

  Vector2<> along_edges[] = {
    Vector2<>(0, -1),
    Vector2<>(1, 0),
    Vector2<>(0, 1),
    Vector2<>(-1, 0),
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
  Point3<32> p[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(0, 1, 10),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(p);

  HalfSpace2<> only_corner[] = {
    HalfSpace2<>(p[1].DropDimension(2), p[3].DropDimension(2)),
    HalfSpace2<>(p[2].DropDimension(2), p[0].DropDimension(2)),
    HalfSpace2<>(p[3].DropDimension(2), p[1].DropDimension(2)),
    HalfSpace2<>(p[0].DropDimension(2), p[2].DropDimension(2)),
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
  HalfSpace2<> on_plane[] = {
    HalfSpace2<>(p[1].DropDimension(2), p[0].DropDimension(2)),
    HalfSpace2<>(p[2].DropDimension(2), p[1].DropDimension(2)),
    HalfSpace2<>(p[3].DropDimension(2), p[2].DropDimension(2)),
    HalfSpace2<>(p[0].DropDimension(2), p[3].DropDimension(2)),
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
  Point3<32> p[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(0, 1, 10),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(p);

  HalfSpace2<> only_corner[] = {
    HalfSpace2<>(p[3].DropDimension(2), p[1].DropDimension(2)),
    HalfSpace2<>(p[0].DropDimension(2), p[2].DropDimension(2)),
    HalfSpace2<>(p[1].DropDimension(2), p[3].DropDimension(2)),
    HalfSpace2<>(p[2].DropDimension(2), p[0].DropDimension(2)),
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
  HalfSpace2<> on_plane[] = {
    HalfSpace2<>(p[0].DropDimension(2), p[1].DropDimension(2)),
    HalfSpace2<>(p[1].DropDimension(2), p[2].DropDimension(2)),
    HalfSpace2<>(p[2].DropDimension(2), p[3].DropDimension(2)),
    HalfSpace2<>(p[3].DropDimension(2), p[0].DropDimension(2)),
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
  Point3<32> p[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(0, 1, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(1, 0, 10),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(p);

  Vector2<> to_corner[] = {
    Vector2<>(-1, -1),
    Vector2<>(-1, 1),
    Vector2<>(1, 1),
    Vector2<>(1, -1),
  };

  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(polygon.GetExtremeIndexBisect(to_corner[i],
                                            /*drop_dimension=*/2), i);
  }

  Vector2<> along_edges[] = {
    Vector2<>(-1, 0),
    Vector2<>(0, 1),
    Vector2<>(1, 0),
    Vector2<>(0, -1),
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
  Point3<32> p[] = {
    Point3<32>(0, 0, 0),
    Point3<32>(1, 0, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(0, 1, 0),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(p);

  Vector3<> to_corner[] = {
    Vector3<>(-1, -1, -1),
    Vector3<>(1, -1, 1),
    Vector3<>(1, 1, 1),
    Vector3<>(-1, 1, -1),
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
  Point3<32> p[] = {
    Point3<32>(0, 0, 0),
    Point3<32>(0, 1, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(1, 0, 0),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(p);
  EXPECT_LT(polygon.plane().normal().z(), 0);

  Vector3<> to_corner[] = {
    Vector3<>(-1, -1, -1),
    Vector3<>(-1, 1, 1),
    Vector3<>(1, 1, 1),
    Vector3<>(1, -1, -1),
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
  //      p[3]
  //   /       \
  // p[4]      p[2]
  //  |          ^
  //  |          |
  //  v          |
  // p[0] ---> p[1]
  //
  // However, dropping the X dimension (so that Y and Z are remaining, and Z
  // was a fraction of X) also effectively non-uniformly scales the polygon,
  // squishing horizontally so that it looks like this:
  //
  //      p[3]
  //    /     \
  //   /       \
  // p[4]      p[2]
  //  |          ^
  //  |          |
  //  |          |
  //  |          |
  //  |          |
  //  v          |
  // p[0] ---> p[1]
  //
  // So for a vector from p[0] to p[2], in the first polygon, p[2] is the
  // extreme index, but for the below diagram, p[3] is the extreme index. This
  // happens because distances lines are effectively drawn perpendicular to the
  // vector, and the non-uniform scaling does not update the 90 degree angle.
  // So whether the 90 degree angle is taken before or after the non-uniform
  // scaling affects the result.

  Point3<32> p[] = {
    Point3<32>(0, 0, 0),
    Point3<32>(40, 0, 4),
    Point3<32>(40, 40, 4),
    Point3<32>(20, 45, 2),
    Point3<32>(0, 40, 0),
  };
  ConvexPolygon<32> polygon = MakeConvexPolygon(p);

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
  Point3<32> p[] = {
    /*p[0]=*/Point3<32>(0, 0, 10),
    /*p[1]=*/Point3<32>(1, 0, 10),
    /*p[2]=*/Point3<32>(2, 0, 10),
    /*p[3]=*/Point3<32>(3, 0, 10),
    /*p[4]=*/Point3<32>(3, 1, 10),
    /*p[5]=*/Point3<32>(3, 2, 10),
    /*p[6]=*/Point3<32>(2, 2, 10),
    /*p[7]=*/Point3<32>(1, 2, 10),
    /*p[8]=*/Point3<32>(0, 2, 10),
    /*p[9]=*/Point3<32>(0, 1, 10),
  };

  const ConvexPolygon<32> polygon = MakeConvexPolygon(p);
  HalfSpace2<> half_space(p[9].DropDimension(2), p[4].DropDimension(2));

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
  Point3<32> p[] = {
    /*p[0]=*/Point3<32>(0, 0, 10),
    /*p[1]=*/Point3<32>(1, 0, 10),
    /*p[2]=*/Point3<32>(2, 0, 10),
    /*p[3]=*/Point3<32>(3, 0, 10),
    /*p[4]=*/Point3<32>(3, 1, 10),
    /*p[5]=*/Point3<32>(3, 3, 10),
    /*p[6]=*/Point3<32>(2, 3, 10),
    /*p[7]=*/Point3<32>(1, 3, 10),
    /*p[8]=*/Point3<32>(0, 3, 10),
    /*p[9]=*/Point3<32>(0, 1, 10),
  };

  const ConvexPolygon<32> polygon = MakeConvexPolygon(p);
  HalfSpace2<> half_space(Point2<>(0, 2), Point2<>(1, 2));

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

TEST(ConvexPolygon, ConvertVertexData) {
  Point3<32> input[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10),
    Point3<32>(1, 1, 10),
  };

  struct VertexData {
    VertexData() = default;
    VertexData(const std::tuple<>&) { }

    bool extra = false;

    bool operator!=(const std::tuple<>&) const {
      return false;
    }
  };

  ConvexPolygon<32> polygon_raw = MakeConvexPolygon(input);
  ConvexPolygon<32, VertexData> polygon = polygon_raw;

  EXPECT_EQ(polygon, polygon_raw);
  ASSERT_EQ(polygon.vertex_count(), 3);

  EXPECT_FALSE(polygon.vertex_data(0).extra);
  polygon.vertex_data(0).extra = true;
  EXPECT_TRUE(polygon.vertex_data(0).extra);
}

}  // walnut
