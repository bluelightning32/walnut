#include "walnut/convex_polygon.h"

#include <iterator>

#include "gtest/gtest.h"
#include "walnut/convex_polygon_factory.h"

namespace walnut {

template<typename Container>
auto
MakeConvexPolygon(const Container& vertices) ->
ConvexPolygon<std::iterator_traits<
    decltype(std::begin(vertices))>::value_type::coord_bits> {
  using Iterator = decltype(std::begin(vertices));
  using Vertex3Rep = typename std::iterator_traits<Iterator>::value_type;
  using ConvexPolygonRep = ConvexPolygon<Vertex3Rep::coord_bits>;

  class CollectOne : public ConvexPolygonRep::Factory {
   public:
    CollectOne() = default;

    ConvexPolygonRep&& GetResult() {
      EXPECT_EQ(received_, 1);
      result_.SortVertices();
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

TEST(ConvexPolygon, TrianglePlane) {
  Vertex3<32> input[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(1, 0, 10),
    Vertex3<32>(1, 1, 10),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);
  EXPECT_EQ(polygon.plane(),
            Plane<>(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10));

  EXPECT_EQ(polygon.drop_dimension(), 2);

  for (const Vertex3<32>& v : input) {
    EXPECT_TRUE(polygon.plane().IsCoincident(v));
  }
}

TEST(ConvexPolygon, Triangle0DistPlane) {
  Vertex3<32> input[] = {
    Vertex3<32>(0, 0, 0),
    Vertex3<32>(10, 0, 0),
    Vertex3<32>(10, 10, 0),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);
  EXPECT_EQ(polygon.plane(),
            Plane<>(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/0));

  EXPECT_EQ(polygon.drop_dimension(), 2);

  for (const Vertex3<32>& v : input) {
    EXPECT_TRUE(polygon.plane().IsCoincident(v));
  }
}

TEST(ConvexPolygon, TriangleXZPlane) {
  Vertex3<32> input[] = {
    Vertex3<32>(0 - 5, 0, 0 + 5),
    Vertex3<32>(10 - 5, 0, 10 + 5),
    Vertex3<32>(10 - 5, 10, 10 + 5),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);
  EXPECT_EQ(polygon.plane(),
            Plane<>(/*x=*/-1, /*y=*/0, /*z=*/1, /*dist=*/10));

  for (const Vertex3<32>& v : input) {
    EXPECT_TRUE(polygon.plane().IsCoincident(v));
  }
}

TEST(ConvexPolygon, ClockwiseTrianglePlane) {
  Vertex3<32> input[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(1, 1, 10),
    Vertex3<32>(1, 0, 10),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);
  EXPECT_EQ(polygon.plane(),
            Plane<>(/*x=*/0, /*y=*/0, /*z=*/-1, /*dist=*/-10));

  EXPECT_EQ(polygon.drop_dimension(), 2);

  for (const Vertex3<32>& v : input) {
    EXPECT_TRUE(polygon.plane().IsCoincident(v));
  }
}

TEST(ConvexPolygon, ClockwiseTriangleXZPlane) {
  Vertex3<32> input[] = {
    Vertex3<32>(0 - 5, 0, 0 + 5),
    Vertex3<32>(10 - 5, 10, 10 + 5),
    Vertex3<32>(10 - 5, 0, 10 + 5),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);
  EXPECT_EQ(polygon.plane(),
            Plane<>(/*x=*/1, /*y=*/0, /*z=*/-1, /*dist=*/-10));

  for (const Vertex3<32>& v : input) {
    EXPECT_TRUE(polygon.plane().IsCoincident(v));
  }
}

TEST(ConvexPolygon, CopyConstructor) {
  Vertex3<32> input[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(1, 0, 10),
    Vertex3<32>(1, 1, 10),
  };

  ConvexPolygon<32> polygon1 = MakeConvexPolygon(input);
  ConvexPolygon<32> polygon2(polygon1);
  EXPECT_EQ(polygon2, polygon1);
}

TEST(ConvexPolygon, CounterClockwiseTriangleEdges) {
  Vertex3<32> input[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(1, 0, 10),
    Vertex3<32>(1, 1, 10),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);

  ASSERT_EQ(polygon.vertex_count(), std::end(input) - std::begin(input));
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    EXPECT_EQ(polygon.vertex(i), input[i]);
    const ConvexPolygon<32>::Vertex4Rep& vertex = polygon.vertex(i);
    const ConvexPolygon<32>::Vertex4Rep& next_vertex = polygon.vertex(
        (i + 1) % polygon.vertex_count());
    const Vertex3<32>& input_vertex = input[i];
    const Vertex3<32>& next_input_vertex = input[
        (i + 1) % polygon.vertex_count()];
    EXPECT_TRUE(polygon.edge(i).IsOnLine(vertex));
    EXPECT_TRUE(polygon.edge(i).IsOnLine(next_vertex));
    EXPECT_TRUE(polygon.edge(i).IsOnLine(input_vertex));
    EXPECT_TRUE(polygon.edge(i).IsOnLine(next_input_vertex));
    EXPECT_GT((next_input_vertex - input_vertex).Dot(polygon.edge(i).d()), 0);
  }
}

TEST(ConvexPolygon, CounterClockwiseSquareEdges) {
  Vertex3<32> input[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(1, 0, 10),
    Vertex3<32>(1, 1, 10),
    Vertex3<32>(0, 1, 10),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);

  ASSERT_EQ(polygon.vertex_count(), std::end(input) - std::begin(input));
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    EXPECT_EQ(polygon.vertex(i), input[i]);
    const ConvexPolygon<32>::Vertex4Rep& vertex = polygon.vertex(i);
    const ConvexPolygon<32>::Vertex4Rep& next_vertex = polygon.vertex(
        (i + 1) % polygon.vertex_count());
    const Vertex3<32>& input_vertex = input[i];
    const Vertex3<32>& next_input_vertex = input[
        (i + 1) % polygon.vertex_count()];
    EXPECT_TRUE(polygon.edge(i).IsOnLine(vertex));
    EXPECT_TRUE(polygon.edge(i).IsOnLine(next_vertex));
    EXPECT_TRUE(polygon.edge(i).IsOnLine(input_vertex));
    EXPECT_TRUE(polygon.edge(i).IsOnLine(next_input_vertex));
    EXPECT_GT((next_input_vertex - input_vertex).Dot(polygon.edge(i).d()), 0);
  }
}

TEST(ConvexPolygon, ClockwiseSquareEdges) {
  Vertex3<32> input[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(0, 1, 10),
    Vertex3<32>(1, 1, 10),
    Vertex3<32>(1, 0, 10),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);

  ASSERT_EQ(polygon.vertex_count(), std::end(input) - std::begin(input));
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    EXPECT_EQ(polygon.vertex(i), input[i]);
    const ConvexPolygon<32>::Vertex4Rep& vertex = polygon.vertex(i);
    const ConvexPolygon<32>::Vertex4Rep& next_vertex = polygon.vertex(
        (i + 1) % polygon.vertex_count());
    const Vertex3<32>& input_vertex = input[i];
    const Vertex3<32>& next_input_vertex = input[
        (i + 1) % polygon.vertex_count()];
    EXPECT_TRUE(polygon.edge(i).IsOnLine(vertex));
    EXPECT_TRUE(polygon.edge(i).IsOnLine(next_vertex));
    EXPECT_TRUE(polygon.edge(i).IsOnLine(input_vertex));
    EXPECT_TRUE(polygon.edge(i).IsOnLine(next_input_vertex));
    EXPECT_GT((next_input_vertex - input_vertex).Dot(polygon.edge(i).d()), 0);
  }
}

TEST(ConvexPolygon, RedundantEdges) {
  Vertex3<32> input[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(1, 0, 10), // collinear
    Vertex3<32>(2, 0, 10), // collinear
    Vertex3<32>(3, 0, 10),
    Vertex3<32>(3, 1, 10),
    Vertex3<32>(2, 1, 10), // collinear
    Vertex3<32>(1, 1, 10), // collinear
    Vertex3<32>(0, 1, 10),
  };

  ConvexPolygon<32> polygon = MakeConvexPolygon(input);

  ASSERT_EQ(polygon.vertex_count(), std::end(input) - std::begin(input));
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    EXPECT_EQ(polygon.vertex(i), input[i]);
    const ConvexPolygon<32>::Vertex4Rep& vertex = polygon.vertex(i);
    const ConvexPolygon<32>::Vertex4Rep& next_vertex = polygon.vertex(
        (i + 1) % polygon.vertex_count());
    const Vertex3<32>& input_vertex = input[i];
    const Vertex3<32>& next_input_vertex = input[
        (i + 1) % polygon.vertex_count()];
    EXPECT_TRUE(polygon.edge(i).IsOnLine(vertex));
    EXPECT_TRUE(polygon.edge(i).IsOnLine(next_vertex));
    EXPECT_TRUE(polygon.edge(i).IsOnLine(input_vertex));
    EXPECT_TRUE(polygon.edge(i).IsOnLine(next_input_vertex));
    EXPECT_GT((next_input_vertex - input_vertex).Dot(polygon.edge(i).d()), 0);
  }
}

}  // walnut
