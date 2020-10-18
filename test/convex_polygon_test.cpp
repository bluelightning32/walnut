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

}  // walnut
