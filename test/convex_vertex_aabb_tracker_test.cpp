#include "walnut/convex_vertex_aabb_tracker.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/convex_polygon_factory.h"
#include "walnut/homo_point3.h"

namespace walnut {

template<typename Container>
MutableConvexPolygon<32> MakeUnsortedConvexPolygon(const Container& vertices) {
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
MutableConvexPolygon<32> MakeConvexPolygon(const Container& vertices) {
  auto result = MakeUnsortedConvexPolygon(vertices);
  result.SortVertices();
  return result;
}

TEST(ConvexVertexAABBTracker, ConstructEmpty) {
  std::vector<HomoPoint3<>> vertices;
  ConvexVertexAABBTracker<> tracker(vertices.begin(), vertices.end());

  EXPECT_EQ(tracker.aabb(), AABB<>());
}

TEST(ConvexVertexAABBTracker, ConstructFromOne) {
  std::vector<HomoPoint3<>> vertices{
    HomoPoint3<>{1, 2, 3, 4}
  };
  ConvexVertexAABBTracker<> tracker(vertices.begin(), vertices.end());

  EXPECT_EQ(tracker.min_indices(), (std::array<size_t, 3>{0, 0, 0}));
  EXPECT_EQ(tracker.max_indices(), (std::array<size_t, 3>{0, 0, 0}));
  EXPECT_EQ(tracker.aabb(), AABB<>(/*min_x=*/1, /*min_y=*/2, /*min_z=*/3,
                                   /*max_x=*/1, /*max_y=*/2, /*max_z=*/3,
                                   /*denom=*/4));
}

TEST(ConvexVertexAABBTracker, ConstructFromDifferentDenoms) {
  std::vector<HomoPoint3<>> vertices{
    HomoPoint3<>{1, 2, 3, 4},
    HomoPoint3<>{1, 2, 3, -5}
  };
  ConvexVertexAABBTracker<> tracker(vertices.begin(), vertices.end());

  EXPECT_EQ(tracker.min_indices(), (std::array<size_t, 3>{1, 1, 1}));
  EXPECT_EQ(tracker.max_indices(), (std::array<size_t, 3>{0, 0, 0}));
  EXPECT_EQ(tracker.aabb(), AABB<>(/*min_x=*/-1, /*min_y=*/-2, /*min_z=*/-3,
                                   /*max_x=*/2, /*max_y=*/3, /*max_z=*/4,
                                   /*denom=*/5));
}

TEST(ConvexVertexAABBTracker, RotateIndices) {
  std::vector<HomoPoint3<>> vertices{
    HomoPoint3<>{1, 2, 3, 1},
    HomoPoint3<>{2, 3, 4, 1},
    HomoPoint3<>{3, 4, 1, 1},
    HomoPoint3<>{4, 1, 2, 1},
  };
  ConvexVertexAABBTracker<> tracker(vertices.begin(), vertices.end());

  EXPECT_EQ(tracker.min_indices(), (std::array<size_t, 3>{0, 3, 2}));
  EXPECT_EQ(tracker.max_indices(), (std::array<size_t, 3>{3, 2, 1}));

  ConvexVertexAABBTracker<> rotated(tracker);
  rotated.RotateIndices(2, vertices.size());

  EXPECT_EQ(rotated.min_indices(), (std::array<size_t, 3>{2, 1, 0}));
  EXPECT_EQ(rotated.max_indices(), (std::array<size_t, 3>{1, 0, 3}));

  EXPECT_EQ(rotated.aabb(), tracker.aabb());
}

TEST(ConvexVertexAABBTracker, DefaultConstructsInvalid) {
  ConvexVertexAABBTracker<> tracker;
  EXPECT_TRUE(tracker.IsValidState(0));
}

TEST(ConvexVertexAABBTracker, SplitAtExistingVertices) {
  //
  //     p[3]       |
  //     /  ^       |
  //    v    \      |
  //  p[0]  p[2]    |
  //    \    ^      |
  //     v  /       |
  //     p[1]       |
  //
  Point3 p[] = {
    Point3(0, 0, 10),
    Point3(1, -1, 9),
    Point3(2, 0, 12),
    Point3(1, 1, 13),
  };

  ConvexPolygon<32> square = MakeConvexPolygon(p);

  ConvexVertexAABBTracker<> tracker(square.vertices_begin(),
                                    square.vertices_end());

  EXPECT_EQ(tracker.min_indices(), (std::array<size_t, 3>{0, 1, 1}));
  EXPECT_EQ(tracker.max_indices(), (std::array<size_t, 3>{2, 3, 3}));

  for (size_t i = 0; i < 4; ++i) {
    HalfSpace3 split(p[i], p[(i + 2) % square.vertex_count()],
                       Point3(0, 0, 0));

    ConvexPolygon<>::SplitInfoRep split_info = square.GetSplitInfo(split);
    ASSERT_TRUE(split_info.ShouldEmitNegativeChild());
    ASSERT_TRUE(split_info.ShouldEmitPositiveChild());
    EXPECT_FALSE(split_info.has_new_shared_point1);
    EXPECT_FALSE(split_info.has_new_shared_point2);

    std::pair<ConvexPolygon<>, ConvexPolygon<>> polygon_children =
      square.CreateSplitChildren(split_info);

    std::pair<ConvexVertexAABBTracker<>,
              ConvexVertexAABBTracker<>> tracker_children =
      tracker.CreateSplitChildren(square.vertex_count(),
                                  polygon_children.first.vertices_begin(),
                                  polygon_children.second.vertices_begin(),
                                  split_info.ranges);
    EXPECT_EQ(tracker_children.first,
        ConvexVertexAABBTracker<>(polygon_children.first.vertices_begin(),
                                  polygon_children.first.vertices_end()));
    EXPECT_EQ(tracker_children.second,
        ConvexVertexAABBTracker<>(polygon_children.second.vertices_begin(),
                                  polygon_children.second.vertices_end()))
      << "split_info=" << split_info;
  }
}

TEST(ConvexVertexAABBTracker, SplitBetweenVertices) {
  //
  //     p[3]       |
  //     /  ^       |
  //    v    \      |
  //  p[0]  p[2]    |
  //    \    ^      |
  //     v  /       |
  //     p[1]       |
  //
  Point3 p[] = {
    Point3(0, 0, 10),
    Point3(1, -1, 9),
    Point3(2, 0, 12),
    Point3(1, 1, 13),
  };

  ConvexPolygon<32> square = MakeConvexPolygon(p);

  ConvexVertexAABBTracker<> tracker(square.vertices_begin(),
                                    square.vertices_end());

  EXPECT_EQ(tracker.min_indices(), (std::array<size_t, 3>{0, 1, 1}));
  EXPECT_EQ(tracker.max_indices(), (std::array<size_t, 3>{2, 3, 3}));

  for (size_t i = 0; i < 4; ++i) {
    HomoPoint3<> split_start(
        p[i].vector_from_origin() +
        p[(i + 1) % square.vertex_count()].vector_from_origin(),
        /*w=*/BigInt<32>(2));
    HomoPoint3<> split_end(
        p[(i + 2) % square.vertex_count()].vector_from_origin() +
        p[(i + 3) % square.vertex_count()].vector_from_origin(),
        /*w=*/BigInt<32>(2));
    HalfSpace3 split(split_start, split_end, HomoPoint3<>(0, 0, 0, 1));

    ConvexPolygon<>::SplitInfoRep split_info = square.GetSplitInfo(split);
    ASSERT_TRUE(split_info.ShouldEmitNegativeChild());
    ASSERT_TRUE(split_info.ShouldEmitPositiveChild());
    EXPECT_TRUE(split_info.has_new_shared_point1);
    EXPECT_TRUE(split_info.has_new_shared_point2);

    std::pair<ConvexPolygon<>, ConvexPolygon<>> polygon_children =
      square.CreateSplitChildren(split_info);

    std::pair<ConvexVertexAABBTracker<>,
              ConvexVertexAABBTracker<>> tracker_children =
      tracker.CreateSplitChildren(square.vertex_count(),
                                  polygon_children.first.vertices_begin(),
                                  polygon_children.second.vertices_begin(),
                                  split_info.ranges);
    EXPECT_EQ(tracker_children.first,
        ConvexVertexAABBTracker<>(polygon_children.first.vertices_begin(),
                                  polygon_children.first.vertices_end()));
    EXPECT_EQ(tracker_children.second,
        ConvexVertexAABBTracker<>(polygon_children.second.vertices_begin(),
                                  polygon_children.second.vertices_end()))
      << "split_info=" << split_info;
  }
}

}  // walnut
