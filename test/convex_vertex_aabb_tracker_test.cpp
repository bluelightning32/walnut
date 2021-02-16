#include "walnut/convex_vertex_aabb_tracker.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/homo_point3.h"

namespace walnut {

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

}  // walnut
