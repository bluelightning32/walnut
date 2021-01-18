#include "walnut/monotone_triangulator.h"

#include <array>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::ElementsAre;

template <int vertex3_bits_template = 32>
class ResultCollector :
  public MonotoneTriangulator<Point3<vertex3_bits_template>> {
 public:
  using typename MonotoneTriangulator<Point3<vertex3_bits_template>>::Point3Rep;

  void Emit(bool p3_is_top_chain, const Point3Rep& p1, const Point3Rep& p2, const Point3Rep& p3) override {
    result_.emplace_back(std::array<Point3Rep, 3>{p1, p2, p3});
  }

  const std::vector<std::array<Point3<32>, 3>>& result() {
    return result_;
  }

 private:
  std::vector<std::array<Point3Rep, 3>> result_;
};

TEST(MonotoneTriangulator, AlreadyConvexAllTopChain) {
  //
  //       _t2 -> t3_                |
  //      /          \               |
  //    t1            t4             |
  //    /               \            |
  //  t0---------------> b0          |
  //
  //       _t2 -> t3_                |
  //      //  ___/   \               |
  //    t1/  /       -t4             |
  //    /<----------/   \            |
  //  t0---------------> b0          |
  // 
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 3, 10),
    Point3<32>(2, 5, 10),
    Point3<32>(3, 5, 10),
    Point3<32>(4, 3, 10),
  };
  Point3<32> bottom_chain[] = {
    // The maximum vertex must be in the bottom chain.
    Point3<32>(5, 0, 10),
  };

  ResultCollector<32> collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));

  EXPECT_THAT(collector.result(), ElementsAre(
        std::array<Point3<32>, 3>{top_chain[1], top_chain[0], top_chain[2]},
        std::array<Point3<32>, 3>{top_chain[2], top_chain[0], top_chain[3]},
        std::array<Point3<32>, 3>{top_chain[3], top_chain[0], top_chain[4]},
        std::array<Point3<32>, 3>{top_chain[4], top_chain[0], bottom_chain[0]}
        ));
}

TEST(MonotoneTriangulator, AlreadyConvexAllBottomChain) {
  //
  //  t0---------------> b5
  //   \                /
  //    b0           _b3
  //      \         / 
  //       >b1 -> b2 
  //
  //  t0---------------> b4
  //   \ <-----------\  /
  //    b0\    \     _b3
  //      \\     \  / 
  //       >b1 -> b2 
  // 
  Point3<32> top_chain[] = {
    // The minimum vertex must be in the top chain.
    Point3<32>(0, 0, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(1, -3, 10),
    Point3<32>(2, -5, 10),
    Point3<32>(3, -5, 10),
    Point3<32>(4, -3, 10),
    // The maximum vertex must be in the bottom chain.
    Point3<32>(5, 0, 10),
  };

  ResultCollector<32> collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));

  EXPECT_THAT(collector.result(), ElementsAre(
        std::array<Point3<32>, 3>{top_chain[0], bottom_chain[0], bottom_chain[1]},
        std::array<Point3<32>, 3>{top_chain[0], bottom_chain[1], bottom_chain[2]},
        std::array<Point3<32>, 3>{top_chain[0], bottom_chain[2], bottom_chain[3]},
        std::array<Point3<32>, 3>{top_chain[0], bottom_chain[3], bottom_chain[4]}
        ));
}

TEST(MonotoneTriangulator, AlreadyConvexAlternatingChains) {
  //                        
  //       t2--->t3__               |
  //      /          \              |
  //     t1           t4            |
  //    /              \            |
  //   /                \           |
  //  t0                b4          |
  //   \               /            |
  //    b0          b3-             |
  //     \         /                |
  //      b1-->b2--                 |
  //
  Point3<32> top_chain[] = {
    // The minimum vertex must be in the top chain.
    Point3<32>(0, 0, 10),
    Point3<32>(2, 3, 10),
    Point3<32>(4, 5, 10),
    Point3<32>(6, 5, 10),
    Point3<32>(8, 3, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(1, -3, 10),
    Point3<32>(3, -5, 10),
    Point3<32>(5, -5, 10),
    Point3<32>(7, -3, 10),
    // The maximum vertex must be in the bottom chain.
    Point3<32>(9, 0, 10),
  };

  ResultCollector<32> collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));

  EXPECT_THAT(collector.result(), ElementsAre(
        std::array<Point3<32>, 3>{top_chain[0], bottom_chain[0], top_chain[1]},
        std::array<Point3<32>, 3>{top_chain[1], bottom_chain[0], bottom_chain[1]},
        std::array<Point3<32>, 3>{top_chain[1], bottom_chain[1], top_chain[2]},
        std::array<Point3<32>, 3>{top_chain[2], bottom_chain[1], bottom_chain[2]},
        std::array<Point3<32>, 3>{top_chain[2], bottom_chain[2], top_chain[3]},
        std::array<Point3<32>, 3>{top_chain[3], bottom_chain[2], bottom_chain[3]},
        std::array<Point3<32>, 3>{top_chain[3], bottom_chain[3], top_chain[4]},
        std::array<Point3<32>, 3>{top_chain[4], bottom_chain[3], bottom_chain[4]}
        ));
}

TEST(MonotoneTriangulator, SingleReflexOnTop) {
  //
  //          t2            |
  //          /\            |
  //      ->t1  \           |
  //    _/      |           |
  //  t0------> b0          |
  //
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(2, 3, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(3, 0, 10),
  };

  ResultCollector<32> collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));

  EXPECT_THAT(collector.result(), ElementsAre(
        std::array<Point3<32>, 3>{top_chain[1], top_chain[0], bottom_chain[0]},
        std::array<Point3<32>, 3>{top_chain[2], top_chain[1], bottom_chain[0]}
        ));
}

TEST(MonotoneTriangulator, SelfIntersecting) {
  //
  //      t1          t3                 |
  //     /  \        /  \                |
  //    /    \      /    \               |
  //  t0----- \--- /----> b0             |
  //           \  /                      |
  //            t2                       |
  //
  //      t1          t3                 |
  //     /  \        /  \                |
  //    /    \      /    \               |
  //  t0----- \--- /----> b0             |
  //    \__I__ \I /  I   /               |
  //          \>t2------                 |
  //
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(2, -1, 10),
    Point3<32>(3, 1, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(4, 0, 10),
  };

  ResultCollector<32> collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));

  EXPECT_THAT(collector.result(), ElementsAre(
        std::array<Point3<32>, 3>{top_chain[1], top_chain[0], top_chain[2]},
        std::array<Point3<32>, 3>{top_chain[2], top_chain[0], bottom_chain[0]},
        std::array<Point3<32>, 3>{top_chain[3], top_chain[2], bottom_chain[0]}
        ));
}

}  // walnut
