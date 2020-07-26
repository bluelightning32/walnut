#include "walnut/monotone_triangulator.h"

#include <array>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::ElementsAre;

template <int vertex3_bits_template = 32>
class ResultCollector : public MonotoneTriangulator<vertex3_bits_template> {
 public:
  using typename MonotoneTriangulator<vertex3_bits_template>::Vertex3Rep;

  void Emit(bool p3_is_top_chain, const Vertex3Rep& p1, const Vertex3Rep& p2, const Vertex3Rep& p3) override {
    result_.emplace_back(std::array<Vertex3Rep, 3>{p1, p2, p3});
  }

  const std::vector<std::array<Vertex3<32>, 3>>& result() {
    return result_;
  }

 private:
  std::vector<std::array<Vertex3Rep, 3>> result_;
};

TEST(MonotoneTriangulator, AlreadyConvexAllTopChain) {
  //
  //       _t2 -> t3_
  //      /          \
  //    t1            t4
  //    /               \
  //  t0---------------> b0
  //
  //       _t2 -> t3_
  //      //  ___/   \
  //    t1/  /       -t4
  //    /<----------/   \
  //  t0---------------> b0
  // 
  Vertex3<32> top_chain[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(1, 3, 10),
    Vertex3<32>(2, 5, 10),
    Vertex3<32>(3, 5, 10),
    Vertex3<32>(4, 3, 10),
  };
  Vertex3<32> bottom_chain[] = {
    // The maximum vertex must be in the bottom chain.
    Vertex3<32>(5, 0, 10),
  };

  ResultCollector<32> collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));

  EXPECT_THAT(collector.result(), ElementsAre(
        std::array<Vertex3<32>, 3>{top_chain[1], top_chain[0], top_chain[2]},
        std::array<Vertex3<32>, 3>{top_chain[2], top_chain[0], top_chain[3]},
        std::array<Vertex3<32>, 3>{top_chain[3], top_chain[0], top_chain[4]},
        std::array<Vertex3<32>, 3>{top_chain[4], top_chain[0], bottom_chain[0]}
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
  Vertex3<32> top_chain[] = {
    // The minimum vertex must be in the top chain.
    Vertex3<32>(0, 0, 10),
  };
  Vertex3<32> bottom_chain[] = {
    Vertex3<32>(1, -3, 10),
    Vertex3<32>(2, -5, 10),
    Vertex3<32>(3, -5, 10),
    Vertex3<32>(4, -3, 10),
    // The maximum vertex must be in the bottom chain.
    Vertex3<32>(5, 0, 10),
  };

  ResultCollector<32> collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));

  EXPECT_THAT(collector.result(), ElementsAre(
        std::array<Vertex3<32>, 3>{top_chain[0], bottom_chain[0], bottom_chain[1]},
        std::array<Vertex3<32>, 3>{top_chain[0], bottom_chain[1], bottom_chain[2]},
        std::array<Vertex3<32>, 3>{top_chain[0], bottom_chain[2], bottom_chain[3]},
        std::array<Vertex3<32>, 3>{top_chain[0], bottom_chain[3], bottom_chain[4]}
        ));
}

TEST(MonotoneTriangulator, AlreadyConvexAlternatingChains) {
  //                        
  //       t2--->t3__
  //      /          \
  //     t1           t4
  //    /              \
  //   /                \
  //  t0                b4
  //   \               /
  //    b0          b3-
  //     \         /
  //      b1-->b2--
  //
  Vertex3<32> top_chain[] = {
    // The minimum vertex must be in the top chain.
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(2, 3, 10),
    Vertex3<32>(4, 5, 10),
    Vertex3<32>(6, 5, 10),
    Vertex3<32>(8, 3, 10),
  };
  Vertex3<32> bottom_chain[] = {
    Vertex3<32>(1, -3, 10),
    Vertex3<32>(3, -5, 10),
    Vertex3<32>(5, -5, 10),
    Vertex3<32>(7, -3, 10),
    // The maximum vertex must be in the bottom chain.
    Vertex3<32>(9, 0, 10),
  };

  ResultCollector<32> collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));

  EXPECT_THAT(collector.result(), ElementsAre(
        std::array<Vertex3<32>, 3>{top_chain[0], bottom_chain[0], top_chain[1]},
        std::array<Vertex3<32>, 3>{top_chain[1], bottom_chain[0], bottom_chain[1]},
        std::array<Vertex3<32>, 3>{top_chain[1], bottom_chain[1], top_chain[2]},
        std::array<Vertex3<32>, 3>{top_chain[2], bottom_chain[1], bottom_chain[2]},
        std::array<Vertex3<32>, 3>{top_chain[2], bottom_chain[2], top_chain[3]},
        std::array<Vertex3<32>, 3>{top_chain[3], bottom_chain[2], bottom_chain[3]},
        std::array<Vertex3<32>, 3>{top_chain[3], bottom_chain[3], top_chain[4]},
        std::array<Vertex3<32>, 3>{top_chain[4], bottom_chain[3], bottom_chain[4]}
        ));
}

TEST(MonotoneTriangulator, SingleReflexOnTop) {
  //
  //          t2
  //          /\
  //      ->t1  \
  //    _/      |
  //  t0------> b0
  //
  Vertex3<32> top_chain[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(1, 1, 10),
    Vertex3<32>(2, 3, 10),
  };
  Vertex3<32> bottom_chain[] = {
    Vertex3<32>(3, 0, 10),
  };

  ResultCollector<32> collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));

  EXPECT_THAT(collector.result(), ElementsAre(
        std::array<Vertex3<32>, 3>{top_chain[1], top_chain[0], bottom_chain[0]},
        std::array<Vertex3<32>, 3>{top_chain[2], top_chain[1], bottom_chain[0]}
        ));
}

TEST(MonotoneTriangulator, SelfIntersecting) {
  //
  //      t1          t3
  //     /  \        /  \
  //    /    \      /    \
  //  t0----- \--- /----> b0
  //           \  /
  //            t2
  //
  //      t1          t3
  //     /  \        /  \
  //    /    \      /    \
  //  t0----- \--- /----> b0
  //    \__I__ \I /  I   /
  //          \>t2------
  //
  Vertex3<32> top_chain[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(1, 1, 10),
    Vertex3<32>(2, -1, 10),
    Vertex3<32>(3, 1, 10),
  };
  Vertex3<32> bottom_chain[] = {
    Vertex3<32>(4, 0, 10),
  };

  ResultCollector<32> collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));

  EXPECT_THAT(collector.result(), ElementsAre(
        std::array<Vertex3<32>, 3>{top_chain[1], top_chain[0], top_chain[2]},
        std::array<Vertex3<32>, 3>{top_chain[2], top_chain[0], bottom_chain[0]},
        std::array<Vertex3<32>, 3>{top_chain[3], top_chain[2], bottom_chain[0]}
        ));
}

}  // walnut
