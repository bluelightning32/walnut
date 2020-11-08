#include "walnut/monotone_decomposer.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::ElementsAre;

class ResultCollector : public MonotoneDecomposer<Point3<32>> {
 public:
  std::vector<std::vector<Point3<32>>> GetSortedPolygonResult() {
    for (std::pair<int, std::vector<Point3<32>>>& polygon : result_) {
      SortVertices(polygon.second);
    }

    std::sort(result_.begin(), result_.end(), PolygonLt);

    std::vector<std::vector<Point3<32>>> result;
    for (const auto& polygon : result_) {
      result.push_back(polygon.second);
    }
    return result;
  }

  std::vector<int> GetSortedOrientationResult() {
    for (std::pair<int, std::vector<Point3<32>>>& polygon : result_) {
      SortVertices(polygon.second);
    }

    std::sort(result_.begin(), result_.end(), PolygonLt);

    std::vector<int> result;
    for (const auto& polygon : result_) {
      result.push_back(polygon.first);
    }
    return result;
  }

  static void SortVertices(std::vector<Point3<32>>& polygon) {
    std::vector<Point3<32>>::iterator min = polygon.begin();
    for (auto it = polygon.begin(); it != polygon.end(); ++it) {
      if (PointLt(*it, *min)) {
        min = it;
      }
    }
    std::rotate(polygon.begin(), min, polygon.end());
  }

  static bool PointLt(const Point3<32>& a, const Point3<32>& b) {
    return std::lexicographical_compare(a.coords().begin(), a.coords().end(),
                                        b.coords().begin(), b.coords().end());
  }

  static bool PolygonLt(const std::pair<int, std::vector<Point3<32>>>& a,
                        const std::pair<int, std::vector<Point3<32>>>& b) {
    return std::lexicographical_compare(a.second.begin(), a.second.end(),
                                        b.second.begin(), b.second.end(),
                                        &PointLt);
  }

 protected:
  void Emit(int orientation, const_reverse_iterator range1_begin,
            const_reverse_iterator range1_end, const_iterator range2_begin,
            const_iterator range2_end) override {
    result_.emplace_back(orientation, std::vector<Point3<32>>());
    result_.back().second.reserve((range1_end - range1_begin) +
                           (range2_end - range2_begin));
    result_.back().second.insert(result_.back().second.end(), range1_begin,
                                 range1_end);
    result_.back().second.insert(result_.back().second.end(), range2_begin,
                                 range2_end);
  }

 private:
  std::vector<std::pair<int, std::vector<Point3<32>>>> result_;
};

TEST(MonotoneDecomposer, AlreadyConvexAllTopChain) {
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 3, 10),
    Point3<32>(2, 5, 10),
    Point3<32>(3, 5, 10),
    Point3<32>(4, 3, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(5, 0, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3<32>>{top_chain[0], bottom_chain[0], top_chain[4],
                                 top_chain[3], top_chain[2], top_chain[1]}));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        -1));
}

TEST(MonotoneDecomposer, AlreadyConvexAllBottomChain) {
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(1, -3, 10),
    Point3<32>(2, -5, 10),
    Point3<32>(3, -5, 10),
    Point3<32>(4, -3, 10),
    Point3<32>(5, 0, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3<32>>{top_chain[0], bottom_chain[0], bottom_chain[1],
                                 bottom_chain[2], bottom_chain[3],
                                 bottom_chain[4]}));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        -1));
}

TEST(MonotoneDecomposer, AlreadyConvexAlternatingChains) {
  Point3<32> top_chain[] = {
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
    Point3<32>(9, 0, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3<32>>{top_chain[0], bottom_chain[0], bottom_chain[1],
                                 bottom_chain[2], bottom_chain[3],
                                 bottom_chain[4], top_chain[4], top_chain[3],
                                 top_chain[2], top_chain[1]}));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        -1));
}

TEST(MonotoneDecomposer, SingleReflexOnTop) {
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(2, 3, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(3, 0, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3<32>>{top_chain[0], bottom_chain[0], top_chain[1]},
        std::vector<Point3<32>>{top_chain[1], bottom_chain[0], top_chain[2]}
        ));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        -1, -1));
}

TEST(MonotoneDecomposer, MergeCheckReflex1) {
  // MergeCheckConvex1 has roughly the same shape as this test, but the top
  // chain has convex polygons.
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, -1, 10),
    Point3<32>(3, -1, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(2, -5, 10),
    Point3<32>(4, 0, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3<32>>{top_chain[0], bottom_chain[0], top_chain[1]},
        std::vector<Point3<32>>{top_chain[1], bottom_chain[0], top_chain[2]},
        std::vector<Point3<32>>{bottom_chain[0], bottom_chain[1], top_chain[2]}
        ));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        -1, -1, -1));
}

TEST(MonotoneDecomposer, MergeCheckConvex1) {
  // MergeCheckConvex has roughly the same shape as this test, but the top
  // chain has convex polygons.
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(1, 1, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(2, -5, 10),
    Point3<32>(4, 0, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3<32>>{top_chain[0], bottom_chain[0], bottom_chain[1],
                                 top_chain[2], top_chain[1]}
        ));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        -1));
}

TEST(MonotoneDecomposer, MergeCheckReflex2) {
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(4, -7, 10),
    Point3<32>(14, -7, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(1, -8, 10),
    Point3<32>(9, -10, 10),
    Point3<32>(17, -8, 10),
    Point3<32>(18, 0, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3<32>>{top_chain[0], bottom_chain[0], top_chain[1]},
        std::vector<Point3<32>>{bottom_chain[0], bottom_chain[1],
                                 bottom_chain[2], top_chain[2], top_chain[1]},
        std::vector<Point3<32>>{top_chain[2], bottom_chain[2], bottom_chain[3]}
        ));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        -1, -1, -1));
}

TEST(MonotoneDecomposer, MergeCheckReflex3) {
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(4, -3, 10),
    Point3<32>(10, -3, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(1, -2, 10),
    Point3<32>(7, -7, 10),
    Point3<32>(13, -2, 10),
    Point3<32>(14, 0, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3<32>>{top_chain[0], bottom_chain[0],
                                 bottom_chain[1], top_chain[1]},
        std::vector<Point3<32>>{top_chain[1], bottom_chain[1],
                                 top_chain[2]},
        std::vector<Point3<32>>{bottom_chain[1], bottom_chain[2],
                                 bottom_chain[3], top_chain[2]}
        ));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        -1, -1, -1));
}

TEST(MonotoneDecomposer, MergeAfterReflex) {
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(4, 2, 10),
    Point3<32>(6, 6, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(9, -3, 10),
    Point3<32>(14, 0, 10)
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3<32>>{top_chain[0], bottom_chain[0], top_chain[1]},
        std::vector<Point3<32>>{top_chain[1], bottom_chain[0], bottom_chain[1],
                                 top_chain[2]}
        ));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        -1, -1));
}

TEST(MonotoneDecomposer, SelfIntersecting1) {
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(2, 4, 10),
    Point3<32>(7, 4, 10),
    Point3<32>(15, -4, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(4, -4, 10),
    Point3<32>(12, 4, 10),
    Point3<32>(17, 4, 10),
    Point3<32>(19, 0, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3<32>>{top_chain[0], bottom_chain[0], bottom_chain[1],
                                 top_chain[2], top_chain[1]},
        std::vector<Point3<32>>{top_chain[2], bottom_chain[1], bottom_chain[2],
                                 bottom_chain[3], top_chain[3]}
        ));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        -1, 1));
}

TEST(MonotoneDecomposer, SelfIntersecting2) {
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(4, -2, 10),
    Point3<32>(9, -2, 10),
    Point3<32>(17, -8, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(5, -8, 10),
    Point3<32>(14, -2, 10),
    Point3<32>(19, -2, 10),
    Point3<32>(23, 0, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3<32>>{top_chain[0], bottom_chain[0], top_chain[1]},
        std::vector<Point3<32>>{top_chain[1], bottom_chain[0], bottom_chain[1],
                                 top_chain[2]},
        std::vector<Point3<32>>{top_chain[2], bottom_chain[1], bottom_chain[2],
                                 top_chain[3]},
        std::vector<Point3<32>>{top_chain[3], bottom_chain[2], bottom_chain[3]}
        ));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        -1, -1, 1, 1));
}

TEST(MonotoneDecomposer, SelfIntersecting3) {
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(4, 2, 10),
    Point3<32>(6, 6, 10),
    Point3<32>(14, -3, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(9, -3, 10),
    Point3<32>(17, 6, 10),
    Point3<32>(19, 2, 10),
    Point3<32>(23, 0, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3<32>>{top_chain[0], bottom_chain[0], top_chain[1]},
        std::vector<Point3<32>>{top_chain[1], bottom_chain[0], top_chain[3],
                                 top_chain[2]},
        std::vector<Point3<32>>{bottom_chain[0], bottom_chain[1],
                                 bottom_chain[3], top_chain[3]},
        std::vector<Point3<32>>{bottom_chain[1], bottom_chain[2],
                                 bottom_chain[3]}
        ));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        -1, -1, 1, -1));
}

TEST(MonotoneDecomposer, SelfIntersecting4) {
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(3, 1, 10),
    Point3<32>(5, 3, 10),
    Point3<32>(6, 6, 10),
    Point3<32>(10, -3, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(7, -3, 10),
    Point3<32>(11, 6, 10),
    Point3<32>(12, 3, 10),
    Point3<32>(14, 1, 10),
    Point3<32>(17, 0, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3<32>>{top_chain[0], bottom_chain[0], top_chain[1]},
        std::vector<Point3<32>>{top_chain[1], bottom_chain[0], top_chain[2]},
        std::vector<Point3<32>>{top_chain[2], bottom_chain[0], top_chain[4],
                                 top_chain[3]},
        std::vector<Point3<32>>{bottom_chain[0], bottom_chain[1],
                                 top_chain[4]},
        std::vector<Point3<32>>{top_chain[4], bottom_chain[1],
                                 bottom_chain[4]},
        std::vector<Point3<32>>{bottom_chain[1], bottom_chain[2],
                                 bottom_chain[3]},
        std::vector<Point3<32>>{bottom_chain[1], bottom_chain[3],
                                 bottom_chain[4]}
        ));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        -1, -1, -1, 1, 1, -1, -1));
}

TEST(MonotoneDecomposer, AllCollinear) {
  //
  // t0 -> t1 -> t2 -> b0 -> t3 -> b1 -> b2 -> b3
  //
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10),
    Point3<32>(2, 0, 10),
    Point3<32>(4, 0, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(3, 0, 10),
    Point3<32>(5, 0, 10),
    Point3<32>(6, 0, 10),
    Point3<32>(7, 0, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3<32>>{top_chain[0], bottom_chain[0], bottom_chain[1],
                                 bottom_chain[2], bottom_chain[3], top_chain[3],
                                 top_chain[2], top_chain[1]}
        ));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        0));
}

TEST(MonotoneDecomposer, DuplicateReflex) {
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(2, -2, 10),
    Point3<32>(2, -2, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(1, -4, 10),
    Point3<32>(5, -1, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3<32>>{top_chain[0], bottom_chain[0], top_chain[1]},
        std::vector<Point3<32>>{bottom_chain[0], bottom_chain[1],
                                 top_chain[2]}
        ));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        -1, -1));
}

TEST(MonotoneDecomposer, TopChainDegenerateTriangle) {
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(2, -2, 10),
    Point3<32>(3, 0, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(1, -4, 10),
    Point3<32>(4, -1, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3<32>>{top_chain[0], bottom_chain[0], top_chain[1]},
        std::vector<Point3<32>>{bottom_chain[0], bottom_chain[1],
                                 top_chain[2], top_chain[1]}
        ));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        -1, -1));
}

TEST(MonotoneDecomposer, NonstrictlyMonotone) {
  Point3<32> top_chain[] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10),
    Point3<32>(1, -2, 10),
    Point3<32>(1, -1, 10),
    Point3<32>(2, -1, 10),
  };
  Point3<32> bottom_chain[] = {
    Point3<32>(0, -3, 10),
    Point3<32>(1, -3, 10),
    Point3<32>(2, -3, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3<32>>{bottom_chain[0], bottom_chain[1], top_chain[2],
                                 top_chain[1], top_chain[0]},
        std::vector<Point3<32>>{bottom_chain[1], bottom_chain[2],
                                 top_chain[4], top_chain[3], top_chain[2]}
        ));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        -1, -1));
}

}  // walnut
