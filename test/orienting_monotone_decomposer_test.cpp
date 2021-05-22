#include "walnut/orienting_monotone_decomposer.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::ElementsAre;

template <typename Point3Rep>
class ResultCollector : public OrientingMonotoneDecomposer<Point3Rep> {
 public:
  struct PolygonResult {
    int orientation;
    std::vector<Point3Rep> vertices;
  };

  using Parent = OrientingMonotoneDecomposer<Point3Rep>;
  using typename Parent::const_reverse_iterator;
  using typename Parent::const_iterator;

  std::vector<std::vector<Point3Rep>> GetSortedPolygonResult() {
    for (PolygonResult& polygon : result_) {
      SortVertices(polygon.vertices);
    }

    std::sort(result_.begin(), result_.end(), PolygonLt);

    std::vector<std::vector<Point3Rep>> result;
    for (const auto& polygon : result_) {
      result.push_back(polygon.vertices);
    }
    return result;
  }

  std::vector<int> GetSortedOrientationResult() {
    for (PolygonResult& polygon : result_) {
      SortVertices(polygon.vertices);
    }

    std::sort(result_.begin(), result_.end(), PolygonLt);

    std::vector<int> result;
    for (const auto& polygon : result_) {
      result.push_back(polygon.orientation);
    }
    return result;
  }

  static void SortVertices(std::vector<Point3Rep>& polygon) {
    typename std::vector<Point3Rep>::iterator min = polygon.begin();
    for (auto it = polygon.begin(); it != polygon.end(); ++it) {
      if (PointLt(*it, *min)) {
        min = it;
      }
    }
    std::rotate(polygon.begin(), min, polygon.end());
  }

  static bool PointLt(const Point3Rep& a, const Point3Rep& b) {
    return a.LexicographicallyLt(b);
  }

  static bool PolygonLt(const PolygonResult& a,
                        const PolygonResult& b) {
    return std::lexicographical_compare(a.vertices.begin(), a.vertices.end(),
                                        b.vertices.begin(), b.vertices.end(),
                                        &PointLt);
  }

 protected:
  void EmitOriented(int orientation,
                    const_reverse_iterator range1_begin,
                    const_reverse_iterator range1_end,
                    const_iterator range2_begin,
                    const_iterator range2_end) override {
    result_.emplace_back();
    result_.back().orientation = orientation;
    result_.back().vertices.reserve((range1_end - range1_begin) +
                           (range2_end - range2_begin));
    result_.back().vertices.insert(result_.back().vertices.end(), range1_begin,
                                   range1_end);
    result_.back().vertices.insert(result_.back().vertices.end(), range2_begin,
                                   range2_end);
  }

 private:
  std::vector<PolygonResult> result_;
};

TEST(OrientingMonotoneDecomposer, NotFlipped) {
  Point3 top_chain[] = {
    Point3(0, 0, 10),
    Point3(1, 3, 10),
    Point3(2, 5, 10),
    Point3(3, 5, 10),
    Point3(4, 3, 10),
    Point3(5, 0, 10),
  };
  Point3 bottom_chain[] = {
    Point3(0, 0, 10),
    Point3(5, 0, 10),
  };

  ResultCollector<Point3> collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(bottom_chain), std::end(bottom_chain),
             std::begin(top_chain), std::end(top_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3>{top_chain[0], top_chain[5], top_chain[4],
                                 top_chain[3], top_chain[2], top_chain[1]}));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        -1));
}

TEST(OrientingMonotoneDecomposer, Flipped) {
  Point3 top_chain[] = {
    Point3(0, 0, 10),
    Point3(1, 3, 10),
    Point3(2, 5, 10),
    Point3(3, 5, 10),
    Point3(4, 3, 10),
    Point3(5, 0, 10),
  };
  Point3 bottom_chain[] = {
    Point3(0, 0, 10),
    Point3(5, 0, 10),
  };

  ResultCollector<Point3> collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3>{top_chain[0], top_chain[1], top_chain[2],
                                 top_chain[3], top_chain[4], top_chain[5]}));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        1));
}

TEST(OrientingMonotoneDecomposer, AllCollinear) {
  Point3 top_chain[] = {
    Point3(0, 0, 10),
    Point3(1, 1, 10),
    Point3(4, 4, 10),
    Point3(5, 5, 10),
  };
  Point3 bottom_chain[] = {
    Point3(0, 0, 10),
    Point3(2, 2, 10),
    Point3(3, 3, 10),
    Point3(5, 5, 10),
  };

  ResultCollector<Point3> collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(bottom_chain), std::end(bottom_chain),
             std::begin(top_chain), std::end(top_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3>{top_chain[0], top_chain[1], top_chain[2],
                                 top_chain[3], bottom_chain[2],
                                 bottom_chain[1]}));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        0));
}

TEST(OrientingMonotoneDecomposer, CollinearPrefixNotFlipped) {
  Point3 top_chain[] = {
    Point3(-2, 0, 10),
    Point3(-1, 0, 10),
    Point3(1, 4, 10),
    Point3(2, 4, 10),
    Point3(3, 0, 10),
  };
  Point3 bottom_chain[] = {
    Point3(-2, 0, 10),
    Point3(0, 0, 10),
    Point3(3, 0, 10),
  };

  ResultCollector<Point3> collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(bottom_chain), std::end(bottom_chain),
             std::begin(top_chain), std::end(top_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3>{top_chain[0], bottom_chain[1], top_chain[1]},
        std::vector<Point3>{top_chain[1], bottom_chain[1],
                                 bottom_chain[2], top_chain[3], top_chain[2]}));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        0, -1));
}

TEST(OrientingMonotoneDecomposer, CollinearPrefixFlipped) {
  Point3 top_chain[] = {
    Point3(-2, 0, 10),
    Point3(-1, 0, 10),
    Point3(1, 4, 10),
    Point3(2, 4, 10),
    Point3(3, 0, 10),
  };
  Point3 bottom_chain[] = {
    Point3(-2, 0, 10),
    Point3(0, 0, 10),
    Point3(3, 0, 10),
  };

  ResultCollector<Point3> collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Point3>{top_chain[0], top_chain[1], bottom_chain[1]},
        std::vector<Point3>{top_chain[1], top_chain[2], top_chain[3],
                                 top_chain[4], bottom_chain[1]}));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        0, 1));
}

}  // walnut
