#include "walnut/orienting_monotone_decomposer.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::ElementsAre;

struct PolygonResult {
  bool flipped;
  int orientation;
  std::vector<Vertex3<32>> vertices;
};

class ResultCollector : public OrientingMonotoneDecomposer<32> {
 public:
  std::vector<std::vector<Vertex3<32>>> GetSortedPolygonResult() {
    for (PolygonResult& polygon : result_) {
      SortVertices(polygon.vertices);
    }

    std::sort(result_.begin(), result_.end(), PolygonLt);

    std::vector<std::vector<Vertex3<32>>> result;
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

  static void SortVertices(std::vector<Vertex3<32>>& polygon) {
    std::vector<Vertex3<32>>::iterator min = polygon.begin();
    for (auto it = polygon.begin(); it != polygon.end(); ++it) {
      if (VertexLt(*it, *min)) {
        min = it;
      }
    }
    std::rotate(polygon.begin(), min, polygon.end());
  }

  static bool VertexLt(const Vertex3<32>& a, const Vertex3<32>& b) {
    return std::lexicographical_compare(a.coords().begin(), a.coords().end(),
                                        b.coords().begin(), b.coords().end());
  }

  static bool PolygonLt(const PolygonResult& a,
                        const PolygonResult& b) {
    return std::lexicographical_compare(a.vertices.begin(), a.vertices.end(),
                                        b.vertices.begin(), b.vertices.end(),
                                        &VertexLt);
  }

 protected:
  void EmitOriented(bool flipped, int orientation,
                    const_reverse_iterator range1_begin,
                    const_reverse_iterator range1_end,
                    const_iterator range2_begin,
                    const_iterator range2_end) override {
    result_.emplace_back();
    result_.back().flipped = flipped;
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
  Vertex3<32> top_chain[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(1, 3, 10),
    Vertex3<32>(2, 5, 10),
    Vertex3<32>(3, 5, 10),
    Vertex3<32>(4, 3, 10),
    Vertex3<32>(5, 0, 10),
  };
  Vertex3<32> bottom_chain[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(5, 0, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(bottom_chain), std::end(bottom_chain),
             std::begin(top_chain), std::end(top_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Vertex3<32>>{top_chain[0], top_chain[5], top_chain[4],
                                 top_chain[3], top_chain[2], top_chain[1]}));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        -1));
}

TEST(OrientingMonotoneDecomposer, Flipped) {
  Vertex3<32> top_chain[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(1, 3, 10),
    Vertex3<32>(2, 5, 10),
    Vertex3<32>(3, 5, 10),
    Vertex3<32>(4, 3, 10),
    Vertex3<32>(5, 0, 10),
  };
  Vertex3<32> bottom_chain[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(5, 0, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Vertex3<32>>{top_chain[0], top_chain[1], top_chain[2],
                                 top_chain[3], top_chain[4], top_chain[5]}));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        1));
}

TEST(OrientingMonotoneDecomposer, AllCollinear) {
  Vertex3<32> top_chain[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(1, 1, 10),
    Vertex3<32>(4, 4, 10),
    Vertex3<32>(5, 5, 10),
  };
  Vertex3<32> bottom_chain[] = {
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(2, 2, 10),
    Vertex3<32>(3, 3, 10),
    Vertex3<32>(5, 5, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(bottom_chain), std::end(bottom_chain),
             std::begin(top_chain), std::end(top_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Vertex3<32>>{top_chain[0], top_chain[1], top_chain[2],
                                 top_chain[3], bottom_chain[2],
                                 bottom_chain[1]}));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        0));
}

TEST(OrientingMonotoneDecomposer, CollinearPrefixNotFlipped) {
  Vertex3<32> top_chain[] = {
    Vertex3<32>(-2, 0, 10),
    Vertex3<32>(-1, 0, 10),
    Vertex3<32>(1, 4, 10),
    Vertex3<32>(2, 4, 10),
    Vertex3<32>(3, 0, 10),
  };
  Vertex3<32> bottom_chain[] = {
    Vertex3<32>(-2, 0, 10),
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(3, 0, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(bottom_chain), std::end(bottom_chain),
             std::begin(top_chain), std::end(top_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Vertex3<32>>{top_chain[0], bottom_chain[1], top_chain[1]},
        std::vector<Vertex3<32>>{top_chain[1], bottom_chain[1],
                                 bottom_chain[2], top_chain[3], top_chain[2]}));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        0, -1));
}

TEST(OrientingMonotoneDecomposer, CollinearPrefixFlipped) {
  Vertex3<32> top_chain[] = {
    Vertex3<32>(-2, 0, 10),
    Vertex3<32>(-1, 0, 10),
    Vertex3<32>(1, 4, 10),
    Vertex3<32>(2, 4, 10),
    Vertex3<32>(3, 0, 10),
  };
  Vertex3<32> bottom_chain[] = {
    Vertex3<32>(-2, 0, 10),
    Vertex3<32>(0, 0, 10),
    Vertex3<32>(3, 0, 10),
  };

  ResultCollector collector;
  collector.Build(/*drop_dimension=*/2, /*monotone_dimension=*/0,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedPolygonResult(), ElementsAre(
        std::vector<Vertex3<32>>{top_chain[0], top_chain[1], bottom_chain[1]},
        std::vector<Vertex3<32>>{top_chain[1], top_chain[2], top_chain[3],
                                 top_chain[4], bottom_chain[1]}));
  EXPECT_THAT(collector.GetSortedOrientationResult(), ElementsAre(
        0, 1));
}

}  // walnut
