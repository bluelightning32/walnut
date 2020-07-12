#include "walnut/monotone_decomposer.h"

#include <array>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::ElementsAre;

class ResultCollector {
 public:

  MonotoneDecomposer<32>::Emitter GetAppender() {
    return [this](const Vertex3<32>& first,
                  std::vector<Vertex3<32>>::const_iterator range1_begin,
                  std::vector<Vertex3<32>>::const_iterator range1_end,
                  std::vector<Vertex3<32>>::const_reverse_iterator range2_begin,
                  std::vector<Vertex3<32>>::const_reverse_iterator range2_end) {
      result_.emplace_back();
      result_.back().reserve(1 + (range1_end - range1_begin) +
                            (range2_end - range2_begin));
      result_.back().push_back(first);
      result_.back().insert(result_.back().end(), range1_begin, range1_end);
      result_.back().insert(result_.back().end(), range2_begin, range2_end);
    };
  }

  const std::vector<std::vector<Vertex3<32>>>& GetSortedResult() {
    for (std::vector<Vertex3<32>>& polygon : result_) {
      SortVertices(polygon);
    }

    std::sort(result_.begin(), result_.end(), PolygonLt);
    return result_;
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

  static bool PolygonLt(const std::vector<Vertex3<32>>& a,
                        const std::vector<Vertex3<32>>& b) {
    return std::lexicographical_compare(a.begin(), a.end(),
                                        b.begin(), b.end(),
                                        &VertexLt);
  }

 private:
  std::vector<std::vector<Vertex3<32>>> result_;
};

TEST(MonotoneDecomposer, AlreadyConvexAllTopChain) {
  Vertex3<32> minimum_vertex(0, 0, 10);
  Vertex3<32> top_chain[] = {
    Vertex3<32>(1, 3, 10),
    Vertex3<32>(2, 5, 10),
    Vertex3<32>(3, 5, 10),
    Vertex3<32>(4, 3, 10),
    Vertex3<32>(5, 0, 10),
  };
  Vertex3<32> bottom_chain[] = {
    // The maximum vertex must be in the top and bottom chains.
    Vertex3<32>(5, 0, 10),
  };

  MonotoneDecomposer<32> decomposer;
  ResultCollector collector;
  decomposer.Build(collector.GetAppender(), /*drop_dimension=*/2, /*monotone_dimension=*/0,
             /*minimum_vertex=*/minimum_vertex,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedResult(), ElementsAre(std::vector<Vertex3<32>>{
        minimum_vertex, top_chain[4], top_chain[3], top_chain[2], top_chain[1],
        top_chain[0]}));
}

TEST(MonotoneDecomposer, AlreadyConvexAllBottomChain) {
  Vertex3<32> minimum_vertex(0, 0, 10);
  Vertex3<32> top_chain[] = {
    // The maximum vertex must be in the top and bottom chains.
    Vertex3<32>(5, 0, 10),
  };
  Vertex3<32> bottom_chain[] = {
    Vertex3<32>(1, -3, 10),
    Vertex3<32>(2, -5, 10),
    Vertex3<32>(3, -5, 10),
    Vertex3<32>(4, -3, 10),
    Vertex3<32>(5, 0, 10),
  };

  MonotoneDecomposer<32> decomposer;
  ResultCollector collector;
  decomposer.Build(collector.GetAppender(), /*drop_dimension=*/2, /*monotone_dimension=*/0,
             /*minimum_vertex=*/minimum_vertex,
             std::begin(top_chain), std::end(top_chain),
             std::begin(bottom_chain), std::end(bottom_chain));
  EXPECT_THAT(collector.GetSortedResult(), ElementsAre(std::vector<Vertex3<32>>{
        minimum_vertex, bottom_chain[0], bottom_chain[1], bottom_chain[2],
        bottom_chain[3], bottom_chain[4]}));
}

}  // walnut
