#ifndef WALNUT_MONOTONE_DECOMPOSER_H__
#define WALNUT_MONOTONE_DECOMPOSER_H__

// MonotoneDecomposer takes the top and bottom chain from a monotone polygon
// and converts them into one or more convex polygons. The MonotoneDecomposer
// tries to minimize the number of convex polygons it outputs, instead of
// always outputting triangles.

#include <functional>
#include <vector>

#include "walnut/monotone_triangulator.h"

namespace walnut {

template <int vertex3_bits_template = 32>
class MonotoneDecomposer : protected MonotoneTriangulator<vertex3_bits_template> {
 public:
  using Parent = MonotoneTriangulator<vertex3_bits_template>;
  using Vertex3Rep = typename Parent::Vertex3Rep;
  using const_iterator = typename std::vector<Vertex3Rep>::const_iterator;
  using const_reverse_iterator = typename
    std::vector<Vertex3Rep>::const_reverse_iterator;

  // Called to emit a convex polygon. The arguments represent 2
  // iterator ranges of vertices. To get the actual list of vertices for the
  // polygon, calculate range1 + range2.
  using Emitter = std::function<void (const_reverse_iterator,
                                      const_reverse_iterator,
                                      const_iterator,
                                      const_iterator)>;

  // Given a monotone polygon in the form of an iterator range for its top
  // chain and an iterator range for its bottom chain, this converts the
  // monotone polygon into convex polygons.
  //
  // The algorithm tries to output convex polygons with more than 3 vertices.
  //
  // `drop_dimension` is the dimension to ignore on the input vertices in order
  // to project them to 2D.
  //
  // The `monotone_dimension` represents the dimension in which the chains are
  // monotone. That is, for each chain, each successive vertex in that chain
  // must be greater than or equal the previous in the monotone dimension.
  // `monotone_dimension` must not equal `drop_dimension`.
  //
  // The minimum_vertex is the vertex smallest in the monotone dimension. It
  // must be included in only the top chain. The maximum vertex (vertex
  // greatest in the monotone dimension) must be included in only the bottom
  // chain. Violating either of these rules results in undefined behavior.
  //
  // `emit` is called for each convex polygon produced. `emit` must not
  // re-entrantly call back into the same MonotoneDecomposer.
  //
  // Runtime: O(n)
  template <typename TopIterator, typename BottomIterator>
  void Build(Emitter emit, int drop_dimension, int monotone_dimension,
             TopIterator top_begin, TopIterator top_end,
             BottomIterator bottom_begin, BottomIterator bottom_end) {
    assert(convex_top_.empty());
    assert(convex_bottom_.empty());

    emit_ = std::move(emit);
    drop_dimension_ = drop_dimension;
    Parent::Build(drop_dimension, monotone_dimension,
                  top_begin, top_end, bottom_begin, bottom_end);
    emit_(convex_top_.rbegin(), convex_top_.rend(),
          convex_bottom_.begin(), convex_bottom_.end());
    convex_top_.clear();
    convex_bottom_.clear();
  }

 protected:
  void Emit(bool p3_is_top_chain, const Vertex3Rep& p1, const Vertex3Rep& p2, const Vertex3Rep& p3) override {
    if (!convex_top_.empty()) {
      assert(!convex_bottom_.empty());
      if (convex_top_.back() == p1 && convex_bottom_.back() == p2) {
        const Vertex3Rep& prev_top = convex_top_.size() > 1 ?
          convex_top_.end()[-2] : convex_bottom_.front();
        const Vertex3Rep& prev_bottom = convex_bottom_.size() > 1 ?
          convex_bottom_.end()[-2] : convex_top_.front();
        if (p1.Get2DTwistDir(drop_dimension_, prev_top, p3) >= 0 &&
            p2.Get2DTwistDir(drop_dimension_, prev_bottom, p3) <= 0) {
          if (p3_is_top_chain) {
            convex_top_.push_back(p3);
          } else {
            convex_bottom_.push_back(p3);
          }
          return;
        }
      }
      emit_(convex_top_.rbegin(), convex_top_.rend(),
            convex_bottom_.begin(), convex_bottom_.end());
      convex_top_.clear();
      convex_bottom_.clear();
    }
    convex_top_.push_back(p1);
    convex_bottom_.push_back(p2);
    if (p3_is_top_chain) {
      convex_top_.push_back(p3);
    } else {
      convex_bottom_.push_back(p3);
    }
  }

 private:
  // The top of the convex polygon currently being constructed. The vertices
  // are sorted in the monotone dimension.
  std::vector<Vertex3Rep> convex_top_;

  // The bottom of the convex polygon currently being constructed. The vertices
  // are sorted in the monotone dimension.
  std::vector<Vertex3Rep> convex_bottom_;

  Emitter emit_;
  int drop_dimension_;
};

}  // walnut

#endif // WALNUT_MONOTONE_DECOMPOSER_H__
