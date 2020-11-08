#ifndef WALNUT_MONOTONE_TRIANGULATOR_H__
#define WALNUT_MONOTONE_TRIANGULATOR_H__

// MonotoneTriangulator takes the top and bottom chain from a monotone polygon
// and converts them intro triangles.

#include <type_traits>
#include <vector>

#include "walnut/point3.h"

namespace walnut {

template <typename Point3RepTemplate = Point3<32>>
class MonotoneTriangulator {
 public:
  using Point3Rep = Point3RepTemplate;
  static_assert(std::is_base_of<Point3<Point3Rep::coord_bits>,
                                Point3Rep>::value,
                "Point3Rep must be derived from Point3.");

  // Given a monotone polygon in the form of an iterator range for its top
  // chain and an iterator range for its bottom chain, this converts the
  // monotone polygon into convex polygons.
  //
  // `drop_dimension` is the dimension to ignore on the input vertices in order
  // to project them to 2D.
  //
  // The `monotone_dimension` represents the dimension in which the chains are
  // monotone. That is, for each chain, each successive vertex in that chain
  // must be greater than or equal the previous in the monotone dimension.
  // `monotone_dimension` must not equal `drop_dimension`.
  //
  // Out of the 3 dimensions, the compare dimension is the remaining one (not
  // drop_dimension and not monotone_dimension). The vertex with the minimum
  // value in the compare dimension should be part of the top chain. The top
  // and bottom chains should not cross each other.
  //
  // The minimum_vertex is the vertex smallest in the monotone dimension. It
  // must be included in only the top chain. The maximum vertex (vertex
  // greatest in the monotone dimension) must be included in only the bottom
  // chain. Violating either of these rules results in undefined behavior.
  //
  // With the top and bottom chains combined, at least 3 vertices must be given
  // as input.
  //
  // `Emit` is called for each triangle produced.
  //
  // After Build returns, `this` is left in a state ready for Build to be
  // called again.
  //
  // Runtime: O(n)
  template <typename TopIterator, typename BottomIterator>
  void Build(int drop_dimension, int monotone_dimension,
             TopIterator top_begin, TopIterator top_end,
             BottomIterator bottom_begin, BottomIterator bottom_end);

 protected:
  // Called to emit a triangle.
  //
  // [p1, p2, p3] will be in counter-clockwise order, when viewed with the
  // `drop_dimension` coordinate set to 0.
  //
  // `p1` and `p2` come from `reflex_stack`. `p3` is a newer vertex from one
  // of the chains. `p3_is_top_chain` is set to true when `p3` comes from the
  // top chain, or false if it comes from the bottom chain.
  virtual void Emit(bool p3_is_top_chain, const Point3Rep& p1, const Point3Rep& p2, const Point3Rep& p3) = 0;

 private:
  void ProcessVertex(int drop_dimension, bool is_top, const Point3Rep& v);

  // Emits triangles for all the vertices waiting in reflex_stack_, and uses
  // `top_chain_is_current_` as the 3rd vertex.
  //
  // This function leaves the reflex_stack_ with 1 element in it.
  void SwitchChains(const Point3Rep& next);

  // This is a stack of vertices that must be closed to form triangles.
  //
  // The stack has at least 2 elements in it (except temporarily before the new vertex
  // has been pushed on, or at the end of `Build` when then whole polygon has
  // been processed).
  //
  // reflex_stack_[0] is that last vertex from the previous chain (or the
  // minimum vertex if the opposite chain has not been visited yet).
  //
  // At any point, either a new triangle will be formed with the top 2 vertices
  // from reflex_stack_ along with a third from the current chain, or else the
  // current vertex will be pushed onto the stack.
  std::vector<const Point3Rep*> reflex_stack_;

  // True if the top of reflex_stack_ came from the top chain, or false if it
  // came from the bottom chain.
  bool top_chain_is_current_;
};

template <typename Point3RepTemplate>
template <typename TopIterator, typename BottomIterator>
void MonotoneTriangulator<Point3RepTemplate>::Build(
    int drop_dimension, int monotone_dimension,
    TopIterator top_begin, TopIterator top_end,
    BottomIterator bottom_begin, BottomIterator bottom_end) {

  TopIterator top_pos = top_begin;
  BottomIterator bottom_pos = bottom_begin;

  // Top chain must contain the minimum vertex.
  assert(top_pos != top_end);
  assert(reflex_stack_.empty());
  reflex_stack_.push_back(&*top_pos);
  ++top_pos;

  // Bottom chain must contain the maximum vertex.
  assert(bottom_pos != bottom_end);
  if (top_pos == top_end ||
      bottom_pos->coords()[monotone_dimension] <
      top_pos->coords()[monotone_dimension]) {
    reflex_stack_.push_back(&*bottom_pos);
    ++bottom_pos;
    top_chain_is_current_ = false;
  } else {
    reflex_stack_.push_back(&*top_pos);
    ++top_pos;
    top_chain_is_current_ = true;
  }

  while (top_pos != top_end) {
    // The bottom chain must have the maximum vertex remaining.
    assert(bottom_pos != bottom_end);
    if (bottom_pos->coords()[monotone_dimension] <
        top_pos->coords()[monotone_dimension]) {
      ProcessVertex(drop_dimension, /*is_top=*/false, *bottom_pos);
      ++bottom_pos;
    } else {
      ProcessVertex(drop_dimension, /*is_top=*/true, *top_pos);
      ++top_pos;
    }
  }
  // Process the remaining bottom chain vertices, up to but not including the
  // maximum vertex.
  BottomIterator bottom_prev = bottom_pos;
  ++bottom_pos;
  while (bottom_pos != bottom_end) {
    ProcessVertex(drop_dimension, /*is_top=*/false, *bottom_prev);
    bottom_prev = bottom_pos;
    ++bottom_pos;
  }
  SwitchChains(*bottom_prev);
  assert(reflex_stack_.size() == 1);
  reflex_stack_.clear();
}

template <typename Point3RepTemplate>
void MonotoneTriangulator<Point3RepTemplate>::ProcessVertex(
    int drop_dimension, bool is_top, const Point3Rep& v) {
  assert(reflex_stack_.size() >= 2);

  if (is_top != top_chain_is_current_) {
    SwitchChains(v);
    top_chain_is_current_ = is_top;
  } else {
    // Keep emitting triangles as long as there are at least 2 elements in the
    // reflex stack, and the top of the reflex stack is now convex.
    while (reflex_stack_.size() >= 2) {
      const Point3Rep* p1;
      const Point3Rep* p2;
      if (is_top) {
        p1 = reflex_stack_.back();
        p2 = *(reflex_stack_.end() - 2);
      } else {
        p1 = *(reflex_stack_.end() - 2);
        p2 = reflex_stack_.back();
      }

      if (p2->Get2DTwistDir(drop_dimension, *p1, v) <= 0) {
        Emit(/*p3_is_top_chain=*/is_top, *p1, *p2, v);
        reflex_stack_.pop_back();
      } else {
        break;
      }
    }
  }
  reflex_stack_.push_back(&v);
}

template <typename Point3RepTemplate>
void MonotoneTriangulator<Point3RepTemplate>::SwitchChains(
    const Point3Rep& next) {
  const Point3Rep* reflex_back = reflex_stack_.back();
  for (int i = 0; i < reflex_stack_.size() - 1; ++i) {
    const Point3Rep* p1;
    const Point3Rep* p2;
    if (top_chain_is_current_) {
      p1 = reflex_stack_[i + 1];
      p2 = reflex_stack_[i];
    } else {
      p1 = reflex_stack_[i];
      p2 = reflex_stack_[i + 1];
    }

    Emit(/*p3_is_top_chain=*/!top_chain_is_current_, *p1, *p2, next);
  }
  reflex_stack_.clear();
  reflex_stack_.push_back(reflex_back);
}

}  // walnut

#endif // WALNUT_MONOTONE_TRIANGULATOR_H__
