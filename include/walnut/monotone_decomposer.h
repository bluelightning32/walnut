#ifndef WALNUT_MONOTONE_DECOMPOSER_H__
#define WALNUT_MONOTONE_DECOMPOSER_H__

// MonotoneDecomposer takes the top and bottom chain from a monotone polygon
// and converts them into one or more convex polygons. The MonotoneDecomposer
// tries to minimize the number of convex polygons it outputs, instead of
// always outputting triangles.

#include <functional>
#include <vector>

#include "walnut/plane.h"
#include "walnut/vertex3.h"
#include "walnut/vertex4.h"

namespace walnut {

template <int vertex3_bits_template = 32>
class MonotoneDecomposer {
 public:
  using Vertex3Rep = Vertex3<vertex3_bits_template>;
  using const_iterator = typename std::vector<Vertex3Rep>::const_iterator;
  using const_reverse_iterator = typename
    std::vector<Vertex3Rep>::const_reverse_iterator;
  // Called to emit a convex polygon. The arguments represent 1 vertex and 2
  // iterator ranges of vertices. To get the actual list of vertices for the
  // polygon, calculate [v, *range1, *range2].
  using Emitter = std::function<void (const Vertex3Rep&,
                                      const_iterator, const_iterator,
                                      const_reverse_iterator,
                                      const_reverse_iterator)>;

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
  // `minimum_vertex` is the vertex smallest in the monotone dimension. It must
  // not be included in either the top chain or bottom chain. The maximum
  // vertex (vertex greatest in the monotone dimension) must be included in
  // both the top and bottom chain.
  //
  // `emit` is called for each convex polygon produced. `emit` must not
  // re-entrantly call back into the same MonotoneDecomposer.
  //
  // Runtime: O(n)
  template <typename TopIterator, typename BottomIterator>
  void Build(Emitter emit, int drop_dimension, int monotone_dimension,
             const Vertex3Rep& minimum_vertex,
             TopIterator top_begin, TopIterator top_end,
             BottomIterator bottom_begin, BottomIterator bottom_end) {
    reflex_stack_.push_back(minimum_vertex);
    TopIterator top_pos = top_begin;
    BottomIterator bottom_pos = bottom_begin;
    top_prev_ = &minimum_vertex;
    bottom_prev_ = &minimum_vertex;

    // Stop before the maximum vertex
    --bottom_end;
    --top_end;

    while (top_pos != top_end || bottom_pos != bottom_end) {
      while (top_pos != top_end && top_pos->coords()[monotone_dimension] <=
                                   bottom_pos->coords()[monotone_dimension]) {
        AddTopVertex(emit, drop_dimension, *top_pos,
                     /*top_next=*/*(top_pos + 1));
        top_prev_ = &*top_pos;
        ++top_pos;
      }
      CloseSide(emit, /*top=*/true, /*last=*/*top_prev_, /*next=*/*bottom_pos);
      while (bottom_pos != bottom_end &&
             bottom_pos->coords()[monotone_dimension] <
             top_pos->coords()[monotone_dimension]) {
        AddBottomVertex(emit, drop_dimension, *bottom_pos,
                        /*bottom_next=*/*(bottom_pos + 1));
        bottom_prev_ = &*bottom_pos;
        ++bottom_pos;
      }
      CloseSide(emit, /*top=*/false, /*last=*/*bottom_prev_, /*next=*/*top_pos);
    }
    assert(reflex_stack_.size() == 1);
    if (!convex_top_.empty() || !convex_bottom_.empty()) {
      convex_top_.push_back(*top_pos);
      // It doesn't matter what is passed for last_prev_side, because it is only
      // used to set the reflex_stack_, which will be thrown out.
      EmitConvex(emit, /*last_prev_side=*/minimum_vertex);
    }
    assert(convex_top_.empty());
    assert(convex_bottom_.empty());
    assert(reflex_stack_.size() == 1);
    reflex_stack_.clear();
  }

 private:

  // Adds a vertex from the top chain to either `convex_top_` or
  // `reflex_stack_`.
  //
  // `cur` is the vertex to add to `convex_top_` or `reflex_stack_`.
  //
  // `next` is the next vertex on the top chain, or the maximum vertex if there
  // are no more vertexes located only on the top chain.
  void AddTopVertex(const Emitter& emit, int drop_dimension,
                    const Vertex3Rep& cur,
                    const Vertex3Rep& next) {
    // As long as `cur` is a reflex vertex relative to the back of convex_top_
    // (or relative to the end of reflex_stack_ if convex_top_ is empty), try
    // to emit polygons from the convex list or from the end of the reflex
    // stack in hopes that `cur` is convex relative to what remains.
    //
    // If no more polygons can be emitted, resort to adding `cur` to
    // `reflex_stack_`.
    while (cur.Get2DTwistDir(drop_dimension, *top_prev_, next) <= 0) {
      if (convex_top_.empty() && convex_bottom_.empty()) {
        // No more polygons can be emitted to make `cur` convex. It has to be
        // added to `reflex_stack_`.
        reflex_stack_.push_back(cur);
        return;
      }
      convex_top_.push_back(cur);
      EmitConvex(emit, *bottom_prev_);
      top_prev_ = &reflex_stack_.back();
      // If the end of the reflex_stack_ is now convex relative to `cur`, move
      // it to the convex list.
      //
      // Note that since this is the top chain, clockwise means convex.
      if (reflex_stack_.size() > 1 &&
          top_prev_->Get2DTwistDir(drop_dimension, *(reflex_stack_.end() - 2), cur) < 0) {
        // Add top_prev_ to the convex list.
        convex_top_.push_back(*top_prev_);
        // Remove it from the reflex stack.
        reflex_stack_.pop_back();
        top_prev_ = &convex_top_.back();
      }
    }
    // At this point (top_prev_, cur, next) is convex.
    assert(cur.Get2DTwistDir(drop_dimension, *top_prev_, next) > 0);
    convex_top_.push_back(cur);
  }

  // Adds a vertex from the bottom chain to either `convex_bottom_` or
  // `reflex_stack_`.
  //
  // `cur` is the vertex to add to `convex_bottom_` or `reflex_stack_`.
  //
  // `next` is the next vertex on the bottom chain, or the maximum vertex if
  // there are no more vertexes located only on the bottom chain.
  void AddBottomVertex(const Emitter& emit, int drop_dimension,
                       const Vertex3Rep& cur,
                       const Vertex3Rep& next) {
    // As long as `cur` is a reflex vertex relative to the back of convex_bottom_
    // (or relative to the end of reflex_stack_ if convex_bottom_ is empty), try
    // to emit polygons from the convex list or from the end of the reflex
    // stack in hopes that `cur` is convex relative to what remains.
    //
    // If no more polygons can be emitted, resort to adding `cur` to
    // `reflex_stack_`.
    while (cur.Get2DTwistDir(drop_dimension, *bottom_prev_, next) >= 0) {
      if (convex_top_.empty() && convex_bottom_.empty()) {
        // No more polygons can be emitted to make `cur` convex. It has to be
        // added to `reflex_stack_`.
        reflex_stack_.push_back(cur);
        return;
      }
      convex_bottom_.push_back(cur);
      EmitConvex(emit, *bottom_prev_);
      bottom_prev_ = &reflex_stack_.back();
      // If the end of the reflex_stack_ is now convex relative to `cur`, move
      // it to the convex list.
      //
      // Note that since this is the bottom chain, counter-clockwise means convex.
      if (reflex_stack_.size() > 1 &&
          bottom_prev_->Get2DTwistDir(drop_dimension, *(reflex_stack_.end() - 2), cur) > 0) {
        // Add bottom_prev_ to the convex list.
        convex_bottom_.push_back(*bottom_prev_);
        // Remove it from the reflex stack.
        reflex_stack_.pop_back();
        bottom_prev_ = &convex_bottom_.back();
      }
    }
    // At this point (bottom_prev_, cur, next) is convex.
    assert(cur.Get2DTwistDir(drop_dimension, *bottom_prev_, next) < 0);
    convex_bottom_.push_back(cur);
  }

  // The maximum vertex for the emitted polygon must be in either
  // `convex_top_` or `convex_bottom_` before calling this.
  //
  // `last_prev_side` is the last vertex processed from the previous monotone
  // chain.
  void EmitConvex(const Emitter& emit, const Vertex3Rep& last_prev_side) {
    assert(convex_top_.size() + convex_bottom_.size() > 0);
    emit(reflex_stack_.back(), convex_bottom_.begin(), convex_bottom_.end(),
          convex_top_.rbegin(), convex_top_.rend());
    assert(convex_top_.empty() || convex_bottom_.empty() ||
           reflex_stack_.size() == 1);
    reflex_stack_[0] = last_prev_side;
    convex_top_.clear();
    convex_bottom_.clear();
  }

  // Clears all but the first vertex from `reflex_stack_` in preparation for
  // processing vertices from the opposite side. The current side can be
  // resumed again if one of its vertices is next in the merged list of sorted
  // vertices.
  //
  // Passing true for `top` indicates that the top chain should be closed.
  // Otherwise the bottom chain is closed. This is necessary to determine how
  // to emit vertices from `reflex_stack_` in counter-clockwise order.
  //
  // `last` is the last vertex processed from the closing side, and `next` is
  // the next vertex on the next side.
  void CloseSide(const Emitter& emit, bool top, const Vertex3Rep& last,
                 const Vertex3Rep& next) {
    if (reflex_stack_.size() == 1)
      return;

    if (!convex_top_.empty() || !convex_bottom_.empty()) {
      // It doesn't matter whether next is added to convex_bottom_ or
      // convex_top_, but it can only be added to one.
      convex_top_.push_back(next);
      EmitConvex(emit, last);
    }

    while (reflex_stack_.size() > 1) {
      if (top) {
        // emit(next, r[-1], r[-2]);
        emit(next, reflex_stack_.end(), reflex_stack_.end(),
             reflex_stack_.rbegin(), reflex_stack_.rbegin() + 2);
      } else {
        // emit(next, r[-2], r[-1]);
        emit(next, reflex_stack_.end() - 2, reflex_stack_.end(),
             reflex_stack_.rend(), reflex_stack_.rend());
      }
      reflex_stack_.pop_back();
    }
    reflex_stack_[0] = last;
  }

  // The top of the convex polygon currently being constructed. The vertices
  // are sorted in the monotone dimension. This does not contain the minimum
  // vertex for the polygon being constructed.
  //
  // By the time the convex polygon is emitted, either `convex_top` or
  // `convex_bottom` will contain the maximum vertex for the convex polygon,
  // but not both.
  std::vector<Vertex3Rep> convex_top_;

  // The bottom of the convex polygon currently being constructed. The vertices
  // are sorted in the monotone dimension. This does not contain the minimum
  // vertex for the polygon being constructed.
  //
  // By the time the convex polygon is emitted, either `convex_top` or
  // `convex_bottom` will contain the maximum vertex for the convex polygon,
  // but not both.
  std::vector<Vertex3Rep> convex_bottom_;

  // This is a stack of reflex vertices that must be eventually resolved.
  //
  // reflex_stack[0] is an exception:
  // * It always exists (the length of reflex_stack is always at least 1 after
  //   initialization)
  // * It is the minimum vertex that needs to be emitted for any remaining
  //   convex polygons.
  // * It is the position of the opposite side.
  std::vector<Vertex3Rep> reflex_stack_;

  const Vertex3Rep* top_prev_;
  const Vertex3Rep* bottom_prev_;
};

}  // walnut

#endif // WALNUT_MONOTONE_DECOMPOSER_H__
