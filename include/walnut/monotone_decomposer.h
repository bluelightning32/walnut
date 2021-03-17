#ifndef WALNUT_MONOTONE_DECOMPOSER_H__
#define WALNUT_MONOTONE_DECOMPOSER_H__

// MonotoneDecomposer takes the top and bottom chain from a monotone polygon
// and converts them into one or more convex polygons. The MonotoneDecomposer
// tries to minimize the number of convex polygons it outputs, instead of
// always outputting triangles.
//
// This class is designed to output counter clockwise polygons as viewed with
// the  drop dimension set to 0.  Although for self-intersecting polygons, it
// may also output clockwise convex polygons.  For non-self-intersecting
// polygons, the top chain edges should be above the bottom chain edges in the
// compare_dimension. If this can't be guaranteed, OrientingMonotoneDecomposer
// should be used instead.
//
// The MonotoneDecomposer takes 3D vertices. The 3 dimensions can be
// configured as:
// * drop_dimension - this dimension is removed from the 3D vertices to make 2D
//     vertices.
// * monotone_dimension - each vertex of the top and bottom chains must be
//     increasing in this dimension.
// * compare_dimension - for non-self-intersecting input polygons, all of the
//     edges from the top chain should be above the edges from the bottom chain
//     in this dimension. For self-intersecting input polygons, the top and
//     bottom chains may be flipped in this dimension, but the decomposer will
//     return worse results for the portions that are flipped. They will be
//     worse in the sense that more overlapping polygons will be outputted than
//     strictly necessary.

#include <functional>
#include <vector>

#include "walnut/monotone_triangulator.h"

namespace walnut {

template <typename Point3RepTemplate = Point3<32>>
class MonotoneDecomposer : public MonotoneTriangulator<Point3RepTemplate> {
 public:
  using Parent = MonotoneTriangulator<Point3RepTemplate>;
  using Point3Rep = typename Parent::Point3Rep;
  using const_iterator = typename std::vector<Point3Rep>::const_iterator;
  using const_reverse_iterator = typename
    std::vector<Point3Rep>::const_reverse_iterator;

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
  // Emit is called for each convex polygon produced. Emit must not
  // re-entrantly call back into the same MonotoneDecomposer.
  //
  // Runtime: O(n)
  template <typename TopIterator, typename BottomIterator>
  void Build(int drop_dimension, int monotone_dimension,
             TopIterator top_begin, TopIterator top_end,
             BottomIterator bottom_begin, BottomIterator bottom_end) {
    assert(convex_top_.empty());
    assert(convex_bottom_.empty());
    orientation_ = 0;

    drop_dimension_ = drop_dimension;
    Parent::Build(drop_dimension, monotone_dimension,
                  top_begin, top_end, bottom_begin, bottom_end);
    EmitRange(orientation_, convex_top_.rbegin(), convex_top_.rend(),
              convex_bottom_.begin(), convex_bottom_.end());
    convex_top_.clear();
    convex_bottom_.clear();
  }

 protected:
  // The top of the convex polygon currently being constructed. The vertices
  // are sorted in the monotone dimension.
  std::vector<Point3Rep> convex_top_;

  // The bottom of the convex polygon currently being constructed. The vertices
  // are sorted in the monotone dimension.
  std::vector<Point3Rep> convex_bottom_;

  // Called to emit a convex polygon. The first argument represents the
  // orientation of the polygon. It is set to -1 if the polygon is
  // counter-clockwise, 1 if it is clockwise, and 0 if it is collinear.
  //
  // The next 4 arguments represent 2 iterator ranges of vertices. To get the
  // actual list of vertices for the polygon, calculate range1 + range2.
  virtual void EmitRange(int orientation, const_reverse_iterator range1_begin,
                         const_reverse_iterator range1_end,
                         const_iterator range2_begin,
                         const_iterator range2_end) = 0;

  void EmitTriangle(bool p3_is_top_chain, const Point3Rep& p1,
                    const Point3Rep& p2, const Point3Rep& p3) override {
    if (!convex_top_.empty()) {
      assert(!convex_bottom_.empty());
      if (convex_top_.back() == p1 && convex_bottom_.back() == p2) {
        bool set_orientation = false;
        if (orientation_ == 0) {
          set_orientation = true;
          orientation_ = p2.Get2DTwistDirReduced(drop_dimension_, p1, p3);
        }
        // For non-self-intersecting polygons, orientation_ will always be 0 or
        // -1. However for self-intersecting polygons, the triangulator can
        // produce flipped triangles, in which case orientation_ will be 1.
        const Point3Rep& prev_top = convex_top_.size() > 1 ?
          convex_top_.end()[-2] : convex_bottom_.front();
        const Point3Rep& prev_bottom = convex_bottom_.size() > 1 ?
          convex_bottom_.end()[-2] : convex_top_.front();
        if (p1.Get2DTwistDir(drop_dimension_, prev_top, p3) *
              orientation_ <= 0 &&
            p2.Get2DTwistDir(drop_dimension_, prev_bottom, p3) *
              orientation_ >= 0) {
          if (p3_is_top_chain) {
            if (p1 != p3) {
              convex_top_.push_back(p3);
            }
          } else {
            if (p2 != p3) {
              convex_bottom_.push_back(p3);
            }
          }
          return;
        }
        if (set_orientation) {
          // If the triangle isn't merged, restore the old orientation.
          orientation_ = 0;
        }
      }
      EmitRange(orientation_, convex_top_.rbegin(), convex_top_.rend(),
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
    orientation_ = p2.Get2DTwistDirReduced(drop_dimension_, p1, p3);
  }

 private:
  // The orientation of the polygon being built. -1 for counter-clockwise, 1
  // for clockwise, or 0 for collinear.
  int orientation_;

  int drop_dimension_;
};

}  // walnut

#endif // WALNUT_MONOTONE_DECOMPOSER_H__
