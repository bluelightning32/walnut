#ifndef WALNUT_ORIENTING_MONOTONE_DECOMPOSER_H__
#define WALNUT_ORIENTING_MONOTONE_DECOMPOSER_H__

// OrientingMonotoneDecomposer takes two chains. If the first chain is a bottom
// chain (has edges less than the second chain's edges in compare_dimension),
// then OrientingMonotoneDecomposer outputs counter-clockwise convex polygons.
// If the first chain is the top chain, then OrientingMonotoneDecomposer
// outputs clockwise convex polygons. If the two chains cross (which happens
// for self-intersecting monotone polygons), OrientingMonotoneDecomposer may
// output clockwise and counter-clockwise convex polygons, and those polygons
// may overlap.
//
// OrientingMonotoneDecomposer does this by autodetecting which chains are top
// and bottom. It uses the angle between the minimum vertex and the first
// vertex on each chain to determine which is the top and bottom. If those
// vertices are collinear, it continues down the chains, to find some that are
// not collinear. If all vertices are collinear, no polygon is outputted.

#include "walnut/monotone_decomposer.h"

namespace walnut {

template <typename Point3RepTemplate = Point3>
class OrientingMonotoneDecomposer :
  public MonotoneDecomposer<Point3RepTemplate> {
 public:
  using Parent = MonotoneDecomposer<Point3RepTemplate>;
  using Point3Rep = typename Parent::Point3Rep;
  using const_iterator = typename std::vector<Point3Rep>::const_iterator;
  using const_reverse_iterator = typename
    std::vector<Point3Rep>::const_reverse_iterator;

  // Given a monotone polygon in the form of two iterator ranges, representing
  // the top and bottom chains (in either order), this converts the monotone
  // polygon into convex polygons.
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
  // must be included in both chains. The maximum vertex (vertex greatest in
  // the monotone dimension) must also be included in both chains.  chain.
  // Violating either of these rules results in undefined behavior.
  //
  // `emit` is called for each convex polygon produced. `emit` must not
  // re-entrantly call back into the same OrientingMonotoneDecomposer.
  //
  // Runtime: O(n)
  template <typename Chain1Iterator, typename Chain2Iterator>
  void Build(int drop_dimension, int monotone_dimension,
             Chain1Iterator chain1_begin, Chain1Iterator chain1_end,
             Chain2Iterator chain2_begin, Chain2Iterator chain2_end) {
    if (!DetectOrientation(drop_dimension, monotone_dimension,
                           chain1_begin, chain1_end,
                           chain2_begin, chain2_end)) {
      // All of the vertices are collinear.
      Chain1Iterator chain1_before_end = chain1_end;
      --chain1_before_end;
      Chain2Iterator chain2_after_begin = chain2_begin;
      ++chain2_after_begin;

      convex_top_.insert(convex_top_.end(), chain1_begin, chain1_before_end);
      convex_bottom_.insert(convex_bottom_.end(), chain2_after_begin, chain2_end);
      EmitOriented(/*orientation=*/0, convex_top_.rbegin(),
                   convex_top_.rend(), convex_bottom_.begin(),
                   convex_bottom_.end());
      convex_top_.clear();
      convex_bottom_.clear();
      return;
    }
    
    if (flipped_) {
      Chain1Iterator chain1_before_end = chain1_end;
      --chain1_before_end;
      Chain2Iterator chain2_after_begin = chain2_begin;
      ++chain2_after_begin;

      // Skip the last vertex chain1, because it is the maximum vertex (which
      // is also included in chain2). Skip the first vertex of chain2, because
      // it is the minimum vertex (which is also included in chain1).
      Parent::Build(drop_dimension, monotone_dimension,
                    /*top_begin=*/chain1_begin, /*top_end=*/chain1_before_end,
                    /*bottom_begin=*/chain2_after_begin,
                    /*bottom_end=*/chain2_end);
    } else {
      Chain2Iterator chain2_before_end = chain2_end;
      --chain2_before_end;
      Chain1Iterator chain1_after_begin = chain1_begin;
      ++chain1_after_begin;

      // Skip the last vertex chain2, because it is the maximum vertex (which
      // is also included in chain1). Skip the first vertex of chain1, because
      // it is the minimum vertex (which is also included in chain2).
      Parent::Build(drop_dimension, monotone_dimension,
                    /*top_begin=*/chain2_begin, /*top_end=*/chain2_before_end,
                    /*bottom_begin=*/chain1_after_begin,
                    /*bottom_end=*/chain1_end);
    }
  }
 
 protected:
  using Parent::convex_top_;
  using Parent::convex_bottom_;

  // Called to emit a convex polygon. The orientation argument represents the
  // orientation of the polygon. It is set to -1 if the polygon is
  // counter-clockwise, 1 if it is clockwise, and 0 if it is collinear.
  //
  // The next 4 arguments represent 2 iterator ranges of vertices. To get the
  // actual list of vertices for the polygon, calculate range1 + range2.
  virtual void EmitOriented(int orientation,
                            const_reverse_iterator range1_begin,
                            const_reverse_iterator range1_end,
                            const_iterator range2_begin,
                            const_iterator range2_end) = 0;

 private:
  // Returns false if all vertices from both chains are collinear, or true
  // otherwise and sets `flipped_`. `flipped_` is set to false if chain1 is the
  // bottom chain, or true if chain1 is the top chain.
  template <typename Chain1Iterator, typename Chain2Iterator>
  bool DetectOrientation(int drop_dimension, int monotone_dimension,
                         Chain1Iterator chain1_begin,
                         Chain1Iterator chain1_end,
                         Chain2Iterator chain2_begin,
                         Chain2Iterator chain2_end);

  void EmitRange(int orientation, const_reverse_iterator range1_begin,
                 const_reverse_iterator range1_end,
                 const_iterator range2_begin,
                 const_iterator range2_end) final {
    if (flipped_) {
      EmitOriented(-orientation,
                   const_reverse_iterator(range2_end),
                   const_reverse_iterator(range2_begin),
                   range1_end.base(), range1_begin.base());
    } else {
      EmitOriented(orientation,
                   range1_begin, range1_end, range2_begin, range2_end);
    }
  }

  // orientation of the input polygon. It is set to false if chain1 is the
  // bottom chain and chain2 is the top chain, or true if chain1 is the top
  // chain and chain2 is the bottom chain.
  bool flipped_;
};

template <typename Point3RepTemplate>
template <typename Chain1Iterator, typename Chain2Iterator>
bool OrientingMonotoneDecomposer<Point3RepTemplate>::DetectOrientation(
    int drop_dimension, int monotone_dimension, Chain1Iterator chain1_begin,
    Chain1Iterator chain1_end, Chain2Iterator chain2_begin,
    Chain2Iterator chain2_end) {
  const Point3Rep& minimum_vertex = *chain1_begin;
  // Both chains must include the minimum vertex.
  assert(*chain2_begin == minimum_vertex);
  Chain1Iterator chain1_pos = chain1_begin;
  ++chain1_pos;
  Chain2Iterator chain2_pos = chain2_begin;
  ++chain2_pos;

  const int twist = minimum_vertex.Get2DTwistDir(drop_dimension,
                                                 *chain1_pos, *chain2_pos);
  if (twist != 0) {
    // The first vertices from each chain are not collinear.
    flipped_ = twist < 0;
    return true;
  }
  // (minimum_vertex, collinear_vertex) define the line that chain1_pos and
  // chain2_pos are collinear with. Either chain1_pos or chain2_pos could be
  // used here.
  const Point3Rep& collinear_vertex = *chain1_pos;
  // Loop through vertices of both chains sorted by the monotone_dimension.
  // Keep looping until a non-collinear vertex is found, or all of the
  // vertices in both chains are consumed.
  while (true) {
    // Entries are preferentially taken from chain1, and both chains contain
    // the max element. So as long as chain1 is not completely consumed,
    // chain2 still contains at least the max element.
    if (chain1_pos == chain1_end) {
      // Chain1 was completely consumed, which means only the maximum vertex
      // remains in chain2. All of the vertices were collinear.
#ifndef NDEBUG
      ++chain2_pos;
      assert(chain2_pos == chain2_end);
#endif
      return false;
    }
    assert(chain2_pos != chain2_end);
    const bool chain1_is_current =
      chain1_pos->CompareComponent(monotone_dimension, *chain2_pos) <= 0;
    const Point3Rep& current = chain1_is_current ? *chain1_pos : *chain2_pos;
    const int twist = minimum_vertex.Get2DTwistDir(drop_dimension,
                                                   collinear_vertex, current);
    if (twist != 0) {
      // A non-collinear vertex has been found. The chain it was part of
      // (indicated by chain1_is_current) can be identified, and the other
      // chain will be the opposite.
      const bool current_chain_is_top = twist > 0;
      flipped_ = (chain1_is_current == current_chain_is_top);
      return true;
    }
    if (chain1_is_current) {
      ++chain1_pos;
    } else {
      ++chain2_pos;
    }
  }
}

}  // walnut

#endif // WALNUT_ORIENTING_MONOTONE_DECOMPOSER_H__
