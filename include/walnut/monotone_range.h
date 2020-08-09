#ifndef WALNUT_MONOTONE_RANGE_H__
#define WALNUT_MONOTONE_RANGE_H__

#include <iterator>

#include "walnut/vertex3.h"

namespace walnut {

// A 2D ConvexPolygon inside of R^3. The vertices are stored using homogeneous coordinates.
template <typename Vertex3Iterator>
class MonotoneRange {
 public:
  using Vertex3Rep = typename std::iterator_traits<Vertex3Iterator>::value_type;

  // Find the next monotone polygon from an iterator range of `Vertex3Rep`s in
  // the same plane.
  //
  // Abstractly, given an input list of vertices, all in the same plane, this
  // function finds the first vertex that is not monotone. Although iterators
  // are used instead of lists, and the function also takes the end of the
  // input list into account.
  //
  // Note that if the input polygon is not already monotone, this function may
  // return a self intersecting (but monotone) polygon. This is a tradeoff for
  // never adding new vertices to the input.
  //
  // monotone_dimension: the dimension that the output polygon should be
  //    monotone in. This is in the range [0, 3).
  // p1: a vertex to consider prepended to the input vertices in
  //    `next_polygon_start`.
  // next_polygon_start: on input, this points to the list of vertices to take
  //    from for the next monotone polygon. On output, this is the first point
  //    to consider in the next monotone polygon (assuming p1 is passed again).
  // vertex_end: this defines the end of the input list of vertices to choose
  //    from.
  // plane: this is the plane of the polygon.
  //
  // Returns an iterator to the end vertex (one after) of the monotone polygon.
  //
  static Vertex3Iterator GetNextMonotone(int monotone_dimension,
                                         const Vertex3Rep& p1,
                                         Vertex3Iterator& next_polygon_start,
                                         Vertex3Iterator vertex_end);
};

template <typename Vertex3Iterator>
inline Vertex3Iterator
MonotoneRange<Vertex3Iterator>::GetNextMonotone(int monotone_dimension,
                                                const Vertex3Rep& p1,
                                                Vertex3Iterator& next_polygon_start,
                                                Vertex3Iterator vertex_end) {
  if (next_polygon_start == vertex_end) {
    return next_polygon_start;
  }
  // Find the first vertex that has a different value in the monotone dimension
  // than p1. This is necessary to find out if the vertices are initially
  // increasing or decreasing in the monotone dimension.
  while (next_polygon_start->coords()[monotone_dimension] ==
         p1.coords()[monotone_dimension]) {
    ++next_polygon_start;
    if (next_polygon_start == vertex_end) {
      return next_polygon_start;
    }
  }
  // If the vertices are initially increasing in the monotone dimension, set
  // this to 1, otherwise set it to -1.
  Vertex3Iterator polygon_end = next_polygon_start;
  int dir = polygon_end->coords()[monotone_dimension] >
            p1.coords()[monotone_dimension] ? 1 : -1;
  // Keep accepting vertices while they continue to increase/decrease
  // (depending on dir) relative to the previous vertex.
  while ((polygon_end->coords()[monotone_dimension] -
          next_polygon_start->coords()[monotone_dimension]).GetSign()
         * dir >= 0) {
    next_polygon_start = polygon_end;
    ++polygon_end;
    if (polygon_end == vertex_end) {
      next_polygon_start = polygon_end;
      return polygon_end;
    }
  }
  // Now accept vertices while they continue to go back towards p1. This may
  // overshoot p1 in that direction.
  dir = -dir;
  while ((polygon_end->coords()[monotone_dimension] -
          next_polygon_start->coords()[monotone_dimension]).GetSign()
         * dir >= 0) {
    next_polygon_start = polygon_end;
    ++polygon_end;
    if (polygon_end == vertex_end) {
      next_polygon_start = polygon_end;
      return polygon_end;
    }
  }
  // In case p1 was overshot, accept vertices that go back to p1, but don't let
  // p1 get overshot again.
  dir = -dir;
  while ((p1.coords()[monotone_dimension] -
          polygon_end->coords()[monotone_dimension]).GetSign()
         * dir >= 0 &&
         (polygon_end->coords()[monotone_dimension] -
          next_polygon_start->coords()[monotone_dimension]).GetSign()
         * dir >= 0) {
    next_polygon_start = polygon_end;
    ++polygon_end;
    if (polygon_end == vertex_end) {
      next_polygon_start = polygon_end;
      return polygon_end;
    }
  }

  return polygon_end;
}

}  // walnut

#endif // WALNUT_MONOTONE_RANGE_H__
