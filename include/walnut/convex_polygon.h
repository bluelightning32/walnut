#ifndef WALNUT_CONVEX_POLYGON_H__
#define WALNUT_CONVEX_POLYGON_H__

#include <vector>

#include "walnut/plane.h"
#include "walnut/vertex3.h"
#include "walnut/vertex4.h"

namespace walnut {

// A 2D ConvexPolygon inside of R^3. The vertices are stored using homogeneous coordinates.
template <int vertex3_bits_template = 32>
class ConvexPolygon {
 public:

  using Vertex3Rep = Vertex3<vertex3_bits_template>;

  using Vertex4Rep = Vertex4<(vertex3_bits_template - 1)*7 + 15,
                            (vertex3_bits_template - 1)*6 + 13>;

  using PlaneRep = Plane<(vertex3_bits_template - 1)*2 + 4,
                         (vertex3_bits_template - 1)*3 + 6>;

  // The minimum number of bits to support for each coordinate of the vertex3's
  // that the polygon is built from.
  static constexpr int vertex3_bits = vertex3_bits_template;

  // The minimum number of bits to support for each of the x, y, and z
  // coordinates for each vertex, after an arbitrary number of splits from
  // planes of the type PlaneRep.
  static constexpr int vertex4_num_bits = Vertex4Rep::num_bits;
  // The minimum number of bits to support the w coordinate for each vertex,
  // after an arbitrary number of splits from planes of the type PlaneRep.
  static constexpr int vertex4_denom_bits = Vertex4Rep::denom_bits_template;

  // Leaves the polygon in an undefined state.
  ConvexPolygon() = default;

  template <int other_vertex3_bits>
  ConvexPolygon(const ConvexPolygon<other_vertex3_bits>& other) :
    ConvexPolygon(other.points_) { }

  const Vertex4Rep& GetPoint(int i) const {
    return points_[i];
  }

  int point_count() const { return points_.size(); }

  // Find the next planar polygon from an iterator range of Vertex3Rep.
  //
  // Abstractly, given an input list of vertices, this finds the first vertex
  // that is not coplanar. Although iterators are used instead of lists, and
  // the function also takes the end of the input list into account.
  //
  // Note that if the input vertices are not already planar, this function may
  // return a self intersecting (but planar) polygon. This is a tradeoff for
  // never adding new vertices to the input.
  //
  // p1: a vertex to consider prepended to the input vertices in
  //    `next_polygon_start`.
  // next_polygon_start: on input, this points to the list of vertices to take
  //    from for the next planar polygon. On output, this is the first point to
  //    consider in the next planar polygon (assuming p1 is passed again).
  // vertex_end: this defines the end of the input list of vertices to choose
  //    from.
  // plane: this is the plane of the polygon.
  //
  // Returns an iterator to the end vertex (one after) of the planar polygon.
  // If there was no planar polygon in the input list (because the input list
  // was too short or all vertices were collinear), then a copy of the input
  // value of `next_polygon_start` is returned, and on output
  // `next_polygon_start` points to `vertex_end`.
  template <typename Vertex3Iterator>
  static Vertex3Iterator GetNextPlanar(const Vertex3<vertex3_bits>& p1,
                                       Vertex3Iterator& next_polygon_start,
                                       Vertex3Iterator vertex_end,
                                       PlaneRep &plane);

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
  template <typename Vertex3Iterator>
  static Vertex3Iterator GetNextMonotone(int monotone_dimension,
                                         const Vertex3<vertex3_bits>& p1,
                                         Vertex3Iterator& next_polygon_start,
                                         Vertex3Iterator vertex_end);

 private:
  std::vector<Vertex4Rep> points_;

  ConvexPolygon(std::vector<Vertex4Rep> points) : points_(std::move(points)) { }
};

template <int vertex3_bits>
template <typename Vertex3Iterator>
inline Vertex3Iterator
ConvexPolygon<vertex3_bits>::GetNextPlanar(const Vertex3<vertex3_bits>& p1,
                                           Vertex3Iterator& next_polygon_start,
                                           Vertex3Iterator vertex_end,
                                           PlaneRep &plane) {
  if (next_polygon_start == vertex_end) {
    return next_polygon_start;
  }
  Vertex3Iterator polygon_end = next_polygon_start;
  const Vertex3Rep& p2 = *next_polygon_start;

  // Keep p1 and p2 pointing to where they are. Use the next soonest point for
  // p3 such that p1, p2, and p3 are not coincident.
  //
  // After the loop ends, `plane` holds the plane formed from p1, p2, and p3.
  do {
    ++next_polygon_start;
    if (next_polygon_start == vertex_end) {
      return polygon_end;
    }
    const Vertex3Rep& p3 = *next_polygon_start;
    plane = PlaneRep(p1, p2, p3);
  } while (!plane.IsValid());

  polygon_end = next_polygon_start;

  // Keep picking more points as long as they are in the same plane and the
  // input list does not run out.
  while (true) {
    ++polygon_end;
    if (polygon_end == vertex_end) {
      next_polygon_start = polygon_end;
      return polygon_end;
    }
    if (plane.Compare(*polygon_end) != 0) {
      return polygon_end;
    }
    next_polygon_start = polygon_end;
  }
}

template <int vertex3_bits>
template <typename Vertex3Iterator>
inline Vertex3Iterator
ConvexPolygon<vertex3_bits>::GetNextMonotone(int monotone_dimension,
                                             const Vertex3<vertex3_bits>& p1,
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
  // Now accepting vertices while they continue to go back towards p1. This may
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

#endif // WALNUT_CONVEX_POLYGON_H__
