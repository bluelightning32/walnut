#ifndef WALNUT_POLYGON_H__
#define WALNUT_POLYGON_H__

#include <vector>

#include "walnut/plane.h"
#include "walnut/vertex3.h"
#include "walnut/vertex4.h"

namespace walnut {

// A 2D Polygon inside of R^3. The vertices are stored using homogeneous coordinates.
template <int vertex3_bits_template = 32>
class Polygon {
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
  Polygon() = default;

  template <int other_vertex3_bits>
  Polygon(const Polygon<other_vertex3_bits>& other) :
    Polygon(other.points_) { }

  template <typename Container>
  Polygon(const Container& points) :
    Polygon(std::begin(points), std::end(points)) { }

  const Vertex4Rep& GetPoint(int i) const {
    return points_[i];
  }

  int point_count() const { return points_.size(); }

  // Find the next planar polygon from an iterator of Vector3Rep.
  //
  // Abstractly, given an input list of vertices, this finds the first vertex
  // that is not coplanar. Although iterators are used instead of lists, and
  // the function also takes the end of the input list into account.
  //
  // p1: a vertex to consider prepended to the input vertices in
  //    `next_polygon_start`.
  // next_polygon_start: on input, this points to the list of vertices to take
  //    from for the next planar polygon. On output, this is the first point to
  //    consider in the next planar polygon (assuming p1 is passed again).
  // vertex_end: this defines the end of the input list of vertices to choose
  //    from for the next planar polygon.
  // plane: this is the plane of the polygon.
  //
  // Returns an iterator to the end vertex (one after) of the planar polygon.
  // If there was no planar polygon in the input list (because the input list
  // was too short or all vertices were colinear), then a copy of the input
  // value of `next_polygon_start` is returned, and on output
  // `next_polygon_start` points to `vertex_end`.
  template <typename Vertex3Iterator>
  static Vertex3Iterator GetNextPlanarPolygon(const Vertex3<vertex3_bits>& p1,
                                             Vertex3Iterator& next_polygon_start,
                                             Vertex3Iterator vertex_end,
                                             PlaneRep &plane);


 private:
  std::vector<Vertex4Rep> points_;
};

template <int vertex3_bits>
template <typename Vertex3Iterator>
inline Vertex3Iterator
Polygon<vertex3_bits>::GetNextPlanarPolygon(const Vertex3<vertex3_bits>& p1,
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

}  // walnut

#endif // WALNUT_POLYGON_H__
