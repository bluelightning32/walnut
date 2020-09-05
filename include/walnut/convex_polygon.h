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

 private:
  template <int other_vertex3_bits>
  ConvexPolygon(const std::vector<Vector3<other_vertex3_bits>>& points) :
    points_(points.begin(), points.end()) { }

  ConvexPolygon(std::vector<Vertex4Rep> points) : points_(std::move(points)) { }

  std::vector<Vertex4Rep> points_;
};

}  // walnut

#endif // WALNUT_CONVEX_POLYGON_H__
