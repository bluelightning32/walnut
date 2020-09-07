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
  // Defined in convex_polygon_factory.h
  class Factory;
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

  template <int other_vertex3_bits>
  ConvexPolygon(const ConvexPolygon<other_vertex3_bits>& other) :
    plane_(other.plane_), drop_dimension_(other.drop_dimension_),
    vertices_(other.vertices_) { }

  const std::vector<Vertex4Rep>& vertices() const {
    return vertices_;
  }

  const PlaneRep& plane() const { return plane_; }

  // When this dimension is projected to 0, 'dropped', the vertices will not
  // become collinear (assuming they were not already collinear).
  int drop_dimension() const { return drop_dimension_; }

  // Sorts `vertices_`, such that the lexicographically minimum vertex comes
  // first.
  //
  // Sorting the vertices does not affect the shape of the polygon.
  void SortVertices() {
    typename std::vector<Vertex4Rep>::iterator min = vertices_.begin();
    for (auto it = vertices_.begin() + 1; it != vertices_.end(); ++it) {
      if (Vertex4Rep::LexicographicallyLt(*it, *min)) {
        min = it;
      }
    }
    std::rotate(vertices_.begin(), min, vertices_.end());
  }

 private:
  template <int other_vertex3_bits>
  ConvexPolygon(const PlaneRep& plane, int drop_dimension,
                const std::vector<Vector3<other_vertex3_bits>>& vertices) :
    plane_(plane), drop_dimension_(drop_dimension),
    vertices_(vertices.begin(), vertices.end()) { }

  ConvexPolygon(const PlaneRep& plane, int drop_dimension,
                std::vector<Vertex4Rep> vertices) :
    plane_(plane), drop_dimension_(drop_dimension),
    vertices_(std::move(vertices)) { }

  // The plane that all of the vertices are in.
  PlaneRep plane_;
  // When this dimension is projected to 0, 'dropped', the vertices will not
  // become collinear (assuming they were not already collinear).
  int drop_dimension_;
  std::vector<Vertex4Rep> vertices_;
};

}  // walnut

#endif // WALNUT_CONVEX_POLYGON_H__
