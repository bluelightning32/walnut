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
  using Vertex4Rep = Vertex4<(vertex3_bits_template - 1)*7 + 10,
                             (vertex3_bits_template - 1)*6 + 10>;
  using PlaneRep =
    typename PlaneFromVertex3Builder<vertex3_bits_template>::PlaneRep;

  class VertexIterator : public std::vector<Vertex4Rep>::const_iterator {
   public:
    VertexIterator(typename std::vector<Vertex4Rep>::const_iterator pos) :
      std::vector<Vertex4Rep>::const_iterator(pos) { }
  };

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

  ConvexPolygon() : plane_(PlaneRep::Zero()), drop_dimension_(-1) { }

  template <int other_vertex3_bits>
  ConvexPolygon(const ConvexPolygon<other_vertex3_bits>& other) :
    plane_(other.plane_), drop_dimension_(other.drop_dimension_),
    vertices_(other.vertices_) { }

  size_t vertex_count() const {
    return vertices_.size();
  }

  const Vertex4Rep& vertex(size_t index) const {
    return vertices_[index];
  }

  VertexIterator vertices_begin() const {
    return VertexIterator(vertices_.begin());
  }

  VertexIterator vertices_end() const {
    return VertexIterator(vertices_.end());
  }

  const PlaneRep& plane() const { return plane_; }

  // When this dimension is projected to 0, 'dropped', the vertices will not
  // become collinear (assuming they were not already collinear).
  int drop_dimension() const { return drop_dimension_; }

  // Gets the index of the lexicographically smallest vertex.
  size_t GetMinimumIndex() const {
    size_t min = 0;
    for (size_t i = 1; i < vertices_.size(); ++i) {
      if (Vertex4Rep::LexicographicallyLt(vertices_[i], vertices_[min])) {
        min = i;
      }
    }
    return min;
  }

  // Sorts `vertices_`, such that the lexicographically minimum vertex comes
  // first.
  //
  // Sorting the vertices does not affect the shape of the polygon.
  void SortVertices() {
    std::rotate(vertices_.begin(), vertices_.begin() + GetMinimumIndex(),
                vertices_.end());
  }

  // Returns true of the other polygon is the same as this.
  //
  // Two polygons are considered equal if:
  // 1. their planes are the same
  //    - different scales are okay.
  // 2. they have the same vertices in the same order
  //    - the vertices are in a cycle. It's okay if the polygon cycles start at
  //      different indices.
  //    - it's okay of the homogenous vertices have a different scale.
  template <int other_vertex3_bits>
  bool operator==(const ConvexPolygon<other_vertex3_bits>& other) const;

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

template <int vertex3_bits>
template <int other_vertex3_bits>
bool ConvexPolygon<vertex3_bits>::operator==(
    const ConvexPolygon<other_vertex3_bits>& other) const {
  if (plane_ != other.plane()) {
    return false;
  }
  if (vertex_count() != other.vertex_count()) {
    return false;
  }
  size_t match_offset = 0;
  while (true) {
    if (match_offset == vertex_count()) {
      return false;
    }
    if (other.vertex(match_offset) == vertex(0)) {
      break;
    }
  }
  for (size_t i = 1, j = match_offset + 1; i < vertices_.size(); ++i, ++j) {
    if (vertex(i) != other.vertex(j)) {
      return false;
    }
  }
  return true;
}

}  // walnut

#endif // WALNUT_CONVEX_POLYGON_H__
