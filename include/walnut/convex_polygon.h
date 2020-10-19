#ifndef WALNUT_CONVEX_POLYGON_H__
#define WALNUT_CONVEX_POLYGON_H__

#include <vector>

#include "walnut/plane.h"
#include "walnut/plucker_line.h"
#include "walnut/point3.h"
#include "walnut/point4.h"

namespace walnut {

// A 2D ConvexPolygon inside of R^3. The vertices are stored using homogeneous coordinates.
template <int point3_bits_template = 32>
class ConvexPolygon {
 public:
  // Defined in convex_polygon_factory.h
  class Factory;
  using Point3Rep = Point3<point3_bits_template>;
  using Point4Rep = Point4<(point3_bits_template - 1)*7 + 10,
                             (point3_bits_template - 1)*6 + 10>;
  using PlaneRep =
    typename PlaneFromPoint3Builder<point3_bits_template>::PlaneRep;
  using LineRep = typename PluckerLineFromPlanesFromPoint3sBuilder<
    point3_bits_template>::PluckerLineRep;

  struct VertexInfo {
    template <int other_point3_bits>
    VertexInfo(const Point3<other_point3_bits>& vertex,
               const Point3<other_point3_bits>& next_vertex) :
      vertex(vertex), edge(vertex, next_vertex) { }

    static bool LexicographicallyLt(const VertexInfo& a, const VertexInfo& b) {
      return Point4Rep::LexicographicallyLt(a.vertex, b.vertex);
    }

    Point4Rep vertex;
    // This is the line for the edge that starts at vertex and ends at
    // the next vertex in the cycle.
    //
    // Notably:
    //   next_vertex == vertex + edge.d()
    LineRep edge;
  };

  // The minimum number of bits to support for each coordinate of the vertex3's
  // that the polygon is built from.
  static constexpr int point3_bits = point3_bits_template;

  // The minimum number of bits to support for each of the x, y, and z
  // coordinates for each vertex, after an arbitrary number of splits from
  // planes of the type PlaneRep.
  static constexpr int point4_num_bits = Point4Rep::num_bits;
  // The minimum number of bits to support the w coordinate for each vertex,
  // after an arbitrary number of splits from planes of the type PlaneRep.
  static constexpr int point4_denom_bits = Point4Rep::denom_bits_template;

  ConvexPolygon() : plane_(PlaneRep::Zero()), drop_dimension_(-1) { }

  template <int other_point3_bits>
  ConvexPolygon(const ConvexPolygon<other_point3_bits>& other) :
    plane_(other.plane_), drop_dimension_(other.drop_dimension_),
    vertices_(other.vertices_) { }

  size_t vertex_count() const {
    return vertices_.size();
  }

  const Point4Rep& vertex(size_t index) const {
    return vertices_[index].vertex;
  }

  const LineRep& edge(size_t index) const {
    return vertices_[index].edge;
  }

  const std::vector<VertexInfo>& vertices() const {
    return vertices_;
  }

  const PlaneRep& plane() const { return plane_; }

  // When this dimension is projected to 0, 'dropped', the vertices will not
  // become collinear (assuming they were not already collinear).
  int drop_dimension() const { return drop_dimension_; }

  // Gets the index of the lexicographically smallest vertex.
  size_t GetMinimumIndex() const {
    size_t min = 0;
    for (size_t i = 1; i < vertices_.size(); ++i) {
      if (Point4Rep::LexicographicallyLt(vertex(i), vertex(min))) {
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
  template <int other_point3_bits>
  bool operator==(const ConvexPolygon<other_point3_bits>& other) const;

 private:
  template <int other_point3_bits>
  ConvexPolygon(const PlaneRep& plane, int drop_dimension,
                const std::vector<Vector3<other_point3_bits>>& vertices) :
    plane_(plane), drop_dimension_(drop_dimension),
    vertices_(vertices.begin(), vertices.end()) { }

  ConvexPolygon(const PlaneRep& plane, int drop_dimension,
                std::vector<VertexInfo> vertices) :
    plane_(plane), drop_dimension_(drop_dimension),
    vertices_(std::move(vertices)) { }

  // The plane that all of the vertices are in.
  PlaneRep plane_;
  // When this dimension is projected to 0, 'dropped', the vertices will not
  // become collinear (assuming they were not already collinear).
  int drop_dimension_;
  std::vector<VertexInfo> vertices_;
};

template <int point3_bits>
template <int other_point3_bits>
bool ConvexPolygon<point3_bits>::operator==(
    const ConvexPolygon<other_point3_bits>& other) const {
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
