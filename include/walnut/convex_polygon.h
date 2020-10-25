#ifndef WALNUT_CONVEX_POLYGON_H__
#define WALNUT_CONVEX_POLYGON_H__

#include <vector>
#include <utility>

#include "walnut/half_space3.h"
#include "walnut/homo_point3.h"
#include "walnut/plucker_line.h"
#include "walnut/point3.h"

namespace walnut {

// A 2D ConvexPolygon inside of R^3. The vertices are stored using homogeneous coordinates.
template <int point3_bits_template = 32>
class ConvexPolygon {
 public:
  // Defined in convex_polygon_factory.h
  class Factory;
  using Point3Rep = Point3<point3_bits_template>;
  using HomoPoint3Rep = HomoPoint3<(point3_bits_template - 1)*7 + 10,
                             (point3_bits_template - 1)*6 + 10>;
  using HalfSpace3Rep =
    typename HalfSpace3FromPoint3Builder<point3_bits_template>::HalfSpace3Rep;
  using LineRep = typename PluckerLineFromPlanesFromPoint3sBuilder<
    point3_bits_template>::PluckerLineRep;

  struct VertexInfo {
    template <int other_point3_bits>
    VertexInfo(const Point3<other_point3_bits>& vertex,
               const Point3<other_point3_bits>& next_vertex) :
      vertex(vertex), edge(vertex, next_vertex) { }

    static bool LexicographicallyLt(const VertexInfo& a, const VertexInfo& b) {
      return HomoPoint3Rep::LexicographicallyLt(a.vertex, b.vertex);
    }

    HomoPoint3Rep vertex;
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
  // planes of the type HalfSpace3Rep.
  static constexpr int homo_point3_num_bits = HomoPoint3Rep::num_bits;
  // The minimum number of bits to support the w coordinate for each vertex,
  // after an arbitrary number of splits from planes of the type HalfSpace3Rep.
  static constexpr int homo_point3_denom_bits = HomoPoint3Rep::denom_bits_template;

  ConvexPolygon() : plane_(HalfSpace3Rep::Zero()), drop_dimension_(-1) { }

  template <int other_point3_bits>
  ConvexPolygon(const ConvexPolygon<other_point3_bits>& other) :
    plane_(other.plane_), drop_dimension_(other.drop_dimension_),
    vertices_(other.vertices_) { }

  size_t vertex_count() const {
    return vertices_.size();
  }

  const HomoPoint3Rep& vertex(size_t index) const {
    return vertices_[index].vertex;
  }

  const LineRep& edge(size_t index) const {
    return vertices_[index].edge;
  }

  const std::vector<VertexInfo>& vertices() const {
    return vertices_;
  }

  const HalfSpace3Rep& plane() const { return plane_; }

  // When this dimension is projected to 0, 'dropped', the vertices will not
  // become collinear (assuming they were not already collinear).
  int drop_dimension() const { return drop_dimension_; }

  // Gets the index of the lexicographically smallest vertex.
  size_t GetMinimumIndex() const {
    size_t min = 0;
    for (size_t i = 1; i < vertices_.size(); ++i) {
      if (HomoPoint3Rep::LexicographicallyLt(vertex(i), vertex(min))) {
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

  // Returns the source index of an edge pointing in roughly the same direction
  // as `v` and one pointing in the roughly opposite direction, using a binary
  // search.
  //
  // This function projects all of the vertices to 2D by dropping the
  // `drop_dimension`. That's why the input vector is a `Vector2`.
  // `drop_dimension` must refer to a non-zero component of the plane normal.
  //
  // For the purposes of this function, any edge is considered to point in
  // roughly the same direction as v, perpendicular to v, or roughly the
  // opposite direction as v. These 3 cases are defined as edge dot v being
  // greater than 0, equal to 0, or less than 0.
  //
  // This function returns edge source indices. An edge is defined by a source
  // and target vertex. An edge source index is the index of the source vertex.
  //
  // The bisect part of the function name refers to how this function uses a
  // binary search to find the edges. This algorithm is good for ConvexPolgyons
  // with many vertices, but a regular linear search is faster for
  // ConvexPolygons with fewer vertices (roughly 5 or fewer vertices).
  //
  // The first element in the returned pair is the source index for an edge
  // pointing in roughly the same direction as `v`. The second element in the
  // pair is the edge source index for an edge pointing in roughly the opposite
  // direction.
  template <int vector_bits>
  std::pair<size_t, size_t> GetOppositeEdgeIndicesBisect(
      const Vector2<vector_bits>& v, int drop_dimension);

 private:
  template <int other_point3_bits>
  ConvexPolygon(const HalfSpace3Rep& plane, int drop_dimension,
                const std::vector<Vector3<other_point3_bits>>& vertices) :
    plane_(plane), drop_dimension_(drop_dimension),
    vertices_(vertices.begin(), vertices.end()) { }

  ConvexPolygon(const HalfSpace3Rep& plane, int drop_dimension,
                std::vector<VertexInfo> vertices) :
    plane_(plane), drop_dimension_(drop_dimension),
    vertices_(std::move(vertices)) { }

  // The plane that all of the vertices are in.
  HalfSpace3Rep plane_;
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

template <int point3_bits>
template <int vector_bits>
std::pair<size_t, size_t>
ConvexPolygon<point3_bits>::GetOppositeEdgeIndicesBisect(
    const Vector2<vector_bits>& v, int drop_dimension) {
  int initial_dir_sign = vertices()[0].edge.d().DropDimension(drop_dimension)
                                      .Dot(v).GetSign();
  if (initial_dir_sign == 0) {
    // The 0th edge is perpendicular to `v`. Find the first non-perpendicular
    // edge before and after `v`.
    int before = vertices().size() - 1;
    int before_sign;
    while (true) {
      // A well formed ConvexPolygon with a valid drop_dimension should always
      // find a match.
      assert(before > 0);
      before_sign = vertices()[before].edge.d().DropDimension(drop_dimension)
                                      .Dot(v).GetSign();
      if (before_sign != 0) break;
      --before;
    }
    int after = 1;
    while (vertices()[after].edge.d().DropDimension(drop_dimension)
                            .Dot(v).IsZero()) {
      assert(after < vertices().size());
      ++after;
    }
    assert(before_sign != 0);
    // Return same dir first then opposite dir.
    if (before_sign > 0) {
      return std::make_pair(before, after);
    } else {
      return std::make_pair(after, before);
    }
  }
  // Copy `v` into `v_flipped` and negate `v_flipped` as necessary such that it
  // roughly points in the same direction as the 0th edge.
  bool flipped = initial_dir_sign < 0;
  Vector2<vector_bits> v_flipped = v;
  if (flipped) v_flipped.Negate();
  auto initial_dist = vertices()[0].vertex.vector_from_origin()
                                   .DropDimension(drop_dimension)
                                   .Dot(v_flipped);

  // Current range being considered by the binary search. The range includes
  // the begin side but excludes the end side.
  size_t begin = 1;
  size_t end = vertices().size();

  while (true) {
    size_t mid = (begin + end) / 2;
    if (vertices()[mid].edge.d().DropDimension(drop_dimension)
                       .Dot(v_flipped).GetSign() < 0) {
      if (flipped) {
        return std::make_pair(mid, 0);
      } else {
        return std::make_pair(0, mid);
      }
    }
    auto dist = vertices()[mid].vertex.vector_from_origin()
                               .DropDimension(drop_dimension).Dot(v_flipped);
    // This check works as long as there are no duplicate vertices in the
    // polygon. Coincident vertices are okay.
    assert (dist != initial_dist);
    if (dist < initial_dist) {
      begin = mid + 1;
    } else {
      end = mid;
    }
  }
}

}  // walnut

#endif // WALNUT_CONVEX_POLYGON_H__
