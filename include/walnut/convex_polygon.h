#ifndef WALNUT_CONVEX_POLYGON_H__
#define WALNUT_CONVEX_POLYGON_H__

#include <vector>
#include <tuple>
#include <utility>

#include "walnut/half_space3.h"
#include "walnut/homo_point3.h"
#include "walnut/plucker_line.h"
#include "walnut/point3.h"

namespace walnut {

// An edge of a ConvexPolygon
template <int point3_bits_template = 32,
          typename VertexDataTemplate = std::tuple<>>
struct ConvexPolygonEdge {
  using Point3Rep = Point3<point3_bits_template>;
  using HomoPoint3Rep = HomoPoint3<(point3_bits_template - 1)*7 + 10,
                                   (point3_bits_template - 1)*6 + 10>;
  using LineRep = typename PluckerLineFromPlanesFromPoint3sBuilder<
    point3_bits_template>::PluckerLineRep;
  using VertexData = VertexDataTemplate;

  // This is used by FactoryWithVertexData to construct a ConvexPolygon and
  // specify the inital values of the vertex data.
  struct Point3WithVertexData : public Point3Rep {
    using Point3Rep::Point3Rep;

    Point3WithVertexData(int x, int y, int z, VertexData data) :
      Point3Rep(x, y, z), data(data) { }

    VertexData data;
  };

  // VertexData must be default-constructible to use this constructor.
  template <int other_point3_bits>
  ConvexPolygonEdge(const Point3<other_point3_bits>& vertex,
                    const Point3<other_point3_bits>& next_vertex) :
    vertex(vertex), line(vertex, next_vertex) { }

  ConvexPolygonEdge(const Point3WithVertexData& vertex,
                    const Point3WithVertexData& next_vertex) :
    vertex(vertex), line(vertex, next_vertex), data(vertex.data) { }

  template <int other_point3_bits,
            typename OtherVertexData>
  ConvexPolygonEdge(const ConvexPolygonEdge<other_point3_bits,
                                            OtherVertexData>& other) :
    vertex(other.vertex), line(other.line), data(other.data) { }

  static bool LexicographicallyLt(const ConvexPolygonEdge& a,
                                  const ConvexPolygonEdge& b) {
    return HomoPoint3Rep::LexicographicallyLt(a.vertex, b.vertex);
  }

  HomoPoint3Rep vertex;
  // This line starts at `vertex` and goes to the next vertex in the polygon.
  //
  // Notably:
  //   next_vertex == vertex + line.d()
  LineRep line;

  VertexData data;
};

// A 2D ConvexPolygon embedded in R^3. The vertices are stored using homogeneous
// coordinates.
//
// `VertexDataTemplate` specifies additional data that the caller can associate
// with each vertex. The type must be copy-constructible.
template <int point3_bits_template = 32,
          typename VertexDataTemplate = std::tuple<>>
class ConvexPolygon {
 public:
  using Point3Rep = Point3<point3_bits_template>;
  using VertexData = VertexDataTemplate;
  using EdgeRep = ConvexPolygonEdge<point3_bits_template, VertexDataTemplate>;

  // Defined in convex_polygon_factory.h. Use the alias `Factory` instead.
  template <typename Point3RepTemplate>
  class GenericFactory;

  // A class to build ConvexPolygons from iterators that produce Point3s.
  // VertexData must be default-constructible to use this factory.
  //
  // Include convex_polygon_factory.h to use this.
  using Factory = GenericFactory<Point3Rep>;
  // A class to build ConvexPolygons from iterators that produce
  // Point3WithVertexDatas.
  //
  // Include convex_polygon_factory.h to use this.
  using FactoryWithVertexData =
    GenericFactory<typename EdgeRep::Point3WithVertexData>;

  using HomoPoint3Rep = HomoPoint3<(point3_bits_template - 1)*7 + 10,
                             (point3_bits_template - 1)*6 + 10>;
  using HalfSpace3Rep =
    typename HalfSpace3FromPoint3Builder<point3_bits_template>::HalfSpace3Rep;
  using LineRep = typename PluckerLineFromPlanesFromPoint3sBuilder<
    point3_bits_template>::PluckerLineRep;

  // The minimum number of bits to support for each component of the vertex3's
  // that the polygon is built from.
  static constexpr int point3_bits = point3_bits_template;

  // The minimum number of bits to support for each of the x, y, and z
  // components for each vertex, after an arbitrary number of splits from
  // planes of the type HalfSpace3Rep.
  static constexpr int homo_point3_num_bits = HomoPoint3Rep::num_bits;
  // The minimum number of bits to support the w component for each vertex,
  // after an arbitrary number of splits from planes of the type HalfSpace3Rep.
  static constexpr int homo_point3_denom_bits =
    HomoPoint3Rep::denom_bits_template;

  ConvexPolygon() : plane_(HalfSpace3Rep::Zero()), drop_dimension_(-1) { }

  // `VertexData` must be constructible from `OtherVertexData`.
  template <int other_point3_bits, typename OtherVertexData>
  ConvexPolygon(const ConvexPolygon<other_point3_bits,
                                    OtherVertexData>& other) :
    plane_(other.plane()), drop_dimension_(other.drop_dimension()),
    edges_(other.edges().begin(), other.edges().end()) { }

  size_t vertex_count() const {
    return edges_.size();
  }

  const HomoPoint3Rep& vertex(size_t index) const {
    return edges_[index].vertex;
  }

  VertexData& vertex_data(size_t index) {
    return edges_[index].data;
  }

  const VertexData& vertex_data(size_t index) const {
    return edges_[index].data;
  }

  const EdgeRep& edge(size_t index) const {
    return edges_[index];
  }

  const std::vector<EdgeRep>& edges() const {
    return edges_;
  }

  const HalfSpace3Rep& plane() const { return plane_; }

  // When this dimension is projected to 0, 'dropped', the vertices will not
  // become collinear (assuming they were not already collinear).
  int drop_dimension() const { return drop_dimension_; }

  // Gets the index of the lexicographically smallest vertex.
  size_t GetMinimumIndex() const {
    size_t min = 0;
    for (size_t i = 1; i < edges_.size(); ++i) {
      if (HomoPoint3Rep::LexicographicallyLt(vertex(i), vertex(min))) {
        min = i;
      }
    }
    return min;
  }

  // Sorts `edges_`, such that the lexicographically minimum vertex comes
  // first.
  //
  // Sorting the vertices does not affect the shape of the polygon.
  void SortVertices() {
    std::rotate(edges_.begin(), edges_.begin() + GetMinimumIndex(),
                edges_.end());
  }

  // Returns true of the other polygon is the same as this.
  //
  // Two polygons are considered equal if all of the following are true:
  // 1. Their planes are the same
  //    - different scales are okay.
  // 2. They have the same vertices in the same order
  //    - the vertices are in a cycle. It's okay if the polygon cycles start at
  //      different indices.
  //    - it's okay of the homogenous vertices have a different scale.
  // 3. The VertexData matches for each vertex.
  template <int other_point3_bits, typename OtherVertexData>
  bool operator==(const ConvexPolygon<other_point3_bits,
                                      OtherVertexData>& other) const;

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
      const Vector2<vector_bits>& v, int drop_dimension) const;

  // Returns an index into vertices() such that:
  //   a <= ret < a + vertices.size()
  // and:
  //   ret % vertices().size() == b % vertices.size()
  //
  // This precondition must be true:
  //   b + vertices.size >= a
  constexpr size_t GetGreaterCycleIndex(size_t a, size_t b) const {
    assert(b + vertex_count() >= a);
    return a + (b - a + vertex_count())%vertex_count();
  }

  // Returns the index of the vertex that is farthest in the `v` direction
  // using a binary search.
  //
  // This function projects all of the vertices to 2D by dropping the
  // `drop_dimension`. That's why the input vector is a `Vector2`.
  // `drop_dimension` must refer to a non-zero component of the plane normal.
  //
  // The furthest vertex is the one whose vector from the origin has the
  // greatest dot product with `v`. If is a tie for vertex with the furthest
  // distance, then the lowest index is returned. If there is a tie between the
  // last vertex in the list and the 0th vertex, then the last vertex is
  // considered to have the lower index.
  //
  // The bisect part of the function name refers to how this function uses a
  // binary search to find the edges. This algorithm is good for ConvexPolgyons
  // with many vertices, but a regular linear search is faster for
  // ConvexPolygons with fewer vertices (roughly 10 or fewer vertices).
  //
  // If `v` is Zero, this returns static_cast<size_t>(-1).
  template <int vector_bits>
  size_t GetExtremeIndexBisect(const Vector2<vector_bits>& v,
                               int drop_dimension) const;

  // Returns the index of the vertex that is farthest in the `v` direction
  // using a binary search.
  //
  // The furthest vertex is the one whose vector from the origin has the
  // greatest dot product with `v`. If is a tie for vertex with the furthest
  // distance, then the lowest index is returned. If there is a tie between the
  // last vertex in the list and the 0th vertex, then the last vertex is
  // considered to have the lower index.
  //
  // The bisect part of the function name refers to how this function uses a
  // binary search to find the edges. This algorithm is good for ConvexPolgyons
  // with many vertices, but a regular linear search is faster for
  // ConvexPolygons with fewer vertices (roughly 10 or fewer vertices).
  //
  // If `v` is Zero or perpendicular to the polygon plane, this returns
  // static_cast<size_t>(-1).
  template <int vector_bits>
  size_t GetExtremeIndexBisect(const Vector3<vector_bits>& v) const {
    auto v_projected = v.DropDimension(drop_dimension()).Scale(
        plane().normal().components()[drop_dimension()]);
    auto normal_projected = plane().normal().DropDimension(
        drop_dimension()).Scale(v.components()[drop_dimension()]);
    auto new_vector = v_projected - normal_projected;
    // sign_adjust is -1 if the drop dimension in the plane normal is negative,
    // and 1 otherwise.
    int sign_adjust =
      plane().normal().components()[drop_dimension()].GetAbsMult();
    new_vector = new_vector * sign_adjust;
    return GetExtremeIndexBisect(new_vector,
                                 drop_dimension());
  }

  // Returns the index of the vertex that is in the given HalfSpace2, if any
  // exist.
  //
  // Specifically the first element of the returned pair indicates whether any
  // vertices exist in the positive half-space:
  //   1 : the second element of the return value is the index of a vertex
  //       in the positive half-space.
  //   0 : no vertices exist in the positive half-space. The returned vertex
  //       index is on the plane. If there are multiple vertices on the plane,
  //       the vertex with the lowest index is returned. If there is a tie
  //       between the last vertex in the list and the 0th vertex, then the
  //       last vertex is considered to have the lower index than the 0th.
  //   -1: no vertices exist in the positive half-space. The returned vertex
  //       index is in the negative half-space, but it is the closest vertex
  //       to the positive half-space. If there is a tie for closest between
  //       multiple vertices, the vertex with the lowest index is returned. If
  //       there is a tie between the last vertex in the list and the 0th
  //       vertex, then the last vertex is considered to have the lower index
  //       than the 0th.
  //
  // This function projects all of the vertices to 2D by dropping the
  // `drop_dimension`. That's why the input half-space is a HalfSpace2 instead
  // of a HalfSpace3. `drop_dimension` must refer to a non-zero component of
  // the plane normal.
  //
  // same_dir_index must be the edge source index of an edge pointing roughly
  // in the same direction as the half-space normal, and opp_dir_index must be
  // the edge source index of an edge pointing in roughtly the opposite
  // direction as the half-space normal. same_dir_index and opp_dir_index may
  // be obtained from `GetOppositeEdgeIndicesBisect`.
  //
  // The vertex is found using a binary search. This algorithm is good for
  // ConvexPolgyons with many vertices, but a regular linear search is faster
  // for ConvexPolygons with fewer vertices (roughly 5 or fewer vertices).
  template <int vector_bits, int dist_bits>
  std::pair<int, size_t> GetPosSideVertex(
      const HalfSpace2<vector_bits, dist_bits>& half_space, int drop_dimension,
      size_t same_dir_index, size_t opp_dir_index) const;

  // Returns the index of the vertex that is outside the given HalfSpace2, if
  // any exist.
  //
  // Specifically the first element of the returned pair indicates whether any
  // vertices exist in the negative half-space:
  //   -1 : the second element of the return value is the index of a vertex
  //       in the negative half-space.
  //   0 : no vertices exist in the negative half-space. The returned vertex
  //       index is on the plane. If there are multiple vertices on the plane,
  //       the vertex with the lowest index is returned. If there is a tie
  //       between the last vertex in the list and the 0th vertex, then the
  //       last vertex is considered to have the lower index than the 0th.
  //   1:  no vertices exist in the negative half-space. The returned vertex
  //       index is in the positive half-space, but it is the closest vertex
  //       to the negative half-space. If there is a tie for closest between
  //       multiple vertices, the vertex with the lowest index is returned. If
  //       there is a tie between the last vertex in the list and the 0th
  //       vertex, then the last vertex is considered to have the lower index
  //       than the 0th.
  //
  // This function projects all of the vertices to 2D by dropping the
  // `drop_dimension`. That's why the input half-space is a HalfSpace2 instead
  // of a HalfSpace3. `drop_dimension` must refer to a non-zero component of
  // the plane normal.
  //
  // same_dir_index must be the edge source index of an edge pointing roughly
  // in the same direction as the half-space normal, and opp_dir_index must be
  // the edge source index of an edge pointing in roughtly the opposite
  // direction as the half-space normal. same_dir_index and opp_dir_index may
  // be obtained from `GetOppositeEdgeIndicesBisect`.
  //
  // The vertex is found using a binary search. This algorithm is good for
  // ConvexPolgyons with many vertices, but a regular linear search is faster
  // for ConvexPolygons with fewer vertices (roughly 5 or fewer vertices).
  template <int vector_bits, int dist_bits>
  std::pair<int, size_t> GetNegSideVertex(
      const HalfSpace2<vector_bits, dist_bits>& half_space, int drop_dimension,
      size_t same_dir_index, size_t opp_dir_index) const;

  // Returns the greatest index of a vertex in the negative side of the
  // half-space, or on the plane, such that the index is within:
  //   [neg_side_index, GetGreaterCycleIndex(neg_side_index), pos_side_index)
  //
  // Although the return value is actually modded by the number of vertices.
  // Specifically the first element of the returned pair will be less than 0 if
  // the last index is inside the negative half-space, or 0 if it is on the
  // plane. The second element of the returned pair is the index of the last
  // negative side vertex. The last vertex comes from the range
  // [neg_side_index, GetGreaterCycleIndex(neg_side_index), pos_side_index)),
  // although the value is modded by the number of vertices in the polygon
  // before it is returned.
  //
  // `neg_side_index` is the index of a vertex on the negative side of the
  // half-space, or on the plane. `neg_side_type` must be less than 0 if
  // `neg_side_index` refers to a vertex in the negative half-space, or it must
  // be 0 if `neg_side_index` refers to a vertex on the plane.
  //
  // `pos_side_index` is the index of a vertex on the positive side of the
  // half-space, or on the plane.
  //
  // The vertex is found using a binary search. This algorithm is good for
  // ConvexPolgyons with many vertices, but a regular linear search is faster
  // for ConvexPolygons with fewer vertices (roughly 5 or fewer vertices).
  template <int vector_bits, int dist_bits>
  std::pair<int, size_t> GetLastNegSideVertex(
      const HalfSpace2<vector_bits, dist_bits>& half_space, int drop_dimension,
      size_t neg_side_index, int neg_side_type, size_t pos_side_index) const;

 private:
  ConvexPolygon(const HalfSpace3Rep& plane, int drop_dimension,
                std::vector<EdgeRep> edges) :
    plane_(plane), drop_dimension_(drop_dimension),
    edges_(std::move(edges)) { }

  // The plane that all of the vertices are in.
  HalfSpace3Rep plane_;
  // When this dimension is projected to 0, 'dropped', the vertices will not
  // become collinear (assuming they were not already collinear).
  int drop_dimension_;
  std::vector<EdgeRep> edges_;
};

template <int point3_bits, typename VertexData>
template <int other_point3_bits, typename OtherVertexData>
bool ConvexPolygon<point3_bits, VertexData>::operator==(
    const ConvexPolygon<other_point3_bits, OtherVertexData>& other) const {
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
  for (size_t i = 1, j = match_offset + 1; i < vertex_count(); ++i, ++j) {
    if (vertex(i) != other.vertex(j)) {
      return false;
    }
    if (vertex_data(i) != other.vertex_data(j)) {
      return false;
    }
  }
  return true;
}

template <int point3_bits, typename VertexData>
template <int vector_bits>
std::pair<size_t, size_t>
ConvexPolygon<point3_bits, VertexData>::GetOppositeEdgeIndicesBisect(
    const Vector2<vector_bits>& v, int drop_dimension) const {
  int initial_dir_sign = edge(0).line.d().DropDimension(drop_dimension)
                                .Dot(v).GetSign();
  if (initial_dir_sign == 0) {
    // The 0th edge is perpendicular to `v`. Find the first non-perpendicular
    // edge before and after `v`.
    int before = vertex_count() - 1;
    int before_sign;
    while (true) {
      // A well formed ConvexPolygon with a valid drop_dimension should always
      // find a match.
      assert(before > 0);
      before_sign = edge(before).line.d().DropDimension(drop_dimension)
                                .Dot(v).GetSign();
      if (before_sign != 0) break;
      --before;
    }
    int after = 1;
    while (edge(after).line.d().DropDimension(drop_dimension).Dot(v).IsZero()) {
      assert(after < vertex_count());
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
  auto initial_dist = edge(0).vertex.vector_from_origin()
                             .DropDimension(drop_dimension).Dot(v_flipped);

  // Current range being considered by the binary search. The range includes
  // the begin side but excludes the end side.
  size_t begin = 1;
  size_t end = vertex_count();

  while (true) {
    size_t mid = (begin + end) / 2;
    if (edge(mid).line.d().DropDimension(drop_dimension)
                 .Dot(v_flipped).GetSign() < 0) {
      if (flipped) {
        return std::make_pair(mid, 0);
      } else {
        return std::make_pair(0, mid);
      }
    }
    auto dist = edge(mid).vertex.vector_from_origin()
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

template <int point3_bits, typename VertexData>
template <int vector_bits>
size_t ConvexPolygon<point3_bits, VertexData>::GetExtremeIndexBisect(
    const Vector2<vector_bits>& v, int drop_dimension) const {
  if (v.IsZero()) return -1;

  std::pair<size_t, size_t> dir_indices =
    GetOppositeEdgeIndicesBisect(v, drop_dimension);

  // Current range being considered by the binary search. The range excludes
  // the begin side but includes the end side.
  //
  // INVARIANT: begin is the source index of an edge that points in roughly the
  // same direction as `v`.
  // INVARIANT: end is the source index of an edge that points in roughly the
  // opposite direction as `v` or perpendicular.
  size_t begin = dir_indices.first;
  size_t end = GetGreaterCycleIndex(begin, dir_indices.second);

  while (begin + 1 < end) {
    size_t mid = (begin + end) / 2;
    if (edge(mid % vertex_count()).line.d().DropDimension(drop_dimension)
                                  .Dot(v).GetSign() > 0) {
      begin = mid;
    } else {
      end = mid;
    }
  }
  return end % vertex_count();
}

template <int point3_bits, typename VertexData>
template <int vector_bits, int dist_bits>
std::pair<int, size_t> ConvexPolygon<point3_bits, VertexData>::GetPosSideVertex(
    const HalfSpace2<vector_bits, dist_bits>& half_space, int drop_dimension,
    size_t same_dir_index, size_t opp_dir_index) const {
  // Current range being considered by the binary search. The range excludes
  // the begin side but includes the end side.
  //
  // INVARIANT: begin is the source index of an edge that points in roughly the
  // same direction as `v`.
  // INVARIANT: end is the source index of an edge that points in roughly the
  // opposite direction as `v` or perpendicular.
  size_t begin = same_dir_index;
  size_t end = GetGreaterCycleIndex(begin, opp_dir_index);

  while (begin + 1 < end) {
    size_t mid = (begin + end) / 2;
    if (half_space.Compare(vertex(
          mid % vertex_count()).DropDimension(drop_dimension)) > 0) {
      return std::make_pair(1, mid);
    }
    if (edge(mid % vertex_count()).line.d().DropDimension(drop_dimension)
                                  .Dot(half_space.normal()).GetSign() > 0) {
      begin = mid;
    } else {
      end = mid;
    }
  }
  end %= vertex_count();
  return std::make_pair(
      half_space.Compare(vertex(end).DropDimension(drop_dimension)), end);
}

template <int point3_bits, typename VertexData>
template <int vector_bits, int dist_bits>
std::pair<int, size_t> ConvexPolygon<point3_bits, VertexData>::GetNegSideVertex(
    const HalfSpace2<vector_bits, dist_bits>& half_space, int drop_dimension,
    size_t same_dir_index, size_t opp_dir_index) const {
  const auto opp_result = GetPosSideVertex(-half_space, drop_dimension,
                                           opp_dir_index, same_dir_index);
  return std::make_pair(-opp_result.first, opp_result.second);
}

template <int point3_bits, typename VertexData>
template <int vector_bits, int dist_bits>
std::pair<int, size_t>
ConvexPolygon<point3_bits, VertexData>::GetLastNegSideVertex(
    const HalfSpace2<vector_bits, dist_bits>& half_space, int drop_dimension,
    size_t neg_side_index, int neg_side_type, size_t pos_side_index) const {
  // Current range being considered by the binary search. The range includes
  // the begin side but excludes the end side.
  //
  // INVARIANT: `begin` is the index of a vertex in the negative half-space or
  // on the plane. `begin_type` is less than 0 if `begin` is in the negative
  // half-space, otherwise it is 0.
  // INVARIANT: `end` is the index of a vertex in the positive half-space.
  size_t begin = neg_side_index;
  int begin_type = neg_side_type;
  size_t end = GetGreaterCycleIndex(begin, pos_side_index);
  assert(begin_type <= 0);

  while (begin + 1 < end) {
    size_t mid = (begin + end) / 2;
    int mid_type = half_space.Compare(
        vertex(mid % vertex_count()).DropDimension(drop_dimension));
    if (mid_type <= 0) {
      begin = mid;
      begin_type = mid_type;
    } else {
      end = mid;
    }
  }
  return std::make_pair(begin_type, begin % vertex_count());
}

}  // walnut

#endif // WALNUT_CONVEX_POLYGON_H__
