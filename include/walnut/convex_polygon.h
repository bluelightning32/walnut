#ifndef WALNUT_CONVEX_POLYGON_H__
#define WALNUT_CONVEX_POLYGON_H__

#include <sstream>
#include <vector>
#include <utility>

#include "walnut/assignable_wrapper.h"
#include "walnut/convex_polygon_edge.h"
#include "walnut/convex_polygon_split_info.h"
#include "walnut/convex_polygon_vertex_iterator.h"
#include "walnut/half_space3.h"
#include "walnut/homo_point3.h"
#include "walnut/member_forward.h"
#include "walnut/plucker_line.h"
#include "walnut/point3.h"

namespace walnut {

// A 2D ConvexPolygon embedded in R^3. The vertices are stored using
// homogeneous coordinates.
//
// `EdgeParentTemplate` specifies additional data that the caller can associate
// with each vertex. The type must be copy-constructible.
//
// All of the mutating functions are marked as protected. So this class can be
// instatiated directly and used as an immutable convex polygon, or a derived
// class such as `MutableConvexPolygon` can be used which exposes the mutating
// functions.
template <typename EdgeParentTemplate = EdgeInfoRoot>
class ConvexPolygon {
 public:
  static_assert(std::is_base_of<EdgeInfoRoot, EdgeParentTemplate>::value,
                "EdgeParentTemplate should inherit from EdgeInfoRoot");

  using EdgeParent = EdgeParentTemplate;
  using EdgeRep = ConvexPolygonEdge<EdgeParent>;
  using EdgeVector = std::vector<AssignableWrapper<EdgeRep>>;
  using ConstVertexIterator =
    ConvexPolygonVertexIterator<typename EdgeVector::const_iterator>;

  // Subclasses can inherit from this. `NewEdgeParent` should be the subclass's
  // class's EdgeInfo type.
  template <typename FinalPolygon, typename NewEdgeParent>
  using MakeParent = ConvexPolygon<NewEdgeParent>;

  ConvexPolygon() : plane_(HalfSpace3::Zero()), drop_dimension_(-1) { }

  ConvexPolygon(const ConvexPolygon& other) = default;

  ConvexPolygon(RValueKey<ConvexPolygon> other)
    noexcept(std::is_nothrow_move_constructible<EdgeVector>::value)
    : plane_(std::move(other.get().plane_)),
      drop_dimension_(other.get().drop_dimension_),
      edges_(std::move(other.get().edges_)) { }

  // `EdgeParent` must be constructible from `OtherEdgeParent`.
  template <typename OtherEdgeParent>
  explicit ConvexPolygon(const ConvexPolygon<OtherEdgeParent>& other) :
    plane_(other.plane()), drop_dimension_(other.drop_dimension()),
    edges_(other.edges().begin(), other.edges().end()) { }

  ConvexPolygon(const HalfSpace3& plane, int drop_dimension,
                EdgeVector edges) :
      plane_(plane), drop_dimension_(drop_dimension),
      edges_(std::move(edges)) {
    assert(IsValidState());
  }

  ConvexPolygon(const HalfSpace3& plane, int drop_dimension,
                const std::vector<Point3>& vertices) :
      plane_(plane), drop_dimension_(drop_dimension) {
    edges_.reserve(vertices.size());
    const Point3* prev = &vertices.back();
    for (const Point3& vertex : vertices) {
      edges_.emplace_back(*prev, vertex);
      prev = &vertex;
    }
    assert(IsValidState());
  }

  ConvexPolygon(const HalfSpace3& plane, int drop_dimension,
                const std::vector<HomoPoint3>& vertices) :
      plane_(plane), drop_dimension_(drop_dimension) {
    edges_.reserve(vertices.size());
    const HomoPoint3* prev = &vertices.back();
    for (const HomoPoint3& vertex : vertices) {
      edges_.emplace_back(*prev, vertex);
      assert(edges_.back().IsValidState());
      prev = &vertex;
    }
    assert(IsValidState());
  }

  template <size_t n>
  ConvexPolygon(HalfSpace3&& plane, int drop_dimension,
                HomoPoint3 (&&vertices)[n]) :
      plane_(std::move(plane)), drop_dimension_(drop_dimension) {
    edges_.reserve(n);
    for (size_t i = 0; i < n - 1; ++i) {
      edges_.emplace_back(std::move(vertices[i]), vertices[i + 1]);
    }
    edges_.emplace_back(std::move(vertices[n - 1]), edges_[0].vertex());
    assert(IsValidState());
  }

  // Verifies the polygon is really convex
  bool IsValidState() const {
    if (vertex_count() == 0) return true;

    if (normal().components()[drop_dimension()].IsZero()) return false;
    const EdgeRep* prev_edge = &edges().back();
    for (const EdgeRep& edge : edges()) {
      if (!edge.IsValidState()) return false;
      if (!plane().IsCoincident(edge.vertex())) return false;

      PluckerLine expected_line(prev_edge->vertex(), edge.vertex());
      if (prev_edge->line() != expected_line) return false;
      if (!prev_edge->line().d().IsSameDir(expected_line.d())) return false;

      BigInt cross =
        prev_edge->line().d().DropDimension(drop_dimension()).Cross(
            edge.line().d().DropDimension(drop_dimension()));
      if (!cross.IsZero() &&
          cross.HasDifferentSign(normal().components()[drop_dimension()])) {
        // The vertex is reflex (not convex and not collinear).
        return false;
      }

      prev_edge = &edge;
    }
    return true;
  }

  size_t vertex_count() const {
    return edges_.size();
  }

  const HomoPoint3& vertex(size_t index) const {
    return edges_[index].vertex();
  }

  // Returns the information about an edge and the source vertex for that edge.
  const EdgeRep& edge(size_t index) const {
    return edges_[index];
  }

  const EdgeRep& const_edge(size_t index) const {
    return edge(index);
  }

  const EdgeVector& edges() const {
    return edges_;
  }

  ConstVertexIterator vertices_begin() const {
    return ConstVertexIterator(edges_.begin());
  }

  ConstVertexIterator vertices_end() const {
    return ConstVertexIterator(edges_.end());
  }

  const HalfSpace3& plane() const { return plane_; }

  const Vector3& normal() const { return plane().normal(); }

  // When this dimension is projected to 0, 'dropped', the vertices will not
  // become collinear (assuming they were not already collinear).
  int drop_dimension() const { return drop_dimension_; }

  // Gets the index of the lexicographically smallest vertex.
  size_t GetMinimumIndex() const {
    size_t min = 0;
    for (size_t i = 1; i < edges_.size(); ++i) {
      if (HomoPoint3::LexicographicallyLt(vertex(i), vertex(min))) {
        min = i;
      }
    }
    return min;
  }

  // Returns true if the other polygon is the same as this.
  //
  // Two polygons are considered equal if all of the following are true:
  // 1. Their planes are the same
  //    - different scales are okay.
  // 2. They have the same vertices in the same order
  //    - the vertices are in a cycle. It's okay if the polygon cycles start at
  //      different indices.
  //    - it's okay of the homogenous vertices have a different scale.
  // 3. The EdgeParent matches for each vertex.
  template <typename OtherEdgeParent>
  bool operator==(const ConvexPolygon<OtherEdgeParent>& other) const;

  // Returns false if the other polygon is the same as this.
  //
  // Two polygons are considered equal if all of the following are true:
  // 1. Their planes are the same
  //    - different scales are okay.
  // 2. They have the same vertices in the same order
  //    - the vertices are in a cycle. It's okay if the polygon cycles start at
  //      different indices.
  //    - it's okay of the homogenous vertices have a different scale.
  // 3. The EdgeParent matches for each vertex.
  template <typename OtherEdgeParent>
  bool operator!=(const ConvexPolygon<OtherEdgeParent>& other) const {
    return !(*this == other);
  }

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
  std::pair<size_t, size_t> GetOppositeEdgeIndicesBisect(
      const Vector2& v, int drop_dimension) const;

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
  size_t GetExtremeIndexBisect(const Vector2& v,
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
  size_t GetExtremeIndexBisect(const Vector3& v) const {
    auto v_projected = v.DropDimension(drop_dimension()).Scale(
        normal().components()[drop_dimension()]);
    auto normal_projected = normal().DropDimension(
        drop_dimension()).Scale(v.components()[drop_dimension()]);
    auto new_vector = v_projected - normal_projected;
    // sign_adjust is -1 if the drop dimension in the plane normal is negative,
    // and 1 otherwise.
    int sign_adjust = normal().components()[drop_dimension()].GetAbsMult();
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
  std::pair<int, size_t> GetPosSideVertex(
      const HalfSpace2& half_space, int drop_dimension,
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
  std::pair<int, size_t> GetNegSideVertex(
      const HalfSpace2& half_space, int drop_dimension,
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
  std::pair<int, size_t> GetLastNegSideVertex(
      const HalfSpace2& half_space, int drop_dimension,
      size_t neg_side_index, int neg_side_type, size_t pos_side_index) const;

  // Returns the greatest index of a vertex in the positive side of the
  // half-space, or on the plane, such that the index is within:
  //   [pos_side_index, GetGreaterCycleIndex(pos_side_index), neg_side_index)
  //
  // Although the return value is actually modded by the number of vertices.
  // Specifically the first element of the returned pair will be greater than 0
  // if the last index is inside the positive half-space, or 0 if it is on the
  // plane. The second element of the returned pair is the index of the last
  // positive side vertex. The last vertex comes from the range
  // [pos_side_index, GetGreaterCycleIndex(pos_side_index, neg_side_index)),
  // although the value is modded by the number of vertices in the polygon
  // before it is returned.
  //
  // `neg_side_index` is the index of a vertex on the negative side of the
  // half-space, or on the plane.
  //
  // `pos_side_type` must be greater than 0 if `pos_side_index` refers to a
  // vertex in the positive half-space, or it must be 0 if `pos_side_index`
  // refers to a vertex on the plane.
  //
  // The vertex is found using a binary search. This algorithm is good for
  // ConvexPolgyons with many vertices, but a regular linear search is faster
  // for ConvexPolygons with fewer vertices (roughly 5 or fewer vertices).
  std::pair<int, size_t> GetLastPosSideVertex(
      const HalfSpace2& half_space, int drop_dimension,
      size_t neg_side_index, size_t pos_side_index, int pos_side_type) const {
    std::pair<int, size_t> flipped = GetLastNegSideVertex(
        -half_space, drop_dimension, pos_side_index,
        /*neg_side_type=*/-pos_side_type, neg_side_index);
    return std::make_pair(-flipped.first, flipped.second);
  }

  // Returns information about how to build the positive and negative sides of
  // a ConvexPolygon split by a 3D half-space.
  //
  // On the returned info, `ShouldEmitOnPlane`, `ShouldEmitNegativeChild`, and
  // `ShouldEmitPositiveChild` indicate which sides of `half_space` the polygon
  // is present on.
  //
  // If exactly one of `ShouldEmitOnPlane`, `ShouldEmitNegativeChild`, or
  // `ShouldEmitPositiveChild` returns true, then the input polygon should be
  // copied entirely to that child. For a negative or positive child, any
  // vertex not within that range of the returned info is a vertex that touches
  // the plane.
  //
  // Otherwise if `ShouldEmitNegativeChild` and `ShouldEmitPositiveChild` both
  // return true, the info should be passed to `CreateSplitChildren` to create
  // both children.
  ConvexPolygonSplitInfo GetSplitInfo(const HalfSpace3& half_space) const;

  // Returns the vertex indices for the positive and negative sides of a
  // ConvexPolygon split by a 2D half-space.
  //
  // `GetSplitKey` should be called instead of this function. This function is
  // only exposed for testing purposes.
  //
  // The polygon's vertices are first projected to 2D by dropping
  // `drop_dimension` before comparing them against `half_space2`. The
  // polygon's plane normal must be non-zero in the `drop_dimension` component.
  //
  // The ranges are found using a binary search. This algorithm is good for
  // ConvexPolgyons with many vertices, but a regular linear search is faster
  // for ConvexPolygons with fewer vertices (roughly 10 or fewer vertices).
  ConvexPolygonSplitRanges FindSplitRangesBisect(
      const HalfSpace2& half_space2,
      int drop_dimension) const;

  // Returns the vertex indices for the positive and negative sides of a
  // ConvexPolygon split by a 2D half-space.
  //
  // `GetSplitKey` should be called instead of this function. This function is
  // only exposed for testing purposes.
  //
  // The polygon's vertices are first projected to 2D by dropping
  // `drop_dimension` before comparing them against `half_space2`. The
  // polygon's plane normal must be non-zero in the `drop_dimension` component.
  //
  // The ranges are found using a linear search. This algorithm is good for
  // ConvexPolgyons with few vertices, but a bisect search is faster for
  // ConvexPolygons with more vertices (roughly 10 or more vertices).
  ConvexPolygonSplitRanges FindSplitRangesLinear(
      const HalfSpace2& half_space2,
      int drop_dimension) const;

  // Creates both split children.
  //
  // This function may only be called if `split.ShouldEmitNegativeChild()`
  // `split.ShouldEmitPositiveChild()` are both true.
  //
  // The first entry of the returned pair is the negative child, and the second
  // entry is the positive child.
  //
  // The last 2 vertices of the negative child will be on the split
  // plane. Whereas for the postive child, the first and last vertices will be
  // on the split plane.
  std::pair<ConvexPolygon, ConvexPolygon> CreateSplitChildren(
      const ConvexPolygonSplitInfo& split) const {
    std::pair<ConvexPolygon, ConvexPolygon> result;
    FillInSplitChildren(*this, split, result.first, result.second);
    return result;
  }

  // Creates both split children.
  //
  // This function may only be called if `split.ShouldEmitNegativeChild()`
  // `split.ShouldEmitPositiveChild()` are both true.
  //
  // The first entry of the returned pair is the negative child, and the second
  // entry is the positive child.
  //
  // The last 2 vertices of the negative child will be on the split
  // plane. Whereas for the postive child, the first and last vertices will be
  // on the split plane.
  static std::pair<ConvexPolygon, ConvexPolygon> CreateSplitChildren(
      RValueKey<ConvexPolygon> polygon,
      ConvexPolygonSplitInfo&& split) {
    std::pair<ConvexPolygon, ConvexPolygon> result;
    FillInSplitChildren(std::move(polygon.get()), std::move(split),
                        result.first, result.second);
    return result;
  }

  // Return a string representation of the polygon that uses decimal points to
  // approximate the vertex coordinates.
  std::string Approximate() const;

  // Return a string representation of the polygon that uses decimal points to
  // approximate the vertex coordinates and does not include the extra edge
  // data fields.
  std::string ApproximateNoData() const;

  // Returns the signed area of the polygon projected so that `drop_dimension`
  // is removed.
  //
  // A negative area is returned if the polygon is clockwise, or a positive
  // area is returned if the polygon is counter-clockwise. The first item in
  // the returned pair is the numerator and the denominator is second.
  std::pair<BigInt, BigInt> GetProjectedArea(int drop_dimension) const;

  // Returns the centroid of the polygon projected so that `drop_dimension`
  // is removed.
  HomoPoint2 GetProjectedCentroid(int drop_dimension) const;

  // Returns the centroid of the polygon.
  HomoPoint3 GetCentroid() const {
    return plane_.ProjectOntoPlane(GetProjectedCentroid(drop_dimension_),
                                   drop_dimension_);
  }

  void ReducePlaneNormal() {
    plane_.Reduce();
  }

 protected:
  // Returns the information about an edge and the source vertex for that edge.
  EdgeRep& edge(size_t index) {
    return edges_[index];
  }

  // Splits the edge at `split_index` by adding a new edge that starts at
  // `mid_point`.
  //
  // The caller must ensure that `mid_point` is on the `split_index` edge.
  //
  // One of the split edges will remain at `split_index`, and the other will be
  // at (split_index + 1).
  // This function may invalidate pointers to edges. The updated pointers can
  // be obtained with the `edge` function.
  void SplitEdge(size_t split_index, HomoPoint3 mid_point) {
    const EdgeRep& original_edge = edge(split_index);
    // Verify that `mid_point` is on the line of the original edge.
    assert(original_edge.line().IsCoincident(mid_point));
    // Verify that `mid_point` comes after the start vertex of the original
    // edge.
    assert(
        (original_edge.line().d().Dot(original_edge.vertex()
                                     .vector_from_origin()) *
         mid_point.w()).LessThan(
           /*flip=*/original_edge.vertex().w().HasDifferentSign(mid_point.w()),
           original_edge.line().d().Dot(mid_point.vector_from_origin()) *
           original_edge.vertex().w()));
    // Verify that `mid_point` comes before the end vertex of the original
    // edge.
    assert(
        (original_edge.line().d().Dot(mid_point.vector_from_origin()) *
         vertex((split_index + 1) % vertex_count()).w()).LessThan(
          /*flip=*/mid_point.w().HasDifferentSign(
            vertex((split_index + 1) % vertex_count()).w()),
          original_edge.line().d().Dot(edge((split_index + 1) % vertex_count()).
                                       vertex().vector_from_origin() *
                                       mid_point.w())));
    edges_.insert(edges_.begin() + split_index + 1,
                  AssignableWrapper<EdgeRep>(original_edge, mid_point));
  }

  // Tries to merge `other` into `this` by joining the half edges at
  // `my_edge_index` and `other_edge_index`. If the polygons are successfully
  // merged, true is returned and all of the edges are removed from `other`.
  //
  // After merging the polygons, if either vertex of the merged half edges is
  // redundant, and edge.TryMerge returns true, then the vertex is removed.
  //
  // The caller must ensure that the endpoints of the edges match up.
  //
  // The caller must ensure that the `nonzero_edge_dimension` component of the
  // edge vector is non-zero.
  //
  // False will be returned if the polygons cannot be merged for of any of the
  // following reasons:
  // * The normal vectors of the polygons point in different directions.
  // * The merged polygon would not be convex.
  bool TryMergePolygon(int nonzero_edge_dimension, size_t my_edge_index,
                       ConvexPolygon& other, size_t other_edge_index) {
    assert(!edge(my_edge_index).line().d()
           .components()[nonzero_edge_dimension].IsZero());
    assert(edge(my_edge_index).vertex() ==
           other.vertex((other_edge_index + 1) % other.vertex_count()));
    assert(other.edge(other_edge_index).vertex() ==
           vertex((my_edge_index + 1) % vertex_count()));

    if (!normal().DropDimension(nonzero_edge_dimension).IsSameDir(
          other.normal().DropDimension(nonzero_edge_dimension))) {
      return false;
    }

    const EdgeRep& my_prev_edge =
      edge((my_edge_index - 1 + vertex_count()) % vertex_count());
    const EdgeRep& other_next_edge =
      other.edge((other_edge_index + 1 + other.vertex_count()) %
                 other.vertex_count());

    const EdgeRep& my_next_edge =
      edge((my_edge_index + 1 + vertex_count()) % vertex_count());
    const EdgeRep& other_prev_edge =
      other.edge((other_edge_index - 1 + other.vertex_count()) %
                 other.vertex_count());

    // The twist of the merged polygon at the vertex that used to be
    // my_edge_index.
    BigIntWord my_twist =
      my_prev_edge.line().d().DropDimension(drop_dimension_).Cross(
          other_next_edge.line().d().DropDimension(drop_dimension_)).GetSign();
    BigIntWord normal_sign = normal().components()[drop_dimension_].GetSign();
    if (my_twist == 0) {
      if (my_prev_edge.line().d().components()[nonzero_edge_dimension]
          .HasDifferentSign(other_next_edge.line().d()
                            .components()[nonzero_edge_dimension])) {
        return false;
      }
    } else {
      if ((my_twist ^ normal_sign) < 0) {
        return false;
      }
    }
    // The twist of the merged polygon at the vertex that used to be
    // other_edge_index.
    BigIntWord other_twist =
      other_prev_edge.line().d().DropDimension(drop_dimension_).Cross(
          my_next_edge.line().d().DropDimension(drop_dimension_)).GetSign();
    if (other_twist == 0) {
      if (other_prev_edge.line().d().components()[nonzero_edge_dimension]
          .HasDifferentSign(my_next_edge.line().d()
                            .components()[nonzero_edge_dimension])) {
        return false;
      }
    } else {
      if ((other_twist ^ normal_sign) < 0) {
        return false;
      }
    }

    // All checks passed. Merge it.
    const size_t old_size = edges_.size();
    const size_t old_other_size = other.edges_.size();
    edges_.reserve(old_size + old_other_size - 2);
    std::move_iterator<typename EdgeVector::iterator> range1_begin(
        other.edges_.begin() + other_edge_index + 1);
    std::move_iterator<typename EdgeVector::iterator> range1_end(
        other.edges_.end());
    std::move_iterator<typename EdgeVector::iterator> range2_begin(
        other.edges_.begin());
    std::move_iterator<typename EdgeVector::iterator> range2_end(
        other.edges_.begin() + other_edge_index);
    if (range1_begin != range1_end) {
      edges_[my_edge_index] = *range1_begin;
      ++range1_begin;
    } else {
      edges_[my_edge_index] = *range2_begin;
      ++range2_begin;
    }
    // First insert the remaining other.edges_ at the end of edges_.
    edges_.insert(edges_.end(), range1_begin, range1_end);
    edges_.insert(edges_.end(), range2_begin, range2_end);
    // Now rotate the edges inserted from other into the correct place in the
    // middle of edges_.
    std::rotate(edges_.begin() + my_edge_index + 1, edges_.begin() + old_size,
                edges_.end());
    other.edges_.clear();
    for (auto it = edges_.begin() + my_edge_index;
         it != edges_.begin() + (my_edge_index + old_other_size - 1); ++it) {
      it->EdgeMoved(*this);
    }

    if (my_twist == 0 &&
        TryMergeEdge((my_edge_index - 1 + edges_.size()) % edges_.size())) {
      --my_edge_index;
    }
    if (other_twist == 0) {
      TryMergeEdge(my_edge_index + old_other_size - 2);
    }

    return true;
  }

  // Try to merge the edge at `index` with the next edge.
  bool TryMergeEdge(size_t index) {
    assert(index < vertex_count());
    const size_t next_index = (index + 1) % vertex_count();
    if (!edge(index).CanMerge(/*next=*/edge(next_index))) {
      return false;
    }
    edges_[index].Merge(/*next=*/edges_[next_index]);
    edges_.erase(edges_.begin() + next_index);
    return true;
  }

  // `EdgeParent` must be assignable from `OtherEdgeParent`.
  template <typename OtherEdgeParent>
  ConvexPolygon& operator=(const ConvexPolygon<OtherEdgeParent>& other) {
    plane_ = other.plane();
    drop_dimension_ = other.drop_dimension();
    edges_.assign(other.edges().begin(), other.edges().end());
    return *this;
  }

  ConvexPolygon& operator=(const ConvexPolygon& other) {
    return operator=<EdgeParent>(other);
  }

  ConvexPolygon& operator=(RValueKey<ConvexPolygon> other) {
    plane_ = std::move(other.get().plane());
    drop_dimension_ = other.get().drop_dimension();
    edges_ = std::move(other.get().edges_);
    return *this;
  }

  // Sorts `edges_`, such that the lexicographically minimum vertex comes
  // first.
  //
  // Sorting the vertices does not affect the shape of the polygon.
  void SortVertices() {
    RotateEdges(GetMinimumIndex());
  }

  // Rotates `edges_`, such that the edge at `offset` becomes the first.
  //
  // The caller must ensure:
  //   0 <= offset < vertex_count()
  //
  // Rotating the vertices does not affect the shape of the polygon.
  void RotateEdges(size_t offset) {
    std::rotate(edges_.begin(), edges_.begin() + offset, edges_.end());
  }

  // Creates both split children.
  //
  // This function may only be called if `split.ShouldEmitNegativeChild()`
  // `split.ShouldEmitPositiveChild()` are both true.
  //
  // On the returned `neg_child`, the last 2 vertices will be on the split
  // plane. Whereas for `pos_child`, the first and last vertices will be on
  // the split plane.
  //
  // `ParentRef` must be a reference to a ConvexPolygon. If `ParentRef` is an
  // rvalue reference, the parent is put into an unspecified state afterwards,
  // such that it is only safe to call the destructor or assignment operator on
  // the parent.
  //
  // `SplitInfoRef` must be a reference to a ConvexPolygonSplitInfo. If
  // `SplitInfoRef` is an rvalue reference, the `split` is put into an
  // unspecified state afterwards, such that it is only safe to call the
  // destructor or assignment operator on it.
  template <typename ParentRef, typename SplitInfoRef>
  static void FillInSplitChildren(ParentRef&& parent, SplitInfoRef&& split,
                                  ConvexPolygon& neg_child,
                                  ConvexPolygon& pos_child);

  // Reverse the vertices and flip the normal of the polygon.
  void Invert() {
    if (vertex_count() == 0) return;

    // Reverse vertices [1, n)
    using EdgeIterator = typename EdgeVector::iterator;
    for (EdgeIterator i = edges_.begin() + 1, j = edges_.end() - 1; i < j;
         ++i, --j) {
      using std::swap;
      swap(i->vertex_, j->vertex_);
    }

    // Reverse edges [0, n)
    for (EdgeIterator i = edges_.begin(), j = edges_.end() - 1; i < j;
         ++i, --j) {
      using std::swap;
      swap(i->line_, j->line_);
    }

    // Negate the edges
    for (AssignableWrapper<EdgeRep>& edge : edges_) {
      edge.line_.Negate();
    }

    plane_.Negate();
  }

 private:
  template <typename OtherEdgeParent>
  friend class ConvexPolygon;

  // The plane that all of the vertices are in.
  HalfSpace3 plane_;
  // When this dimension is projected to 0, 'dropped', the vertices will not
  // become collinear (assuming they were not already collinear).
  int drop_dimension_;
  // Each entry in edges_ contains information about that edge and the source
  // vertex of that edge.
  EdgeVector edges_;
};

template <typename EdgeParent>
template <typename OtherEdgeParent>
bool ConvexPolygon<EdgeParent>::operator==(
    const ConvexPolygon<OtherEdgeParent>& other) const {
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
    if (other.vertex(match_offset).DropDimension(drop_dimension()) ==
        vertex(0).DropDimension(drop_dimension())) {
      break;
    }
    ++match_offset;
  }
  for (size_t i = 1, j = match_offset + 1; i < vertex_count(); ++i, ++j) {
    if (vertex(i).DropDimension(drop_dimension()) !=
        other.vertex(j % vertex_count()).DropDimension(drop_dimension())) {
      return false;
    }
    const EdgeParent& edge_parent = edge(i);
    const OtherEdgeParent& other_edge_parent = other.edge(j % vertex_count());
    if (edge_parent != other_edge_parent) {
      return false;
    }
  }
  return true;
}

template <typename EdgeParent>
std::pair<size_t, size_t>
ConvexPolygon<EdgeParent>::GetOppositeEdgeIndicesBisect(
    const Vector2& v, int drop_dimension) const {
  BigIntWord initial_dir_sign =
    edge(0).line().d().DropDimension(drop_dimension).Dot(v).GetSign();
  if (initial_dir_sign == 0) {
    // The 0th edge is perpendicular to `v`. Find the first non-perpendicular
    // edge before and after `v`.
    size_t before = vertex_count() - 1;
    BigIntWord before_sign;
    while (true) {
      // A well formed ConvexPolygon with a valid drop_dimension should always
      // find a match.
      assert(static_cast<ssize_t>(before) > 0);
      before_sign = edge(before).line().d().DropDimension(drop_dimension)
                                .Dot(v).GetSign();
      if (before_sign != 0) break;
      --before;
    }
    size_t after = 1;
    while (edge(after).line().d().DropDimension(drop_dimension)
                      .Dot(v).IsZero()) {
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
  Vector2 v_flipped = v;
  if (flipped) v_flipped.Negate();
  auto initial_dist = edge(0).vertex().vector_from_origin()
                             .DropDimension(drop_dimension).Dot(v_flipped);

  const BigInt& initial_dist_denom = edge(0).vertex().w();

  // Current range being considered by the binary search. The range includes
  // the begin side but excludes the end side.
  size_t begin = 1;
  size_t end = vertex_count();

  while (true) {
    size_t mid = (begin + end) / 2;
    if (edge(mid).line().d().DropDimension(drop_dimension)
                .Dot(v_flipped).GetSign() < 0) {
      if (flipped) {
        return std::make_pair(mid, 0);
      } else {
        return std::make_pair(0, mid);
      }
    }
    auto dist = vertex(mid).vector_from_origin()
                           .DropDimension(drop_dimension).Dot(v_flipped);
    const BigInt& dist_denom = vertex(mid).w();
    if ((initial_dist*dist_denom).LessThan(
          /*flip=*/initial_dist_denom.HasDifferentSign(dist_denom),
          dist*initial_dist_denom)) {
      begin = mid + 1;
    } else {
      end = mid;
    }
  }
}

template <typename EdgeParent>
size_t ConvexPolygon<EdgeParent>::GetExtremeIndexBisect(
    const Vector2& v, int drop_dimension) const {
  if (v.IsZero()) return static_cast<size_t>(-1);

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
    if (edge(mid % vertex_count()).line().d().DropDimension(drop_dimension)
                                  .Dot(v).GetSign() > 0) {
      begin = mid;
    } else {
      end = mid;
    }
  }
  return end % vertex_count();
}

template <typename EdgeParent>
std::pair<int, size_t> ConvexPolygon<EdgeParent>::GetPosSideVertex(
    const HalfSpace2& half_space, int drop_dimension,
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
    if (edge(mid % vertex_count()).line().d().DropDimension(drop_dimension)
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

template <typename EdgeParent>
std::pair<int, size_t> ConvexPolygon<EdgeParent>::GetNegSideVertex(
    const HalfSpace2& half_space, int drop_dimension,
    size_t same_dir_index, size_t opp_dir_index) const {
  const auto opp_result = GetPosSideVertex(-half_space, drop_dimension,
                                           opp_dir_index, same_dir_index);
  return std::make_pair(-opp_result.first, opp_result.second);
}

template <typename EdgeParent>
std::pair<int, size_t> ConvexPolygon<EdgeParent>::GetLastNegSideVertex(
    const HalfSpace2& half_space, int drop_dimension,
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

template <typename EdgeParent>
template <typename ParentRef, typename SplitInfoRef>
void ConvexPolygon<EdgeParent>::FillInSplitChildren(
    ParentRef&& parent, SplitInfoRef&& split,
    ConvexPolygon& neg_child, ConvexPolygon& pos_child) {
  assert(split.ShouldEmitNegativeChild() && split.ShouldEmitPositiveChild());
  assert(&neg_child != &parent);
  assert(&pos_child != &parent);

  neg_child.edges_.clear();
  neg_child.edges_.reserve(split.neg_range().second -
                           split.neg_range().first + 2);
  neg_child.plane_ = parent.plane_;
  neg_child.drop_dimension_ = parent.drop_dimension_;

  pos_child.edges_.clear();
  pos_child.edges_.reserve(split.pos_range().second -
                           split.pos_range().first + 2);
  pos_child.plane_ = MemberForward<ParentRef>(parent.plane_);
  pos_child.drop_dimension_ = parent.drop_dimension_;

  assert(split.neg_range().second - split.neg_range().first <=
         parent.vertex_count());
  for (size_t i = split.neg_range().first; i < split.neg_range().second; ++i) {
    neg_child.edges_.push_back(MemberForward<ParentRef>(parent.edges_[
            i % parent.vertex_count()]));
  }

  if (!split.has_new_shared_point2) {
    // The range [neg_range.second, pos_range.first) is non-empty, holds
    // exactly 1 vertex, and is shared between the negative and positive
    // children.
    neg_child.edges_.emplace_back(
        parent.edge(split.neg_range().second % parent.vertex_count()),
        split.new_line);
    pos_child.edges_.push_back(MemberForward<ParentRef>(parent.edges_[
          split.neg_range().second % parent.vertex_count()]));
  } else {
    pos_child.edges_.emplace_back(neg_child.edges_.back(),
                                  split.new_shared_point2);
    assert(neg_child.edges_.size() < neg_child.edges_.capacity());
    neg_child.edges_.emplace_back(
        neg_child.edges_.back(),
        MemberForward<SplitInfoRef>(split.new_shared_point2), split.new_line);
  }

  for (size_t i = split.pos_range().first; i < split.pos_range().second; ++i) {
    pos_child.edges_.push_back(MemberForward<ParentRef>(parent.edges_[
            i % parent.vertex_count()]));
  }

  if (!split.has_new_shared_point1) {
    // The range [pos_range.second, neg_range.first) is non-empty, holds
    // exactly 1 vertex, and is shared between the negative and positive
    // children.
    pos_child.edges_.emplace_back(
        parent.edge(split.pos_range().second % parent.vertex_count()),
        -split.new_line);
    neg_child.edges_.push_back(MemberForward<ParentRef>(parent.edges_[
          split.pos_range().second % parent.vertex_count()]));
  } else {
    neg_child.edges_.emplace_back(pos_child.edges_.back(),
                                  split.new_shared_point1);
    assert(pos_child.edges_.size() < pos_child.edges_.capacity());
    pos_child.edges_.emplace_back(
        pos_child.edges_.back(),
        MemberForward<SplitInfoRef>(split.new_shared_point1),
        -MemberForward<SplitInfoRef>(split.new_line));
  }
}

template <typename EdgeParent>
ConvexPolygonSplitInfo ConvexPolygon<EdgeParent>::GetSplitInfo(
    const HalfSpace3& half_space) const {
  int flip = normal().components()[drop_dimension()].GetAbsMult();
  ConvexPolygonSplitInfo result;
  result.new_line = PluckerLine(plane_, half_space);

  if (!result.new_line.IsValid()) {
    // half_space is parallel to plane_.
    //
    // Now calculate the following:
    //   If half.n > 0, then
    //     if poly.d / poly.n < half.d / half.n, then
    //       poly is below half
    //     else
    //       poly is above half
    //    else
    //     if poly.d / poly.n < half.d / half.n, then
    //       poly is above half
    //     else
    //       poly is below half
    //
    // The above is equivalent to:
    //   If half.n > 0, then
    //     if poly.d * half.n / poly.n < half.d, then
    //       poly is below half
    //     else
    //       poly is above half
    //    else
    //     if poly.d * half.n / poly.n < half.d, then
    //       poly is below half
    //     else
    //       poly is above half
    //
    // Which simplifies to:
    //   if poly.d * half.n / poly.n < half.d, then
    //     poly is below half
    //   else
    //     poly is above half
    int compare = (plane().d() *
                   half_space.normal().components()[drop_dimension()]).Compare(
        half_space.d() *
        normal().components()[drop_dimension()]) * flip;
    if (compare < 0) {
      // The polygon is entirely on the negative side.
      result.ranges.neg_range.second = vertex_count();
    } else if (compare > 0) {
      // The polygon is entirely on the positive side.
      result.ranges.pos_range.second = vertex_count();
    }
    // Else the polygon is on the plane.
    assert(result.IsValid(vertex_count()));
    return result;
  }

  auto half_space2 = result.new_line.Project2D(drop_dimension()) * -flip;
  result.ranges = vertex_count() < 10 ?
    FindSplitRangesLinear(half_space2, drop_dimension()) :
    FindSplitRangesBisect(half_space2, drop_dimension());
  assert(result.ranges.IsValid(vertex_count()));

  if (!result.ShouldEmitPositiveChild() || !result.ShouldEmitNegativeChild()) {
    assert(result.IsValid(vertex_count()));
    return result;
  }

  if (result.ranges.neg_range.second % vertex_count() ==
      result.ranges.pos_range.first % vertex_count()) {
    result.has_new_shared_point2 = true;
    size_t last_neg_index = (result.ranges.neg_range.second + vertex_count() -
                             1) % vertex_count();
    result.new_shared_point2 =
      edge(last_neg_index).line().Intersect(half_space);
  }

  if (result.ranges.pos_range.second % vertex_count() ==
      result.ranges.neg_range.first % vertex_count()) {
    result.has_new_shared_point1 = true;
    size_t last_pos_index = (result.ranges.pos_range.second + vertex_count() -
                             1) % vertex_count();
    result.new_shared_point1 =
      edge(last_pos_index).line().Intersect(half_space);
  }
  assert(result.IsValid(vertex_count()));
  return result;
}

template <typename EdgeParent>
ConvexPolygonSplitRanges ConvexPolygon<EdgeParent>::FindSplitRangesBisect(
    const HalfSpace2& half_space2,
    int drop_dimension) const {
  assert(!normal().components()[drop_dimension].IsZero());
  assert(half_space2.IsValid());
  std::pair<size_t, size_t> opposite_edges = GetOppositeEdgeIndicesBisect(
      /*v=*/half_space2.normal(), drop_dimension);

  std::pair<int, size_t> neg_side_vertex = GetNegSideVertex(
      half_space2, drop_dimension,
      /*same_dir_index=*/opposite_edges.first,
      /*opp_dir_index=*/opposite_edges.second);

  if (neg_side_vertex.first >= 0) {
    // The source polygon is entirely in the positive half-space. Some of the
    // vertices may be coincident with the plane. The first such coincident
    // vertex index (if any) is neg_side_vertex.second.
    ConvexPolygonSplitRanges indices;
    size_t index = neg_side_vertex.second;
    indices.pos_range.second = index + vertex_count();

    if (neg_side_vertex.first == 0) {
      // One or more vertices are coincident with the plane.
      assert(half_space2.IsCoincident(
            vertex(index).DropDimension(drop_dimension)));
      do {
        ++index;
      } while (half_space2.IsCoincident(
            vertex(index % vertex_count()).DropDimension(drop_dimension)));
    }
    indices.pos_range.first = index;
    return indices;
  }

  std::pair<int, size_t> pos_side_vertex = GetPosSideVertex(
      half_space2, drop_dimension,
      /*same_dir_index=*/opposite_edges.first,
      /*opp_dir_index=*/opposite_edges.second);

  if (pos_side_vertex.first <= 0) {
    // The source polygon is entirely in the negative half-space. Some of the
    // vertices may be coincident with the plane. The first such coincident
    // vertex index (if any) is pos_side_vertex.second.
    ConvexPolygonSplitRanges indices;
    size_t index = pos_side_vertex.second;
    indices.neg_range.second = index + vertex_count();

    if (pos_side_vertex.first == 0) {
      // One or more vertices are coincident with the plane.
      assert(half_space2.IsCoincident(
            vertex(index).DropDimension(drop_dimension)));
      do {
        ++index;
      } while (half_space2.IsCoincident(
            vertex(index % vertex_count()).DropDimension(drop_dimension)));
    }
    indices.neg_range.first = index;
    return indices;
  }

  std::pair<int, size_t> neg_before_split = GetLastNegSideVertex(
      half_space2, drop_dimension, neg_side_vertex.second,
      /*neg_side_type=*/-1, pos_side_vertex.second);

  std::pair<int, size_t> pos_before_split = GetLastPosSideVertex(
      half_space2, drop_dimension, neg_side_vertex.second,
      pos_side_vertex.second, /*pos_side_type=*/1);

  ConvexPolygonSplitRanges indices;

  indices.neg_range.first = pos_before_split.second + 1;
  indices.neg_range.second = GetGreaterCycleIndex(pos_before_split.second,
                                                  neg_before_split.second) +
                             (neg_before_split.first != 0);

  indices.pos_range.first = neg_before_split.second + 1;
  indices.pos_range.second = GetGreaterCycleIndex(neg_before_split.second,
                                                  pos_before_split.second) +
                             (pos_before_split.first != 0);
  return indices;
}

template <typename EdgeParent>
ConvexPolygonSplitRanges ConvexPolygon<EdgeParent>::FindSplitRangesLinear(
    const HalfSpace2& half_space2,
    int drop_dimension) const {
  assert(!normal().components()[drop_dimension].IsZero());
  assert(half_space2.IsValid());
  assert(vertex_count() > 0);

  int initial_compare =
    half_space2.Compare(vertex(0).DropDimension(drop_dimension));

  if (initial_compare == 0) {
    // The ConvexPolygon started with a vertex on the line. First find the
    // first vertex that is not on the line.
    size_t index = 1;
    while (true) {
      // A valid ConvexPolygon is not entirely collinear.
      assert (index != vertex_count());
      initial_compare =
        half_space2.Compare(vertex(index).DropDimension(drop_dimension));
      if (initial_compare != 0) break;
      ++index;
    }
    ConvexPolygonSplitRanges result;
    std::pair<size_t, size_t>* first_side;
    std::pair<size_t, size_t>* second_side;
    if (initial_compare < 0) {
      first_side = &result.neg_range;
      second_side = &result.pos_range;
    } else {
      first_side = &result.pos_range;
      second_side = &result.neg_range;
    }
    first_side->first = index;

    // Keep looping until index points to a vertex on the other side.
    // first_side->second is set before leaving the loop.
    int compare;
    while (true) {
      ++index;
      if (index == vertex_count()) {
        first_side->second = index;
        return result;
      }
      compare =
        half_space2.Compare(vertex(index).DropDimension(drop_dimension));
      if (compare == 0) {
        // This vertex is on the line. Skip past any more vertices that are on
        // the line before leaving the parent while loop.
        first_side->second = index;
        while (true) {
          ++index;
          if (index == vertex_count()) {
            return result;
          }
          compare =
            half_space2.Compare(vertex(index).DropDimension(drop_dimension));
          if (compare != 0) break;
        }
        break;
      } else if ((initial_compare ^ compare) < 0) {
        first_side->second = index;
        break;
      }
    }

    assert(compare != 0);
    second_side->first = index;
    // Keep looping as long as the vertex is on the second side.
    while (true) {
      ++index;
      if (index == vertex_count()) {
        break;
      }
      compare =
        half_space2.Compare(vertex(index).DropDimension(drop_dimension));
      if (compare == 0) {
        break;
      }
    }
    second_side->second = index;
    return result;
  }

  // The ConvexPolygon started with a vertex not on the line.
  ConvexPolygonSplitRanges result;
  std::pair<size_t, size_t>* first_side;
  std::pair<size_t, size_t>* second_side;
  if (initial_compare < 0) {
    first_side = &result.neg_range;
    second_side = &result.pos_range;
  } else {
    first_side = &result.pos_range;
    second_side = &result.neg_range;
  }

  // Keep looping until index points to a vertex on the other side.
  // first_side->second is set before leaving the loop.
  int compare;
  size_t index = 1;
  while (true) {
    if (index == vertex_count()) {
      first_side->second = index;
      return result;
    }
    compare = half_space2.Compare(vertex(index).DropDimension(drop_dimension));
    if (compare == 0) {
      // This vertex is on the line. Skip past any more vertices that are on
      // the line before leaving the parent while loop.
      first_side->second = index;
      while (true) {
        ++index;
        if (index == vertex_count()) {
          return result;
        }
        compare =
          half_space2.Compare(vertex(index).DropDimension(drop_dimension));
        if (compare != 0) break;
      }
      if ((initial_compare ^ compare) >= 0) {
        // We're back to the first side again. The polygon is completely on one
        // side.
        first_side->first = index;
        first_side->second += vertex_count();
        assert(result.IsValid(vertex_count()));
        return result;
      }
      break;
    } else if ((initial_compare ^ compare) < 0) {
      first_side->second = index;
      break;
    }
    ++index;
  }

  assert(compare != 0);
  assert((initial_compare ^ compare) < 0);
  second_side->first = index;
  // Keep looping until a vertex is found that is on the first side or touching
  // the line.
  while (true) {
    ++index;
    if (index == vertex_count()) {
      second_side->second = index;
      return result;
    }
    compare = half_space2.Compare(vertex(index).DropDimension(drop_dimension));
    if (compare == 0 || (initial_compare ^ compare) >= 0) {
      second_side->second = index;
      break;
    }
  }

  // Keep looping until the next vertex back on the first side is found.
  while (compare == 0) {
    ++index;
    if (index == vertex_count()) {
      return result;
    }
    compare =
      half_space2.Compare(vertex(index).DropDimension(drop_dimension));
  }
  assert((initial_compare ^ compare) >= 0);
  first_side->first = index;
  first_side->second += vertex_count();
  return result;
}

template <typename EdgeParent>
std::string ConvexPolygon<EdgeParent>::Approximate() const {
  std::ostringstream out;
  out << "[ normal=" << normal() << " ";
  bool first = true;
  for (const auto& edge : edges()) {
    if (!first) out << ", ";
    first = false;
    edge.Approximate(out);
  }
  out << "]";
  return out.str();
}

template <typename EdgeParent>
std::string ConvexPolygon<EdgeParent>::ApproximateNoData() const {
  std::ostringstream out;
  out << "[";
  bool first = true;
  for (const auto& edge : edges()) {
    if (!first) out << ", ";
    first = false;
    out << edge.ApproximateNoData();
  }
  out << "]";
  return out.str();
}

template <typename EdgeParent>
std::pair<BigInt, BigInt> ConvexPolygon<EdgeParent>::GetProjectedArea(
    int drop_dimension) const {
  if (vertex_count() < 3) return std::make_pair(BigInt(0), BigInt(1));

  // Calculate the following formula in an optimized way:
  //
  //         to N-1
  // A = 1/2 sum (x_i/w_i * y_(i+1)/w_(i+1) - x_(i+1)/w_(i+1) * y_i / w_i)
  //         from i=0
  const HomoPoint3& last_vertex = vertex(vertex_count() - 1);
  BigInt numerator = last_vertex.GetComponentAfterDrop(0, drop_dimension) *
                     vertex(0).GetComponentAfterDrop(1, drop_dimension) -
                     vertex(0).GetComponentAfterDrop(0, drop_dimension) *
                     last_vertex.GetComponentAfterDrop(1, drop_dimension);
  BigInt denominator = last_vertex.w();

  for (size_t i = 0; i < vertex_count() - 1; ++i) {
    const HomoPoint3& v = vertex(i);
    const HomoPoint3& next_v = vertex(i + 1);
    const BigInt gcd = denominator.GetGreatestCommonDivisor(next_v.w());
    const BigInt numerator_converter = next_v.w() / gcd;
    const BigInt new_term_converter = denominator / gcd;

    numerator = numerator_converter * numerator + new_term_converter * (
                  v.GetComponentAfterDrop(0, drop_dimension) *
                  next_v.GetComponentAfterDrop(1, drop_dimension) -
                  next_v.GetComponentAfterDrop(0, drop_dimension) *
                  v.GetComponentAfterDrop(1, drop_dimension));

    denominator = new_term_converter * v.w();
  }
  return std::make_pair(std::move(numerator),
                        denominator * last_vertex.w() * 2);
}

template <typename EdgeParent>
HomoPoint2 ConvexPolygon<EdgeParent>::GetProjectedCentroid(
    int drop_dimension) const {
  if (!vertex_count()) return HomoPoint2(0, 0, 0);

  // Calculate the following formula in an optimized way:
  //
  //            to N-1
  // C.x = 1/6A sum (x_i + x_(i+1))*
  //                (x_i/w_i * y_(i+1)/w_(i+1) - x_(i+1)/w_(i+1) * y_i / w_i)
  //            from i=0
  //
  //            to N-1
  // C.y = 1/6A sum (y_i + y_(i+1))*
  //                (x_i/w_i * y_(i+1)/w_(i+1) - x_(i+1)/w_(i+1) * y_i / w_i)
  //            from i=0
  const HomoPoint3& last_vertex = vertex(vertex_count() - 1);
  const BigInt& last0 = last_vertex.GetComponentAfterDrop(0, drop_dimension);
  const BigInt& last1 = last_vertex.GetComponentAfterDrop(1, drop_dimension);
  const BigInt& first0 = vertex(0).GetComponentAfterDrop(0, drop_dimension);
  const BigInt& first1 = vertex(0).GetComponentAfterDrop(1, drop_dimension);

  BigInt area_accumulator = last0 * first1 - first0 * last1;
  BigInt centroid_accumulator0 =
    (last0 * vertex(0).w() + first0 * last_vertex.w()) * area_accumulator;
  BigInt centroid_accumulator1 =
    (last1 * vertex(0).w() + first1 * last_vertex.w()) * area_accumulator;
  BigInt denominator = last_vertex.w();

  for (size_t i = 0; i < vertex_count() - 1; ++i) {
    const HomoPoint3& v = vertex(i);
    const BigInt& cur0 = v.GetComponentAfterDrop(0, drop_dimension);
    const BigInt& cur1 = v.GetComponentAfterDrop(1, drop_dimension);
    const HomoPoint3& next_v = vertex(i + 1);
    const BigInt& next0 = next_v.GetComponentAfterDrop(0, drop_dimension);
    const BigInt& next1 = next_v.GetComponentAfterDrop(1, drop_dimension);

    const BigInt gcd = denominator.GetGreatestCommonDivisor(next_v.w());
    const BigInt numerator_converter = next_v.w() / gcd;
    const BigInt new_term_converter = denominator / gcd;

    const BigInt area_term =
      new_term_converter * (cur0 * next1 - next0 * cur1);

    area_accumulator = numerator_converter * area_accumulator + area_term;
    const BigInt numerator_converter_sqr =
      numerator_converter * numerator_converter;
    centroid_accumulator0 = centroid_accumulator0 * numerator_converter_sqr +
      (cur0 * next_v.w() + next0 * v.w()) * new_term_converter * area_term;
    centroid_accumulator1 = centroid_accumulator1 * numerator_converter_sqr +
      (cur1 * next_v.w() + next1 * v.w()) * new_term_converter * area_term;

    denominator = new_term_converter * v.w();
  }
  return HomoPoint2(centroid_accumulator0, centroid_accumulator1,
                    area_accumulator * 3 * denominator * last_vertex.w());
}

template <typename EdgeParent>
std::ostream& operator<<(std::ostream& out,
                         const ConvexPolygon<EdgeParent>& polygon) {
  out << "[";
  bool first = true;
  for (const auto& edge : polygon.edges()) {
    if (!first) out << ", ";
    first = false;
    out << edge;
  }
  out << "]";
  return out;
}

}  // walnut

#endif // WALNUT_CONVEX_POLYGON_H__
