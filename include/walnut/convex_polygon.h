#ifndef WALNUT_CONVEX_POLYGON_H__
#define WALNUT_CONVEX_POLYGON_H__

#include <vector>
#include <utility>

#include "walnut/convex_polygon_edge.h"
#include "walnut/half_space3.h"
#include "walnut/homo_point3.h"
#include "walnut/plucker_line.h"
#include "walnut/point3.h"

namespace walnut {

// Stores information about which vertex indexes should be included in the
// child ConvexPolygons created from a split.
struct ConvexPolygonSplitRanges {
  bool IsValid() const {
    if (!(neg_range.first <= neg_range.second)) return false;

    if (!(pos_range.first <= pos_range.second)) return false;

    return true;
  }

  bool ShouldEmitNegativeChild() const {
    return neg_range.first != neg_range.second;
  }

  bool ShouldEmitPositiveChild() const {
    return pos_range.first != pos_range.second;
  }

  // [neg_range.first, neg_range.second) should be part of only the negative
  // child.
  //
  // This is the index of the first vertex that should be included in the
  // negative child. If neg_range.first == neg_range.second, then no negative
  // child should be emitted.
  std::pair<size_t, size_t> neg_range{0, 0};

  // [pos_range.first, pos_range.second) should be part of only the positive
  // child.
  //
  // This is the index of the first vertex that should be included in the
  // positive child. If pos_range.first == pos_range.second, then no positive
  // child should be emitted.
  std::pair<size_t, size_t> pos_range{0, 0};
};

// Stores information about how to build the ConvexPolygons on both sides of a
// plane.
template <int point3_bits_template = 32>
struct ConvexPolygonSplitInfo {
  using HomoPoint3Rep = HomoPoint3<(point3_bits_template - 1)*7 + 10,
                             (point3_bits_template - 1)*6 + 10>;
  using LineRep = typename PluckerLineFromPlanesFromPoint3sBuilder<
    point3_bits_template>::PluckerLineRep;

  bool ShouldEmitNegativeChild() const {
    return ranges.ShouldEmitNegativeChild();
  }

  bool ShouldEmitPositiveChild() const {
    return ranges.ShouldEmitPositiveChild();
  }

  bool ShouldEmitOnPlane() const {
    return !ranges.ShouldEmitNegativeChild() &&
           !ranges.ShouldEmitPositiveChild();
  }

  ConvexPolygonSplitRanges ranges;

  // A line that should used for the new edge in the negative child. -new_line
  // should be used for the positive child.
  //
  // This field is only initialized if both children are emitted.
  LineRep new_line;

  bool has_new_shared_point1 = false;
  // If `has_new_shared_point1` is set, this point should be inserted before
  // neg_range and after pos_range.
  HomoPoint3Rep new_shared_point1;

  bool has_new_shared_point2 = false;
  // If `has_new_shared_point2` is set, this point should be inserted before
  // pos_range and after neg_range.
  HomoPoint3Rep new_shared_point2;
};

template <typename InputPoint3Template,
          typename ConvexPolygonTemplate>
class ConvexPolygonFactory;

// A 2D ConvexPolygon embedded in R^3. The vertices are stored using homogeneous
// coordinates.
//
// `VertexDataTemplate` specifies additional data that the caller can associate
// with each vertex. The type must be copy-constructible.
template <int point3_bits_template = 32,
          typename VertexDataTemplate = NoVertexData>
class ConvexPolygon {
 public:
  using Point3Rep = Point3<point3_bits_template>;
  using VertexData = VertexDataTemplate;
  using EdgeRep = ConvexPolygonEdge<point3_bits_template, VertexDataTemplate>;
  using SplitInfoRep = ConvexPolygonSplitInfo<point3_bits_template>;

  // Stores information about how to build the ConvexPolygons on both sides of
  // a plane.
  //
  // This is only constructed by GetSplitInfo.
  template <typename ParentTemplate = const ConvexPolygon&>
  class SplitKey {
   public:
    using Parent = ParentTemplate;

    const SplitInfoRep& info() const {
      return info_;
    }

    bool ShouldEmitNegativeChild() const {
      return info().ShouldEmitNegativeChild();
    }

    bool ShouldEmitPositiveChild() const {
      return info().ShouldEmitPositiveChild();
    }

    bool ShouldEmitOnPlane() const {
      return info().ShouldEmitOnPlane();
    }

    const std::pair<size_t, size_t>& neg_range() const {
      return info().ranges.neg_range;
    }

    const std::pair<size_t, size_t>& pos_range() const {
      return info().ranges.pos_range;
    }

    // Creates the negative child from the split.
    //
    // This function may only be called if `ShouldEmitNegativeChild`
    // returns true and `ShouldEmitPositiveChild` returns false.
    //
    // If `Parent` is an rvalue reference, the parent is put into an
    // unspecified state afterwards, such that it is only safe to call the
    // destructor or assignment operator on the parent. Also if `Parent` is an
    // rvalue reference, then this function can only be called once.
    ConvexPolygon CreateNegativeChild() {
      assert(ShouldEmitNegativeChild());
      assert(!ShouldEmitPositiveChild());
      return ConvexPolygon(std::forward<Parent>(parent_));
    }

    // Creates the positive child from the split.
    //
    // This function may only be called if `ShouldEmitNegativeChild`
    // returns false and `ShouldEmitPositiveChild` returns true.
    //
    // If `Parent` is an rvalue reference, the parent is put into an
    // unspecified state afterwards, such that it is only safe to call the
    // destructor or assignment operator on the parent. Also if `Parent` is an
    // rvalue reference, then this function can only be called once.
    ConvexPolygon CreatePositiveChild() {
      assert(!ShouldEmitNegativeChild());
      assert(ShouldEmitPositiveChild());
      return ConvexPolygon(std::forward<Parent>(parent_));
    }

    // Creates the on plane child from the split.
    //
    // This function may only be called if `ShouldEmitOnPlane` returns true.
    //
    // If `Parent` is an rvalue reference, the parent is put into an
    // unspecified state afterwards, such that it is only safe to call the
    // destructor or assignment operator on the parent. Also if `Parent` is an
    // rvalue reference, then this function can only be called once.
    ConvexPolygon CreateOnPlaneChild() {
      assert(ShouldEmitOnPlane());
      return ConvexPolygon(std::forward<Parent>(parent_));
    }

    // Creates both split children.
    //
    // This function may only be called if `ShouldEmitNegativeChild`
    // `ShouldEmitPositiveChild` both return true.
    //
    // On the returned `neg_child`, the last 2 vertices will be on the split
    // plane. Whereas for `pos_child`, the first and last vertices will be on
    // the split plane.
    //
    // If `Parent` is an rvalue reference, the parent is put into an
    // unspecified state afterwards, such that it is only safe to call the
    // destructor or assignment operator on the parent. Also if `Parent` is an
    // rvalue reference, then this function can only be called once.
    void CreateSplitChildren(ConvexPolygon& neg_child,
                             ConvexPolygon& pos_child);

   private:
    friend ConvexPolygon;

    template <typename T>
    typename std::enable_if<
      std::is_rvalue_reference<Parent>::value &&
        !std::is_const<typename std::remove_reference<T>::type>::value,
      T&&>::type
    MakeForward(T& t) const {
      return std::move(t);
    }

    template <typename T>
    typename std::enable_if<!std::is_rvalue_reference<Parent>::value, T&>::type
    MakeForward(T& t) const {
      return t;
    }

    SplitKey(Parent parent, const SplitInfoRep& info) :
      parent_(std::forward<Parent>(parent)), info_(info) { }

    Parent parent_;

    SplitInfoRep info_;
  };

  // Defined in convex_polygon_factory.h. Use the alias `Factory` instead.
  template <typename Point3RepTemplate>
  using GenericFactory = ConvexPolygonFactory<Point3RepTemplate,
                                              ConvexPolygon>;

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
    GenericFactory<Point3WithVertexData<point3_bits_template, VertexData>>;

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
  explicit ConvexPolygon(const ConvexPolygon<other_point3_bits,
                                             OtherVertexData>& other) :
    plane_(other.plane()), drop_dimension_(other.drop_dimension()),
    edges_(other.edges().begin(), other.edges().end()) { }

  ConvexPolygon(const HalfSpace3Rep& plane, int drop_dimension,
                std::vector<EdgeRep> edges) :
      plane_(plane), drop_dimension_(drop_dimension),
      edges_(std::move(edges)) {
    assert(IsValidState());
  }

  // Verifies the polygon is really convex
  bool IsValidState() const {
    if (vertex_count() == 0) return true;

    if (!plane().IsValidState()) return false;
    if (plane().normal().components()[drop_dimension()].IsZero()) return false;
    const EdgeRep* prev_edge = &edges().back();
    for (const EdgeRep& edge : edges()) {
      if (!edge.line.IsValidState()) return false;
      if (!plane().IsCoincident(edge.vertex)) return false;

      LineRep expected_line(prev_edge->vertex, edge.vertex);
      if (prev_edge->line != expected_line) return false;
      if (!prev_edge->line.d().IsSameDir(expected_line.d())) return false;

      prev_edge = &edge;
    }
    return true;
  }

  // `VertexData` must be assignable from `OtherVertexData`.
  template <int other_point3_bits, typename OtherVertexData>
  ConvexPolygon& operator=(const ConvexPolygon<other_point3_bits,
                                               OtherVertexData>& other) {
    plane_ = other.plane();
    drop_dimension_ = other.drop_dimension();
    edges_.assign(other.edges().begin(), other.edges().end());
    return *this;
  }

  size_t vertex_count() const {
    return edges_.size();
  }

  const HomoPoint3Rep& vertex(size_t index) const {
    return edges_[index].vertex;
  }

  VertexData& vertex_data(size_t index) {
    return edges_[index].data();
  }

  const VertexData& vertex_data(size_t index) const {
    return edges_[index].data();
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

  // Returns true if the other polygon is the same as this.
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

  // Returns false if the other polygon is the same as this.
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
  bool operator!=(const ConvexPolygon<other_point3_bits,
                                      OtherVertexData>& other) const {
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
  template <int vector_bits, int dist_bits>
  std::pair<int, size_t> GetLastPosSideVertex(
      const HalfSpace2<vector_bits, dist_bits>& half_space, int drop_dimension,
      size_t neg_side_index, size_t pos_side_index, int pos_side_type) const {
    std::pair<int, size_t> flipped = GetLastNegSideVertex(
        -half_space, drop_dimension, pos_side_index,
        /*neg_side_type=*/-pos_side_type, neg_side_index);
    return std::make_pair(-flipped.first, flipped.second);
  }

  // Returns information about how to build the positive and negative sides of
  // a ConvexPolygon split by a 3D half-space.
  //
  // On the returned key, `ShouldEmitOnPlane`, `ShouldEmitNegativeChild`, and
  // `ShouldEmitPositiveChild` indicate which sides of `half_space` the polygon
  // is present on.
  //
  // If exactly one of `ShouldEmitOnPlane`, `ShouldEmitNegativeChild`, or
  // `ShouldEmitPositiveChild` returns true, then the input polygon should be
  // copied entirely to that child. For a negative or positive child, any
  // vertex not within that range of the returned key is a vertex that touches
  // the plane.
  //
  // Otherwise if `ShouldEmitNegativeChild` and `ShouldEmitPositiveChild` both
  // return true, the key should be passed to `GetSplitChildren` to create both
  // children.
  template <int vector_bits, int dist_bits>
  SplitKey<const ConvexPolygon&> GetSplitKey(
      const HalfSpace3<vector_bits, dist_bits>& half_space) const {
    return SplitKey<const ConvexPolygon&>(*this, GetSplitInfo(half_space));
  }

  // Overload that takes an rvalue reference
  template <int vector_bits, int dist_bits>
  SplitKey<ConvexPolygon&&> GetSplitKey(
      const HalfSpace3<vector_bits, dist_bits>& half_space) && {
    return SplitKey<ConvexPolygon&&>(std::move(*this),
                                     GetSplitInfo(half_space));
  }

  // Returns information about how to build the positive and negative sides of
  // a ConvexPolygon split by a 3D half-space.
  //
  // `GetSplitKey` should be called instead of this function. This function is
  // only exposed for testing purposes.
  template <int vector_bits, int dist_bits>
  SplitInfoRep GetSplitInfo(
      const HalfSpace3<vector_bits, dist_bits>& half_space) const;

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
  template <int vector_bits, int dist_bits>
  ConvexPolygonSplitRanges FindSplitRangesBisect(
      const HalfSpace2<vector_bits, dist_bits>& half_space2,
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
  template <int vector_bits, int dist_bits>
  ConvexPolygonSplitRanges FindSplitRangesLinear(
      const HalfSpace2<vector_bits, dist_bits>& half_space2,
      int drop_dimension) const;

 private:
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
    if (vertex_data(i) != other.vertex_data(j % vertex_count())) {
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

template <int point3_bits, typename VertexData>
template <typename Parent>
void
ConvexPolygon<point3_bits, VertexData>::SplitKey<Parent>::CreateSplitChildren(
    ConvexPolygon& neg_child, ConvexPolygon& pos_child) {
  assert(ShouldEmitNegativeChild() && ShouldEmitPositiveChild());

  neg_child.edges_.clear();
  neg_child.edges_.reserve(neg_range().second - neg_range().first + 2);
  neg_child.plane_ = parent_.plane_;
  neg_child.drop_dimension_ = parent_.drop_dimension_;

  pos_child.edges_.clear();
  neg_child.edges_.reserve(pos_range().second - pos_range().first + 2);
  pos_child.plane_ = MakeForward(parent_.plane_);
  pos_child.drop_dimension_ = parent_.drop_dimension_;

  for (size_t i = neg_range().first; i < neg_range().second; ++i) {
    neg_child.edges_.push_back(MakeForward(parent_.edges_[
            i % parent_.vertex_count()]));
  }

  if (!info_.has_new_shared_point2) {
    // The range [neg_range.second, pos_range.first) is non-empty, holds
    // exactly 1 vertex, and is shared between the negative and positive
    // children.
    neg_child.edges_.emplace_back(
        parent_.vertex(neg_range().second % parent_.vertex_count()),
        info_.new_line);
    pos_child.edges_.push_back(MakeForward(parent_.edges_[
          neg_range().second % parent_.vertex_count()]));
  } else {
    size_t last_neg_index = (neg_range().second +
                             parent_.vertex_count() - 1) %
                             parent_.vertex_count();
    neg_child.edges_.emplace_back(info_.new_shared_point2,
                                  info_.new_line);
    pos_child.edges_.emplace_back(
        MakeForward(info_.new_shared_point2),
        MakeForward(parent_.edges_[last_neg_index].line));
  }

  for (size_t i = pos_range().first; i < pos_range().second; ++i) {
    pos_child.edges_.push_back(MakeForward(parent_.edges_[
            i % parent_.vertex_count()]));
  }

  if (!info_.has_new_shared_point1) {
    // The range [pos_range.second, neg_range.first) is non-empty, holds
    // exactly 1 vertex, and is shared between the negative and positive
    // children.
    pos_child.edges_.emplace_back(
        parent_.vertex(pos_range().second % parent_.vertex_count()),
        -info_.new_line);
    neg_child.edges_.push_back(MakeForward(parent_.edges_[
          pos_range().second % parent_.vertex_count()]));
  } else {
    size_t last_pos_index = (pos_range().second +
                             parent_.vertex_count() - 1) %
                            parent_.vertex_count();
    pos_child.edges_.emplace_back(info_.new_shared_point1,
                                  -MakeForward(info_.new_line));
    neg_child.edges_.emplace_back(
        MakeForward(info_.new_shared_point1),
        MakeForward(parent_.edges_[last_pos_index].line));
  }
}

template <int point3_bits, typename VertexData>
template <int vector_bits, int dist_bits>
typename ConvexPolygon<point3_bits, VertexData>::SplitInfoRep
ConvexPolygon<point3_bits, VertexData>::GetSplitInfo(
    const HalfSpace3<vector_bits, dist_bits>& half_space) const {
  using PluckerLineBuilder =
    PluckerLineFromPlanesFromPoint3sBuilder<point3_bits>;
  using PluckerLineRep = typename PluckerLineBuilder::PluckerLineRep;

  int flip = plane().normal().components()[drop_dimension()].GetAbsMult();
  PluckerLineRep line = PluckerLineBuilder::Build(
      HalfSpace3<vector_bits, dist_bits>(half_space.normal() * flip,
                                         half_space.d() * flip), plane_);

  if (!line.IsValid()) {
    // half_space is parallel to plane_.
    int half_space_abs_mult =
      half_space.normal().components()[drop_dimension()].GetAbsMult();
    int compare = (plane().normal().components()[drop_dimension()] *
                   plane().d()).Compare(
                     half_space.normal().components()[drop_dimension()] *
                     half_space.d()) * half_space_abs_mult;
    SplitInfoRep result;
    if (compare < 0) {
      // The polygon is entirely on the negative side.
      result.ranges.neg_range.second = vertex_count();
    } else if (compare > 0) {
      // The polygon is entirely on the positive side.
      result.ranges.pos_range.second = vertex_count();
    }
    // Else the polygon is on the plane.
    return result;
  }

  auto half_space2 = line.Project2D(drop_dimension());
  SplitInfoRep result;
  result.ranges = vertex_count() < 10 ?
    FindSplitRangesLinear(half_space2, drop_dimension()) :
    FindSplitRangesBisect(half_space2, drop_dimension());

  if (!result.ShouldEmitPositiveChild() || !result.ShouldEmitNegativeChild()) {
    return result;
  }

  // If the input polygon is counter-clockwise in its projected form, then
  // `line` is in the correct orientation for the positive output polygon.
  int neg_line_mult =
    -plane().normal().components()[drop_dimension()].GetAbsMult();
  result.new_line = LineRep(line.d() * neg_line_mult,
                            line.m() * neg_line_mult);

  if (result.ranges.neg_range.second % vertex_count() ==
      result.ranges.pos_range.first % vertex_count()) {
    result.has_new_shared_point2 = true;
    size_t last_neg_index = (result.ranges.neg_range.second + vertex_count() -
                             1) % vertex_count();
    result.new_shared_point2 = edge(last_neg_index).line.Intersect(half_space);
  }

  if (result.ranges.pos_range.second % vertex_count() ==
      result.ranges.neg_range.first % vertex_count()) {
    result.has_new_shared_point1 = true;
    size_t last_pos_index = (result.ranges.pos_range.second + vertex_count() -
                             1) % vertex_count();
    result.new_shared_point1 = edge(last_pos_index).line.Intersect(half_space);
  }
  return result;
}

template <int point3_bits, typename VertexData>
template <int vector_bits, int dist_bits>
ConvexPolygonSplitRanges
ConvexPolygon<point3_bits, VertexData>::FindSplitRangesBisect(
    const HalfSpace2<vector_bits, dist_bits>& half_space2,
    int drop_dimension) const {
  assert(!plane().normal().components()[drop_dimension].IsZero());
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

template <int point3_bits, typename VertexData>
template <int vector_bits, int dist_bits>
ConvexPolygonSplitRanges
ConvexPolygon<point3_bits, VertexData>::FindSplitRangesLinear(
    const HalfSpace2<vector_bits, dist_bits>& half_space2,
    int drop_dimension) const {
  assert(!plane().normal().components()[drop_dimension].IsZero());
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

template <int point3_bits, typename VertexData>
std::ostream& operator<<(
    std::ostream& out,
    const ConvexPolygon<point3_bits, VertexData>& polygon) {
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
