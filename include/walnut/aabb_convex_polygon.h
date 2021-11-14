#ifndef WALNUT_AABB_CONVEX_POLYGON_H__
#define WALNUT_AABB_CONVEX_POLYGON_H__

#include <type_traits>

#include "walnut/convex_polygon.h"
#include "walnut/convex_vertex_aabb_tracker.h"

namespace walnut {

struct AABBConvexPolygonKey { };

// Adds an axis-aligned bounding box on top of a ConvexPolygon subclass. The
// axis-aligned bounding box speeds up calls to `GetSplitInfo` when the polygon
// is entirely on one side or the other of the splitting half-space. Although
// the bounding box also has a fixed maintenance cost in `CreateSplitChildren`.
template <typename ParentTemplate = ConvexPolygon<>>
class AABBConvexPolygon : public ParentTemplate, public AABBConvexPolygonKey {
 public:
  static_assert(!std::is_base_of<AABBConvexPolygonKey, ParentTemplate>::value,
                "AABBConvexPolygon should not wrap another "
                "AABBConvexPolygon.");
  using Parent = ParentTemplate;
  using typename Parent::EdgeRep;

  // Subclasses can inherit from this. `NewEdgeParent` should be the subclass's
  // EdgeInfo type.
  template <typename FinalPolygon, typename NewEdgeParent>
  using MakeParent =
    AABBConvexPolygon<typename Parent::template MakeParent<FinalPolygon,
                                                           NewEdgeParent>>;

  AABBConvexPolygon() { }

  AABBConvexPolygon(const AABBConvexPolygon&) = default;

  AABBConvexPolygon(AABBConvexPolygon&& other)
    noexcept(
        std::is_nothrow_constructible<
          AABBConvexPolygon, RValueKey<AABBConvexPolygon>
        >::value)
    : AABBConvexPolygon(RValueKey<AABBConvexPolygon>(std::move(other))) { }

  AABBConvexPolygon(RValueKey<AABBConvexPolygon> other)
    noexcept(std::is_nothrow_constructible<Parent, RValueKey<Parent>>::value)
    : Parent(RValueKey<Parent>(other)),
      aabb_tracker_(std::move(other.get().aabb_tracker_)) { }

  // `EdgeParent` must be constructible from `OtherEdgeParent`.
  template <typename OtherParent>
  explicit AABBConvexPolygon(const AABBConvexPolygon<OtherParent> & other) :
    Parent(other),
    aabb_tracker_(other.aabb_tracker_) { }

  template <typename OtherParentPolygon,
            std::enable_if_t<!std::is_base_of<AABBConvexPolygon,
                                              OtherParentPolygon>::value,
                             bool> = true>
  explicit AABBConvexPolygon(const OtherParentPolygon& other) :
      Parent(other),
      aabb_tracker_(Parent::vertices_begin(), Parent::vertices_end()) { }

  AABBConvexPolygon(const HalfSpace3& plane, int drop_dimension,
                    std::vector<EdgeRep> edges) :
      Parent(plane, drop_dimension, std::move(edges)),
      aabb_tracker_(Parent::vertices_begin(), Parent::vertices_end()) { }

  AABBConvexPolygon(const HalfSpace3& plane, int drop_dimension,
                    const std::vector<Point3>& vertices) :
      Parent(plane, drop_dimension, vertices),
      aabb_tracker_(Parent::vertices_begin(), Parent::vertices_end()) { }

  AABBConvexPolygon(const HalfSpace3& plane, int drop_dimension,
                    const std::vector<HomoPoint3>& vertices) :
      Parent(plane, drop_dimension, vertices),
      aabb_tracker_(Parent::vertices_begin(), Parent::vertices_end()) { }

  // Returns false if the polygon is not convex.
  bool IsValidState() const {
    if (!Parent::IsValidState()) return false;
    if (vertex_count() == 0) return true;
    if (!aabb_tracker_.IsValidState(vertex_count())) return false;
    return true;
  }

  AABBConvexPolygon& operator=(const AABBConvexPolygon& other) = default;

  // `EdgeParent` must be assignable from `OtherEdgeParent`.
  template <typename OtherParent>
  AABBConvexPolygon& operator=(const AABBConvexPolygon<OtherParent>& other) {
    Parent::operator=(other);
    aabb_tracker_ = other.aabb_tracker_;
    return *this;
  }

  AABBConvexPolygon& operator=(AABBConvexPolygon&& other) {
    return operator=(RValueKey<AABBConvexPolygon>(std::move(other)));
  }

  AABBConvexPolygon& operator=(RValueKey<AABBConvexPolygon> other) {
    Parent::operator=(RValueKey<Parent>(other));
    aabb_tracker_ = std::move(other.get().aabb_tracker_);
    return *this;
  }

  using Parent::vertex_count;
  using Parent::vertex;

  const AABB& aabb() const {
    return aabb_tracker_.aabb();
  }

  // Returns the vertex of this polygon that is furthest in the positive
  // direction along the `dimension` axis. Technically the vertex is not
  // necessarily on the axis, but just coincident with the furthest plane
  // perpendicular to the axis.
  //
  // The caller must ensure that the polygon has at least 1 vertex.
  const HomoPoint3& max_vertex(int dimension) const {
    assert(vertex_count() > 0);
    return vertex(aabb_tracker_.max_indices()[dimension]);
  }

  // Returns the vertex of this polygon that is furthest in the negative
  // direction along the `dimension` axis. Technically the vertex is not
  // necessarily on the axis, but just coincident with the furthest plane
  // perpendicular to the axis.
  //
  // The caller must ensure that the polygon has at least 1 vertex.
  const HomoPoint3& min_vertex(int dimension) const {
    assert(vertex_count() > 0);
    return vertex(aabb_tracker_.min_indices()[dimension]);
  }

  // Overrides the non-virtual function from ConvexPolygon.
  void SortVertices() {
    RotateEdges(Parent::GetMinimumIndex());
  }

  // Overrides the non-virtual function from ConvexPolygon.
  void RotateEdges(size_t offset) {
    Parent::RotateEdges(offset);
    aabb_tracker_.RotateIndices(offset, vertex_count());
  }

  // Overrides the non-virtual function from ConvexPolygon.
  ConvexPolygonSplitInfo GetSplitInfo(const HalfSpace3& half_space) const {
    switch (aabb().GetPlaneSide(half_space)) {
      case -1:
        // All on the negative side.
        {
          ConvexPolygonSplitInfo result;
          result.ranges.neg_range.second = vertex_count();
          return result;
        }
      case 1:
        // All on the positive side.
        {
          ConvexPolygonSplitInfo result;
          result.ranges.pos_range.second = vertex_count();
          return result;
        }
      case 0:
        // Straddles the half-space.
        return Parent::GetSplitInfo(half_space);
      default:
        assert(false);
        return ConvexPolygonSplitInfo();
    }
  }

  // Overrides the non-virtual function from ConvexPolygon.
  std::pair<AABBConvexPolygon, AABBConvexPolygon> CreateSplitChildren(
      const ConvexPolygonSplitInfo& split) const {
    std::pair<AABBConvexPolygon, AABBConvexPolygon> result;
    FillInSplitChildren(*this, split, result.first, result.second);
    return result;
  }

  // Overrides the non-virtual function from ConvexPolygon.
  std::pair<AABBConvexPolygon, AABBConvexPolygon> CreateSplitChildren(
      ConvexPolygonSplitInfo&& split) && {
    std::pair<AABBConvexPolygon, AABBConvexPolygon> result;
    FillInSplitChildren(std::move(*this), std::move(split), result.first,
                        result.second);
    return result;
  }

  static std::pair<AABBConvexPolygon, AABBConvexPolygon> CreateSplitChildren(
      RValueKey<AABBConvexPolygon> polygon,
      ConvexPolygonSplitInfo&& split) {
    std::pair<AABBConvexPolygon, AABBConvexPolygon> result;
    FillInSplitChildren(std::move(polygon.get()), std::move(split),
                        result.first, result.second);
    return result;
  }

  template <typename OtherParent>
  bool operator==(const AABBConvexPolygon<OtherParent>& other) const {
    if (aabb() != other.aabb()) return false;
    return Parent::operator==(other);
  }

  // Allow the parent's operator== to be used for types that inherit from
  // ConvexPolygon but not AABBConvexPolygon.
  using Parent::operator==;

  bool TryMergePolygon(int nonzero_edge_dimension, size_t my_edge_index,
                       AABBConvexPolygon& other, size_t other_edge_index) {
    if (!Parent::TryMergePolygon(nonzero_edge_dimension, my_edge_index,
                                 other, other_edge_index)) {
      return false;
    }

    aabb_tracker_ = ConvexVertexAABBTracker(Parent::vertices_begin(),
                                            Parent::vertices_end());
    return true;
  }

 protected:
  // Overrides the non-virtual function from ConvexPolygon.
  template <typename ParentRef, typename SplitInfoRef>
  static void FillInSplitChildren(ParentRef&& parent, SplitInfoRef&& split,
                                  AABBConvexPolygon& neg_child,
                                  AABBConvexPolygon& pos_child) {
    // Make a copy of some variables in case FillInSplitChildren below changes
    // them.
    const size_t original_vertex_count = parent.vertex_count();
    ConvexPolygonSplitRanges ranges = split.ranges;
    Parent::FillInSplitChildren(std::forward<ParentRef>(parent),
                                std::forward<SplitInfoRef>(split), neg_child,
                                pos_child);
    auto aabb_children = parent.aabb_tracker_.CreateSplitChildren(
        original_vertex_count, neg_child.vertices_begin(),
        pos_child.vertices_begin(), ranges);
    neg_child.aabb_tracker_ = std::move(aabb_children.first);
    pos_child.aabb_tracker_ = std::move(aabb_children.second);
  }

 private:
  ConvexVertexAABBTracker aabb_tracker_;
};

// Adds AABBConvexPolygon on top of `Parent`, if `Parent` does not already
// inherit from AABBConvexPolygon, otherwise this is equal to `Parent`.
template <typename ParentTemplate>
using WrapAABBConvexPolygon =
  std::conditional<
    std::is_base_of<AABBConvexPolygonKey, ParentTemplate>::value,
    ParentTemplate, AABBConvexPolygon<ParentTemplate>
  >;

}  // walnut

#endif // WALNUT_CONVEX_POLYGON_H__
