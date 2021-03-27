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
  using typename Parent::HalfSpace3Rep;
  using typename Parent::EdgeRep;
  using typename Parent::SplitInfoRep;
  using AABBRep =
    typename ConvexVertexAABBTracker<Parent::homo_point3_num_bits,
                                     Parent::homo_point3_denom_bits>::AABBRep;

  // Subclasses can inherit from this. `NewEdgeParent` should be the subclass's
  // EdgeInfo type.
  template <typename FinalPolygon, typename NewEdgeParent>
  using MakeParent =
    AABBConvexPolygon<typename Parent::template MakeParent<FinalPolygon,
                                                           NewEdgeParent>>;

  using Parent::point3_bits;
  using Parent::homo_point3_num_bits;
  using Parent::homo_point3_denom_bits;

  AABBConvexPolygon() { }

  // `EdgeParent` must be constructible from `OtherEdgeParent`.
  template <typename OtherParent>
  explicit AABBConvexPolygon(const AABBConvexPolygon<OtherParent> & other) :
    Parent(other),
    aabb_tracker_(other.aabb_tracker_) { }

  template <typename OtherParentPolygon>
  explicit AABBConvexPolygon(const OtherParentPolygon& other) :
      Parent(other),
      aabb_tracker_(Parent::vertices_begin(), Parent::vertices_end()) { }

  AABBConvexPolygon(const HalfSpace3Rep& plane, int drop_dimension,
                    std::vector<EdgeRep> edges) :
      Parent(plane, drop_dimension, std::move(edges)),
      aabb_tracker_(Parent::vertices_begin(), Parent::vertices_end()) { }

  AABBConvexPolygon(const HalfSpace3Rep& plane, int drop_dimension,
                    const std::vector<Point3>& vertices) :
      Parent(plane, drop_dimension, vertices),
      aabb_tracker_(Parent::vertices_begin(), Parent::vertices_end()) { }

  template <size_t num_bits, size_t denom_bits>
  AABBConvexPolygon(const HalfSpace3Rep& plane, int drop_dimension,
                    const std::vector<HomoPoint3<num_bits,
                                                 denom_bits>>& vertices) :
      Parent(plane, drop_dimension, vertices),
      aabb_tracker_(Parent::vertices_begin(), Parent::vertices_end()) { }

  // Returns false if the polygon is not convex.
  bool IsValidState() const {
    if (!Parent::IsValidState()) return false;
    if (vertex_count() == 0) return true;
    if (!aabb_tracker_.IsValidState(vertex_count())) return false;
    return true;
  }

  // `EdgeParent` must be assignable from `OtherEdgeParent`.
  template <typename OtherParent>
  AABBConvexPolygon& operator=(const AABBConvexPolygon<OtherParent>& other) {
    Parent::operator=(other);
    aabb_tracker_ = other.aabb_tracker_;
    return *this;
  }

  using Parent::vertex_count;
  using Parent::vertex;

  const AABBRep& aabb() const {
    return aabb_tracker_.aabb();
  }

  // Overrides the non-virtual function from ConvexPolygon.
  void SortVertices() {
    RotateVertices(Parent::GetMinimumIndex());
  }

  // Overrides the non-virtual function from ConvexPolygon.
  void RotateVertices(size_t offset) {
    Parent::RotateVertices(offset);
    aabb_tracker_.RotateIndices(offset, vertex_count());
  }

  // Overrides the non-virtual function from ConvexPolygon.
  template <size_t vector_bits, size_t dist_bits>
  SplitInfoRep GetSplitInfo(
      const HalfSpace3<vector_bits, dist_bits>& half_space) const {
    switch (aabb().GetPlaneSide(half_space)) {
      case -1:
        // All on the negative side.
        {
          SplitInfoRep result;
          result.ranges.neg_range.second = vertex_count();
          return result;
        }
      case 1:
        // All on the positive side.
        {
          SplitInfoRep result;
          result.ranges.pos_range.second = vertex_count();
          return result;
        }
      case 0:
        // Straddles the half-space.
        return Parent::GetSplitInfo(half_space);
      default:
        assert(false);
    }
  }

  // Overrides the non-virtual function from ConvexPolygon.
  std::pair<AABBConvexPolygon, AABBConvexPolygon> CreateSplitChildren(
      const SplitInfoRep& split) const {
    std::pair<AABBConvexPolygon, AABBConvexPolygon> result;
    FillInSplitChildren(*this, split, result.first, result.second);
    return result;
  }

  // Overrides the non-virtual function from ConvexPolygon.
  std::pair<AABBConvexPolygon, AABBConvexPolygon> CreateSplitChildren(
      SplitInfoRep&& split) && {
    std::pair<AABBConvexPolygon, AABBConvexPolygon> result;
    FillInSplitChildren(std::move(*this), std::move(split), result.first,
                        result.second);
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
  ConvexVertexAABBTracker<homo_point3_num_bits,
                          homo_point3_denom_bits> aabb_tracker_;
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
