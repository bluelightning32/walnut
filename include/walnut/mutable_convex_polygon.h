#ifndef WALNUT_MUTABLE_CONVEX_POLYGON_H__
#define WALNUT_MUTABLE_CONVEX_POLYGON_H__

#include "walnut/convex_polygon.h"

namespace walnut {

// This derived class exposes some functions from `ConvexPolygon` to do minor
// mutations.
template <typename EdgeParent = EdgeInfoRoot>
class MutableConvexPolygon : public ConvexPolygon<EdgeParent> {
 public:
  using Parent = ConvexPolygon<EdgeParent>;

  using Parent::Parent;

  // This constructor is not automatically inherited by the above using
  // statement, because its argument is the same as the parent type.
  MutableConvexPolygon(const Parent& other) : Parent(other) { }

  MutableConvexPolygon(const MutableConvexPolygon&) = default;

  MutableConvexPolygon(MutableConvexPolygon&& other)
    : Parent(std::move(other).GetRValueKey()) { }

  MutableConvexPolygon& operator=(const MutableConvexPolygon& other) = default;

  MutableConvexPolygon& operator=(MutableConvexPolygon&& other) {
    *this = std::move(other).GetRValueKey();
    return *this;
  }

  // Exposes the protected mutating functions
  using Parent::operator=;
  using Parent::edge;
  using Parent::SortVertices;
  using Parent::RotateEdges;
  using Parent::CreateSplitChildren;
  using Parent::SplitEdge;
  using Parent::TryMergePolygon;

  RValueKey<MutableConvexPolygon> GetRValueKey() && {
    return RValueKey<MutableConvexPolygon>(std::move(*this));
  }
};

}  // walnut

#endif // WALNUT_MUTABLE_CONVEX_POLYGON_H__
