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

  // Exposes the protected mutating functions
  using Parent::operator=;
  using Parent::edge;
  using Parent::SortVertices;
  using Parent::CreateSplitChildren;
};

}  // walnut

#endif // WALNUT_MUTABLE_CONVEX_POLYGON_H__
