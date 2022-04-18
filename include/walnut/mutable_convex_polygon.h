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
  using EdgeVector = typename Parent::EdgeVector;

  MutableConvexPolygon() {}

  MutableConvexPolygon(const Parent& other) : Parent(other) {}

  MutableConvexPolygon(const MutableConvexPolygon&) = default;

  MutableConvexPolygon(RValueKey<Parent> other)
      : Parent(other){}

  // `EdgeParent` must be constructible from `OtherEdgeParent`.
  template <typename OtherEdgeParent>
  explicit MutableConvexPolygon(const ConvexPolygon<OtherEdgeParent>& other)
      : Parent(other){}

  MutableConvexPolygon(const HalfSpace3& plane, int drop_dimension,
                       EdgeVector edges)
      : Parent(plane, drop_dimension, edges) {}

  MutableConvexPolygon(const HalfSpace3& plane, int drop_dimension,
                       const std::vector<Point3>& vertices)
      : Parent(plane, drop_dimension, vertices) {}

  MutableConvexPolygon(const HalfSpace3& plane, int drop_dimension,
                       const std::vector<HomoPoint3>& vertices)
      : Parent(plane, drop_dimension, vertices) {}

  template <size_t n>
  MutableConvexPolygon(HalfSpace3&& plane, int drop_dimension,
                       HomoPoint3(&&vertices)[n])
      : Parent(std::move(plane), drop_dimension, std::move(vertices)) {}

  MutableConvexPolygon(MutableConvexPolygon&& other)
    noexcept(
        std::is_nothrow_constructible<
          MutableConvexPolygon, RValueKey<MutableConvexPolygon>
        >::value)
    : MutableConvexPolygon(RValueKey<MutableConvexPolygon>(std::move(other))) {
  }

  MutableConvexPolygon(RValueKey<MutableConvexPolygon> other)
    noexcept(std::is_nothrow_constructible<Parent, RValueKey<Parent>>::value)
    : Parent(RValueKey<Parent>(other)) { }

  MutableConvexPolygon& operator=(const MutableConvexPolygon& other) = default;

  MutableConvexPolygon& operator=(RValueKey<MutableConvexPolygon> other) {
    *this = RValueKey<Parent>(other);
    return *this;
  }

  MutableConvexPolygon& operator=(MutableConvexPolygon&& other) {
    return *this = std::move(other).GetRValueKey();
  }

  // Exposes the protected mutating functions
  using Parent::operator=;
  using Parent::edge;
  using Parent::SortVertices;
  using Parent::RotateEdges;
  using Parent::CreateSplitChildren;
  using Parent::SplitEdge;
  using Parent::TryMergePolygon;
  using Parent::Invert;

  RValueKey<MutableConvexPolygon> GetRValueKey() && {
    return RValueKey<MutableConvexPolygon>(std::move(*this));
  }
};

}  // walnut

#endif // WALNUT_MUTABLE_CONVEX_POLYGON_H__
