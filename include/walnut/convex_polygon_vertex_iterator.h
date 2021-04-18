#ifndef WALNUT_CONVEX_POLYGON_VERTEX_ITERATOR_H__
#define WALNUT_CONVEX_POLYGON_VERTEX_ITERATOR_H__

#include "walnut/homo_point3.h"
#include "walnut/transform_iterator.h"

namespace walnut {

template <typename Edge>
struct ReturnVertexFromEdge {
  const HomoPoint3& operator()(const Edge& edge) const {
    return edge.vertex();
  }
};

// An iterator that produces just the const vertex reference from an input
// iterator that produces a `ConvexPolygonEdge`.
template <typename EdgeIterator>
class ConvexPolygonVertexIterator
  : public TransformIterator<
             EdgeIterator,
             ReturnVertexFromEdge<typename EdgeIterator::value_type>> {
 public:
  using Transformer = ReturnVertexFromEdge<typename EdgeIterator::value_type>;
  using Parent = TransformIterator<EdgeIterator, Transformer>;
  using Parent::Parent;
};

}  // walnut

#endif // WALNUT_CONVEX_POLYGON_VERTEX_ITERATOR_H__
