#ifndef WALNUT_POINT3_WITH_VERTEX_DATA_H__
#define WALNUT_POINT3_WITH_VERTEX_DATA_H__

#include "walnut/point3.h"

namespace walnut {

struct NoVertexData {
  constexpr NoVertexData() = default;
  template <typename Other>
  constexpr explicit NoVertexData(const Other&) { }

  template <typename Other>
  constexpr bool operator!=(const Other& other) const {
    return false;
  }

  template <typename Other>
  constexpr NoVertexData& operator=(const Other& other) {
    return *this;
  }
};

// This is used by FactoryWithVertexData to construct a ConvexPolygon and
// specify the inital values of the vertex data.
template <int point3_bits = 32,
          typename VertexDataTemplate = NoVertexData>
struct Point3WithVertexData : public Point3<point3_bits> {
  using Parent = Point3<point3_bits>;
  using VertexData = VertexDataTemplate;

  using Parent::Parent;

  Point3WithVertexData(int x, int y, int z, VertexData data) :
    Parent(x, y, z), data(data) { }

  VertexData data;
};

}  // walnut

#endif // WALNUT_POINT3_WITH_VERTEX_DATA_H__
