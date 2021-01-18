#ifndef WALNUT_POINT3_WITH_VERTEX_DATA_H__
#define WALNUT_POINT3_WITH_VERTEX_DATA_H__

#include "walnut/no_vertex_data.h"
#include "walnut/point3.h"

namespace walnut {

// This is used by FactoryWithVertexData to construct a ConvexPolygon and
// specify the inital values of the vertex data.
template <size_t point3_bits = 32,
          typename VertexDataTemplate = NoVertexData>
struct Point3WithVertexData : public Point3<point3_bits> {
  using Parent = Point3<point3_bits>;
  using VertexData = VertexDataTemplate;

  using Parent::Parent;

  Point3WithVertexData(int x, int y, int z, VertexData data) :
    Parent(x, y, z), data(data) { }

  VertexData data;
};

// Gets the VertexData field out of Point3Rep. If Point3Rep does not have a
// VertexData field, then it defaults to NoVertexData.
template <typename Point3Rep, bool leave_as_default = true>
struct GetVertexData {
  static_assert(leave_as_default == true);
  using VertexData = NoVertexData;
};

template <typename Point3Rep>
struct GetVertexData<Point3Rep,
                     std::is_same<typename Point3Rep::VertexData,
                                  typename Point3Rep::VertexData>::value> {
  using VertexData = typename Point3Rep::VertexData;
};

}  // walnut

#endif // WALNUT_POINT3_WITH_VERTEX_DATA_H__
