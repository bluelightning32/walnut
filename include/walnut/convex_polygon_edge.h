#ifndef WALNUT_CONVEX_POLYGON_EDGE_H__
#define WALNUT_CONVEX_POLYGON_EDGE_H__

#include <ostream>

#include "walnut/homo_point3.h"
#include "walnut/plucker_line.h"
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

// An edge of a ConvexPolygon
template <int point3_bits_template = 32,
          typename VertexDataTemplate = NoVertexData>
struct ConvexPolygonEdge : private VertexDataTemplate {
  using Point3Rep = Point3<point3_bits_template>;
  using HomoPoint3Rep = HomoPoint3<(point3_bits_template - 1)*7 + 10,
                                   (point3_bits_template - 1)*6 + 10>;
  using LineRep = typename PluckerLineFromPlanesFromPoint3sBuilder<
    point3_bits_template>::PluckerLineRep;
  using VertexData = VertexDataTemplate;

  // This is used by FactoryWithVertexData to construct a ConvexPolygon and
  // specify the inital values of the vertex data.
  struct Point3WithVertexData : public Point3Rep {
    using Point3Rep::Point3Rep;

    Point3WithVertexData(int x, int y, int z, VertexData data) :
      Point3Rep(x, y, z), data(data) { }

    VertexData data;
  };

  // VertexData must be default-constructible to use this constructor.
  template <int other_point3_bits>
  ConvexPolygonEdge(const Point3<other_point3_bits>& vertex,
                    const Point3<other_point3_bits>& next_vertex) :
    vertex(vertex), line(vertex, next_vertex) { }

  // VertexData must be default-constructible to use this constructor.
  //
  // `line` should be in the direction from `vertex` to the next vertex in the
  // polygon.
  ConvexPolygonEdge(const HomoPoint3Rep& vertex, const LineRep& line) :
    vertex(vertex), line(line) { }

  ConvexPolygonEdge(const Point3WithVertexData& vertex,
                    const Point3WithVertexData& next_vertex) :
    VertexData(vertex.data), vertex(vertex), line(vertex, next_vertex) { }

  template <int other_point3_bits, typename OtherVertexData>
  explicit ConvexPolygonEdge(
      const ConvexPolygonEdge<other_point3_bits,
                              OtherVertexData>& other) :
    VertexData(other.data()), vertex(other.vertex), line(other.line) { }

  template <int other_point3_bits, typename OtherVertexData>
  ConvexPolygonEdge& operator=(
      const ConvexPolygonEdge<other_point3_bits,
                              OtherVertexData>& other) {
    vertex = other.vertex;
    line = other.line;
    data() = other.data();
    return *this;
  }

  static bool LexicographicallyLt(const ConvexPolygonEdge& a,
                                  const ConvexPolygonEdge& b) {
    return HomoPoint3Rep::LexicographicallyLt(a.vertex, b.vertex);
  }

  VertexData& data() {
    return *this;
  }

  const VertexData& data() const {
    return *this;
  }

  HomoPoint3Rep vertex;
  // This line starts at `vertex` and goes to the next vertex in the polygon.
  //
  // Notably:
  //   next_vertex == vertex + line.d()
  LineRep line;
};

template <int point3_bits, typename VertexData>
std::ostream& operator<<(
    std::ostream& out, const ConvexPolygonEdge<point3_bits, VertexData>& edge) {
  out << edge.vertex << ": " << edge.data();
  return out;
}

template <int point3_bits>
std::ostream& operator<<(
    std::ostream& out,
    const ConvexPolygonEdge<point3_bits, NoVertexData>& edge) {
  out << edge.vertex;
  return out;
}

}  // walnut

#endif // WALNUT_CONVEX_POLYGON_EDGE_H__
