#ifndef WALNUT_CONVEX_POLYGON_EDGE_H__
#define WALNUT_CONVEX_POLYGON_EDGE_H__

#include <ostream>

#include "walnut/homo_point3.h"
#include "walnut/plucker_line.h"
#include "walnut/point3.h"
#include "walnut/point3_with_vertex_data.h"

namespace walnut {

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

  // VertexData must be default-constructible to use this constructor.
  template <int other_point3_bits>
  ConvexPolygonEdge(const Point3<other_point3_bits>& vertex,
                    const Point3<other_point3_bits>& next_vertex) :
    vertex(vertex), line(vertex, next_vertex) { }

  // VertexData must be default-constructible to use this constructor.
  template <int num_bits, int denom_bits>
  ConvexPolygonEdge(const HomoPoint3<num_bits, denom_bits>& vertex,
                    const HomoPoint3<num_bits, denom_bits>& next_vertex) :
    vertex(vertex), line(vertex, next_vertex) { }

  // VertexData must be default-constructible to use this constructor.
  //
  // `line` should be in the direction from `vertex` to the next vertex in the
  // polygon.
  ConvexPolygonEdge(const HomoPoint3Rep& vertex, const LineRep& line) :
    vertex(vertex), line(line) { }

  template <int input_point3_bits = point3_bits_template>
  ConvexPolygonEdge(const Point3WithVertexData<input_point3_bits,
                                               VertexData>& vertex,
                    const Point3<input_point3_bits>& next_vertex) :
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

  bool IsValidState() const {
    return vertex.IsValidState() && line.IsValidState() &&
           line.IsCoincident(vertex);
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
