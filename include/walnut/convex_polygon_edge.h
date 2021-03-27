#ifndef WALNUT_CONVEX_POLYGON_EDGE_H__
#define WALNUT_CONVEX_POLYGON_EDGE_H__

#include <ostream>
#include <sstream>

#include "walnut/homo_point3.h"
#include "walnut/plucker_line.h"
#include "walnut/point3.h"
#include "walnut/point3_with_vertex_data.h"

namespace walnut {

template <size_t point3_bits_template, typename ParentTemplate>
struct ConvexPolygonEdge;

template <size_t point3_bits, typename Parent>
std::string
Approximate(const ConvexPolygonEdge<point3_bits, Parent>& edge);

template <size_t point3_bits>
std::string
Approximate(const ConvexPolygonEdge<point3_bits, EdgeInfoRoot>& edge);

// An edge of a ConvexPolygon
template <size_t point3_bits_template = 32,
          typename ParentTemplate = EdgeInfoRoot>
struct ConvexPolygonEdge : public ParentTemplate {
  using Point3Rep = Point3<point3_bits_template>;
  using HomoPoint3Rep = HomoPoint3<(point3_bits_template - 1)*7 + 10,
                                   (point3_bits_template - 1)*6 + 10>;
  using LineRep = typename PluckerLineFromPlanesFromPoint3sBuilder<
    point3_bits_template>::PluckerLineRep;
  using Parent = ParentTemplate;

  ConvexPolygonEdge(const ConvexPolygonEdge&) = default;

  // Parent must be default-constructible to use this constructor.
  template <size_t other_point3_bits>
  ConvexPolygonEdge(const std::enable_if_t<
                            std::is_default_constructible<Parent>::value,
                            Point3<other_point3_bits>>& vertex,
                    const Point3<other_point3_bits>& next_vertex) :
    vertex_(vertex), line_(vertex, next_vertex) { }

  // Parent must be default-constructible to use this constructor.
  template <size_t num_bits, size_t denom_bits>
  ConvexPolygonEdge(const HomoPoint3<num_bits, denom_bits>& vertex,
                    const HomoPoint3<num_bits, denom_bits>& next_vertex) :
    vertex_(vertex), line_(vertex, next_vertex) { }

  // Parent must be default-constructible to use this constructor.
  //
  // `line` should be in the direction from `vertex` to the next vertex in the
  // polygon.
  ConvexPolygonEdge(const HomoPoint3Rep& vertex, const LineRep& line) :
    vertex_(vertex), line_(line) { }

  // Inherit the line and vertex data from `parent_edge`, but overwrite the
  // vertex.
  //
  // `vertex` must be on `parent_edge.line_`.
  ConvexPolygonEdge(const ConvexPolygonEdge& parent_edge,
                    const HomoPoint3Rep& vertex) :
    Parent(parent_edge, vertex),
    vertex_(vertex),
    line_(parent_edge.line_) { }

  // Inherit the vertex and vertex data from `parent_edge`, but overwrite the
  // line.
  //
  // `line` should be in the direction from `vertex` to the next vertex in the
  // polygon.
  ConvexPolygonEdge(const ConvexPolygonEdge& parent_edge,
                    const LineRep& line) : Parent(parent_edge, line),
                                           vertex_(parent_edge.vertex_),
                                           line_(line) { }

  // Inherit the vertex data from `parent_edge`, but overwrite the vertex and
  // line.
  //
  // `line` should be in the direction from `vertex` to the next vertex in the
  // polygon.
  ConvexPolygonEdge(const ConvexPolygonEdge& parent_edge,
                    const HomoPoint3Rep& vertex,
                    const LineRep& line) :
    Parent(parent_edge, vertex, line),
    vertex_(vertex), line_(line) { }

  template <size_t input_point3_bits = point3_bits_template>
  ConvexPolygonEdge(const Point3WithVertexData<input_point3_bits,
                                               Parent>& vertex,
                    const Point3<input_point3_bits>& next_vertex) :
    Parent(vertex.data), vertex_(vertex), line_(vertex, next_vertex) { }

  template <size_t other_point3_bits, typename OtherParent>
  explicit ConvexPolygonEdge(
      const ConvexPolygonEdge<other_point3_bits, OtherParent>& other) :
    Parent(other), vertex_(other.vertex()), line_(other.line()) { }

  static bool LexicographicallyLt(const ConvexPolygonEdge& a,
                                  const ConvexPolygonEdge& b) {
    return HomoPoint3Rep::LexicographicallyLt(a.vertex_, b.vertex_);
  }

  bool IsValidState() const {
    return vertex_.IsValidState() && line_.IsValidState() &&
           line_.IsValid() && line_.IsCoincident(vertex_);
  }

  // Return a string representation of the edge that uses decimal points to
  // approximate the vertex coordinates.
  std::string Approximate() const {
    return walnut::Approximate(*this);
  }

  // Return a string representation of the edge that uses decimal points to
  // approximate the vertex coordinates and does not include the data fields.
  std::string ApproximateNoData() const {
    std::ostringstream out;
    out << vertex_.Approximate();
    return out.str();
  }

  const LineRep& line() const {
    return line_;
  }

  const HomoPoint3Rep& vertex() const {
    return vertex_;
  }

 protected:
  // In order to make a std::vector of edges, the vector needs access to the
  // assignment operator. So to give the vector access, wrap the edge with an
  // `AssignableWrapper`.

  ConvexPolygonEdge(ConvexPolygonEdge&&) = default;

  ConvexPolygonEdge& operator=(const ConvexPolygonEdge&) = default;
  ConvexPolygonEdge& operator=(ConvexPolygonEdge&&) = default;

  template <size_t other_point3_bits, typename OtherParent>
  ConvexPolygonEdge& operator=(const ConvexPolygonEdge<other_point3_bits,
                                                       OtherParent>& other) {
    Parent::operator=(other);
    vertex_ = other.vertex();
    line_ = other.line();
    return *this;
  }

 private:
  HomoPoint3Rep vertex_;

  // This line starts at `vertex` and goes to the next vertex in the polygon.
  //
  // Notably:
  //   next_vertex == vertex + line.d()
  LineRep line_;
};

template <size_t point3_bits, typename Parent>
std::string
Approximate(const ConvexPolygonEdge<point3_bits, Parent>& edge) {
  std::ostringstream out;
  out << edge.vertex().Approximate() << ": "
      << static_cast<const Parent&>(edge);
  return out.str();
}

template <size_t point3_bits>
std::string
Approximate(const ConvexPolygonEdge<point3_bits, EdgeInfoRoot>& edge) {
  return edge.ApproximateNoData();
}

template <size_t point3_bits, typename Parent>
std::ostream& operator<<(
    std::ostream& out, const ConvexPolygonEdge<point3_bits, Parent>& edge) {
  out << edge.vertex() << ": " << static_cast<const Parent&>(edge);
  return out;
}

template <size_t point3_bits>
std::ostream& operator<<(
    std::ostream& out,
    const ConvexPolygonEdge<point3_bits, EdgeInfoRoot>& edge) {
  out << edge.vertex();
  return out;
}

}  // walnut

#endif // WALNUT_CONVEX_POLYGON_EDGE_H__
