#ifndef WALNUT_CONVEX_POLYGON_EDGE_H__
#define WALNUT_CONVEX_POLYGON_EDGE_H__

#include <ostream>
#include <sstream>

#include "walnut/assignable_wrapper.h"
#include "walnut/homo_point3.h"
#include "walnut/plucker_line.h"
#include "walnut/point3.h"
#include "walnut/point3_with_vertex_data.h"

namespace walnut {

template <typename ParentTemplate>
struct ConvexPolygonEdge;

template <typename Parent>
std::string Approximate(const ConvexPolygonEdge<Parent>& edge);

// An edge of a ConvexPolygon
template <typename ParentTemplate = EdgeInfoRoot>
struct ConvexPolygonEdge : public ParentTemplate {
  using Parent = ParentTemplate;

  ConvexPolygonEdge(const ConvexPolygonEdge&) = default;

  // Leave the move constructor implicitly deleted. Explicitly deleting the
  // move constructor would prevent the compiler from falling back to the copy
  // constructor.
  //
  // ConvexPolygonEdge(ConvexPolygonEdge&&) = delete;

  ConvexPolygonEdge(RValueKey<ConvexPolygonEdge> other)
    noexcept(std::is_nothrow_constructible<Parent, RValueKey<Parent>>::value)
    : Parent(RValueKey<Parent>(other)),
      vertex_(std::move(other.get().vertex_)),
      line_(std::move(other.get().line_)) { }

  // Parent must be default-constructible to use this constructor.
  ConvexPolygonEdge(const Point3& vertex, const Point3& next_vertex) :
    vertex_(vertex), line_(vertex, next_vertex) { }

  // Parent must be default-constructible to use this constructor.
  ConvexPolygonEdge(const HomoPoint3& vertex, const HomoPoint3& next_vertex) :
      vertex_(vertex), line_(vertex, next_vertex) {
    assert(line_.IsValid());
  }

  // Parent must be default-constructible to use this constructor.
  ConvexPolygonEdge(HomoPoint3&& vertex, const HomoPoint3& next_vertex) :
      vertex_(std::move(vertex)), line_(vertex_, next_vertex) {
    assert(line_.IsValid());
  }

  // Parent must be default-constructible to use this constructor.
  //
  // `line` should be in the direction from `vertex` to the next vertex in the
  // polygon.
  ConvexPolygonEdge(HomoPoint3&& vertex, PluckerLine&& line)
    : vertex_(std::move(vertex)), line_(std::move(line)) {
    assert(line_.IsValid());
  }

  // Parent must be default-constructible to use this constructor.
  //
  // `line` should be in the direction from `vertex` to the next vertex in the
  // polygon.
  ConvexPolygonEdge(const HomoPoint3& vertex, const PluckerLine& line) :
    vertex_(vertex), line_(line) {
    assert(line_.IsValid());
  }

  // Inherit the line and vertex data from `parent_edge`, but overwrite the
  // vertex.
  //
  // `vertex` must be on `parent_edge.line_`.
  ConvexPolygonEdge(const ConvexPolygonEdge& parent_edge,
                    const HomoPoint3& vertex) :
    Parent(parent_edge, vertex),
    vertex_(vertex),
    line_(parent_edge.line_) { }

  // Inherit the vertex and vertex data from `parent_edge`, but overwrite the
  // line.
  //
  // `line` should be in the direction from `vertex` to the next vertex in the
  // polygon.
  ConvexPolygonEdge(const ConvexPolygonEdge& parent_edge,
                    const PluckerLine& line) : Parent(parent_edge, line),
                                               vertex_(parent_edge.vertex_),
                                               line_(line) { }

  // Inherit the vertex data from `parent_edge`, but overwrite the vertex and
  // line.
  //
  // `line` should be in the direction from `vertex` to the next vertex in the
  // polygon.
  ConvexPolygonEdge(const ConvexPolygonEdge& parent_edge,
                    const HomoPoint3& vertex, const PluckerLine& line) :
    Parent(parent_edge, vertex, line),
    vertex_(vertex), line_(line) { }

  ConvexPolygonEdge(const Point3WithVertexData<Parent>& vertex,
                    const Point3& next_vertex) :
    ConvexPolygonEdge(vertex, PluckerLine(vertex, next_vertex)) { }

  ConvexPolygonEdge(const Point3WithVertexData<Parent>& vertex,
                    PluckerLine line)
    : Parent(vertex.data), vertex_(vertex), line_(std::move(line)) {
    assert(line_.IsValid());
  }

  template <typename OtherParent>
  explicit ConvexPolygonEdge(const ConvexPolygonEdge<OtherParent>& other) :
    Parent(other), vertex_(other.vertex()), line_(other.line()) { }

  static bool LexicographicallyLt(const ConvexPolygonEdge& a,
                                  const ConvexPolygonEdge& b) {
    return HomoPoint3::LexicographicallyLt(a.vertex_, b.vertex_);
  }

  bool IsValidState() const {
    return vertex_.IsValid() && line_.IsValid() && line_.IsCoincident(vertex_);
  }

  // Print a string representation of the edge that uses decimal points to
  // approximate the vertex coordinates.
  std::ostream& Approximate(std::ostream& out) const {
    return Parent::Approximate(vertex().Approximate(out));
  }

  const PluckerLine& line() const {
    return line_;
  }

  const HomoPoint3& vertex() const {
    return vertex_;
  }

  // Returns true if this edge can be merged (the next removed).
  //
  // The caller must ensure that `next` is the next edge in the same polygon,
  // that this edge is on the same line, and that both edges point in the same
  // direction.
  bool CanMerge(const ConvexPolygonEdge& next) const {
    assert(line().d().IsSameDir(next.line().d()));
    assert(line().IsCoincident(next.vertex_));
    return Parent::CanMerge(next);
  }

 protected:
  // In order to make a std::vector of edges, the vector needs access to the
  // assignment operator. So to give the vector access, wrap the edge with an
  // `AssignableWrapper`.

  ConvexPolygonEdge& operator=(const ConvexPolygonEdge&) = default;

  // Leave the move assignment operator implicitly deleted. Explicitly deleting
  // the move assignment operator would prevent the compiler from falling back
  // to the copy assignment operator.
  //
  // ConvexPolygonEdge& operator=(ConvexPolygonEdge&&) = delete;

  ConvexPolygonEdge& operator=(RValueKey<ConvexPolygonEdge> other)
      noexcept(std::is_nothrow_move_assignable<
                 AssignableWrapper<Parent>
               >::value) {
    Parent::operator=(RValueKey<Parent>(other));
    vertex_ = std::move(other.get().vertex_);
    line_ = std::move(other.get().line_);
    return *this;
  }

  template <typename OtherParent>
  ConvexPolygonEdge& operator=(const ConvexPolygonEdge<OtherParent>& other) {
    Parent::operator=(other);
    vertex_ = other.vertex();
    line_ = other.line();
    return *this;
  }

  RValueKey<ConvexPolygonEdge> GetRValueKey() && {
    return RValueKey<ConvexPolygonEdge>(std::move(*this));
  }

  // Merges the data in this edge with the data in `next`.
  //
  // This may only be called after CanMerge returns true. Afterwards merging
  // the edges, the caller should remove `next` from the polygon.
  //
  // Even though this is protected, it is called by ConvexPolygon.
  void Merge(ConvexPolygonEdge& next) {
    assert(CanMerge(next));
    return Parent::Merge(next);
  }

  template <typename EdgeParent>
  void EdgeMoved(ConvexPolygon<EdgeParent>& target) {
    Parent::EdgeMoved(target);
  }

 private:
  template <typename EdgeParent>
  friend class ConvexPolygon;

  HomoPoint3 vertex_;

  // This line starts at `vertex` and goes to the next vertex in the polygon.
  //
  // Notably:
  //   next_vertex == vertex + line.d()
  PluckerLine line_;
};

template <typename Parent>
std::string Approximate(const ConvexPolygonEdge<Parent>& edge) {
  std::ostringstream out;
  out << edge.Approximate(out);
  return out.str();
}

template <typename Parent>
std::ostream& operator<<(
    std::ostream& out, const ConvexPolygonEdge<Parent>& edge) {
  out << edge.vertex() << ": " << static_cast<const Parent&>(edge);
  return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const ConvexPolygonEdge<EdgeInfoRoot>& edge) {
  out << edge.vertex();
  return out;
}

}  // walnut

#endif // WALNUT_CONVEX_POLYGON_EDGE_H__
