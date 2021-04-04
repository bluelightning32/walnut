#ifndef WALNUT_CONNECTED_POLYGON_H__
#define WALNUT_CONNECTED_POLYGON_H__

#include "gtest/gtest_prod.h"
#include "walnut/convex_polygon.h"
#include "walnut/deed.h"

namespace walnut {

// A half edge of a ConnectedPolygon
//
// The DeedObject is used to keep a pointer to the edges while sorting and
// connecting them.
template <typename FinalPolygonTemplate,
          typename ParentTemplate>
struct ConnectedEdge : public ParentTemplate, public DeedObject {
  using FinalPolygon = FinalPolygonTemplate;
  using Parent = ParentTemplate;
  using ConnectedEdgeRep = ConnectedEdge;

  ConnectedEdge() = default;

  ConnectedEdge(const ConnectedEdge&) = default;

  ConnectedEdge(const Parent& other) : Parent(other) { }

  ConnectedEdge(const ConnectedEdge& parent_edge, const HomoPoint3& vertex) :
    Parent(parent_edge, vertex),
    polygon_(parent_edge.polygon_) { }

  using Parent::Parent;

  bool operator==(const ConnectedEdge& other) const {
    if (!Parent::operator==(other)) return false;

    return partner_ == other.partner_;
  }

  // Allow the parent's operator== to be used for types that inherit from
  // the parent type, but not ConnectedEdge.
  using Parent::operator==;

  const FinalPolygon& polygon() const {
    return *polygon_;
  }

  FinalPolygon& polygon() {
    return *polygon_;
  }

  const ConnectedEdge* partner() const {
    return partner_;
  }

  ConnectedEdge* partner() {
    return partner_;
  }

  size_t edge_index() const {
    using FinalEdgeRep = typename FinalPolygon::EdgeVector::value_type;
    return static_cast<const FinalEdgeRep *>(this) - polygon().edges().data();
  }

  // Returns true if this edge is positive in the `sorted_dimension` component.
  bool IsPositive(int sorted_dimension) const {
    using FinalEdgeRep = typename FinalPolygon::EdgeRep;
    const FinalEdgeRep& final_this = static_cast<const FinalEdgeRep&>(*this);
    return final_this.line().d().components()[sorted_dimension].GetSign() >= 0;
  }

  // Returns the start vertex of the next edge in this edges's polygon.
  const HomoPoint3& next_vertex() const {
    using FinalEdgeRep = typename FinalPolygon::EdgeRep;
    const FinalEdgeRep& next_edge =
      polygon().edge((edge_index() + 1) % polygon().vertex_count());
    return next_edge.vertex();
  }

  // Returns the begin location of this edge as defined by the
  // `EdgeLineConnector`.
  const HomoPoint3& GetBeginLocation(int sorted_dimension) const {
    if (IsPositive(sorted_dimension)) {
      // This is a positive edge. It starts at the start point of the edge.
      using FinalEdgeRep = typename FinalPolygon::EdgeRep;
      const FinalEdgeRep& final_this = static_cast<const FinalEdgeRep&>(*this);
      return final_this.vertex();
    } else {
      // This is a negative edge. It starts at the start of the next edge.
      return next_vertex();
    }
  }

  // Returns the end location of this edge as defined by the
  // `EdgeLineConnector`.
  const HomoPoint3& GetEndLocation(int sorted_dimension) const {
    if (IsPositive(sorted_dimension)) {
      // This is a positive edge. It ends at the start of the next edge.
      return next_vertex();
    } else {
      // This is a negative edge. It ends at the start point of the edge.
      using FinalEdgeRep = typename FinalPolygon::EdgeRep;
      const FinalEdgeRep& final_this = static_cast<const FinalEdgeRep&>(*this);
      return final_this.vertex();
    }
  }

 protected:
  ConnectedEdge(ConnectedEdge&& other) :
      Parent(std::move(other)), DeedObject(std::move(other)),
      polygon_(other.polygon_), partner_(other.partner_) {
    if (partner_ != nullptr) {
      partner_->partner_ = this;
      other.partner_ = nullptr;
    }
  }

  ConnectedEdge& operator=(const ConnectedEdge&) = default;

  ConnectedEdge& operator=(ConnectedEdge&& other) {
    static_cast<Parent&>(*this) = std::move(other);
    static_cast<DeedObject&>(*this) = std::move(other);

    std::swap(partner_, other.partner_);
    polygon_ = other.polygon_;
    if (partner_ != nullptr) {
      partner_->partner_ = this;
    }
    if (other.partner_ != nullptr) {
      other.partner_->partner_ = &other;
    }
    return *this;
  }

 private:
  template <typename ParentPolygon, typename FinalPolygon, typename EdgeParent>
  friend class ConnectedPolygon;
  template <typename EdgeTemplate>
  friend class EdgeLineConnector;

  FRIEND_TEST(ConnectedEdge, MoveAssign);
  FRIEND_TEST(ConnectedEdge, MoveConstruct);
  FRIEND_TEST(ConnectedPolygon, SplitEdge);

  void ResetPartners() {
    partner_ = nullptr;
  }

  // Through the friend statement, this field is be modified by
  // ConnectedPolygon.
  FinalPolygon* polygon_ = nullptr;

  ConnectedEdge* partner_ = nullptr;
};


// This convex polygon is part of a doubly connected edge list. In addition to
// how half edges within the same polygon are implicitly connected together, a
// half edge in this polygon is also connected to the half edge of the adjacent
// polygon.
//
// The half edges start disconnected. They must be explicitly connected.
//
// `ParentTemplate` must inherit from `ConvexPolygon`. The
// `FinalPolygonTemplate` and `EdgeParentTemplate` parameters are used
// internally. The default values should be used when externally referring to a
// ConnectedPolygon.
template <typename ParentTemplate = ConvexPolygon<>,
          typename FinalPolygonTemplate = void,
          typename EdgeParentTemplate = EdgeInfoRoot>
class ConnectedPolygon : public ParentTemplate::template MakeParent<
                                  FinalPolygonTemplate,
                                  ConnectedEdge<
                                    std::conditional_t<
                                      std::is_void<
                                        FinalPolygonTemplate
                                      >::value,
                                      ConnectedPolygon<
                                        ParentTemplate, FinalPolygonTemplate,
                                        EdgeParentTemplate
                                      >,
                                      FinalPolygonTemplate
                                    >,
                                    EdgeParentTemplate
                                  >
                                > {
 public:
  // If `FinalPolygonTemplate` is not void, use it as the final polygon type,
  // otherwise use `ConnectedPolygon`.
  using FinalPolygon =
    std::conditional_t<std::is_void<FinalPolygonTemplate>::value,
                       ConnectedPolygon, FinalPolygonTemplate>;
  using EdgeParent = EdgeParentTemplate;
  using ConnectedEdgeRep = ConnectedEdge<FinalPolygon, EdgeParent>;
  using Parent =
    typename ParentTemplate::template MakeParent<FinalPolygon,
                                                 ConnectedEdgeRep>;
  using typename Parent::EdgeRep;

  // Subclasses can inherit from this. `NewEdgeParent` should be the subclass's
  // EdgeInfo type.
  template <typename FinalPolygon, typename NewEdgeParent>
  using MakeParent = ConnectedPolygon<Parent, FinalPolygon, NewEdgeParent>;

  ConnectedPolygon() = default;

  template <typename OtherPolygon,
            std::enable_if_t<std::is_constructible<Parent,
                                                   OtherPolygon>::value,
                             bool> = true>
  ConnectedPolygon(OtherPolygon&& other) :
      Parent(std::forward<OtherPolygon>(other)) {
    SetEdgeBackPointers();
  }

  ConnectedPolygon(const ConnectedPolygon& other) : Parent(other) {
    SetEdgeBackPointers();
  }

  ConnectedPolygon(ConnectedPolygon&& other) : Parent(std::move(other)) {
    SetEdgeBackPointers();
  }

  ConnectedPolygon(const HalfSpace3& plane, int drop_dimension,
                   const std::vector<HomoPoint3>& vertices) :
      Parent(plane, drop_dimension, vertices) {
    SetEdgeBackPointers();
  }


  // Overrides the non-virtual function from ConvexPolygon.
  std::pair<ConnectedPolygon, ConnectedPolygon> CreateSplitChildren(
      const ConvexPolygonSplitInfo& split) const {
    std::pair<ConnectedPolygon, ConnectedPolygon> result;
    FillInSplitChildren(*this, split, result.first, result.second);
    return result;
  }

  // Overrides the non-virtual function from ConvexPolygon.
  std::pair<ConnectedPolygon, ConnectedPolygon> CreateSplitChildren(
      ConvexPolygonSplitInfo&& split) && {
    std::pair<ConnectedPolygon, ConnectedPolygon> result;
    FillInSplitChildren(std::move(*this), std::move(split), result.first,
                        result.second);
    return result;
  }

  bool IsValidState() const {
    if (!Parent::IsValidState()) {
      return false;
    }
    for (size_t i = 0; i < vertex_count(); ++i) {
      if (&edge(i).polygon() != this) {
        return false;
      }
    }
    return true;
  }

  // The edges must be mutable in order to connect them. So expose both the
  // const and non-const edge functions from the parent.
  using Parent::edge;
  using Parent::vertex_count;
  using Parent::SplitEdge;

 protected:
  template <typename ParentRef, typename SplitInfoRef>
  static void FillInSplitChildren(ParentRef&& parent, SplitInfoRef&& split,
                                  ConnectedPolygon& neg_child,
                                  ConnectedPolygon& pos_child) {
    Parent::FillInSplitChildren(std::forward<ParentRef>(parent),
                                std::forward<SplitInfoRef>(split), neg_child,
                                pos_child);
    neg_child.SetEdgeBackPointers();
    pos_child.SetEdgeBackPointers();
  }

  template <typename OtherPolygon>
  std::enable_if_t<std::is_assignable<Parent,
                                      OtherPolygon>::value,
                   ConnectedPolygon&>
  operator=(OtherPolygon&& other) {
    Parent::operator=(std::forward<OtherPolygon>(other));
    SetEdgeBackPointers();
    return *this;
  }

 private:
  void SetEdgeBackPointers() {
    for (size_t i = 0; i < vertex_count(); ++i) {
      edge(i).polygon_ = this;
    }
  }
};

template <typename FinalPolygon, typename Parent>
inline std::ostream& operator<<(std::ostream& out,
                                const ConnectedEdge<FinalPolygon,
                                                    Parent>& edge) {
  out << "polygon=" << &edge.polygon() << " partner=" << edge.partner();
  return out;
}

}  // walnut

#endif // WALNUT_CONNECTED_POLYGON_H__
