#ifndef WALNUT_CONNECTED_POLYGON_H__
#define WALNUT_CONNECTED_POLYGON_H__

#include "walnut/convex_polygon.h"

namespace walnut {

// A half edge of a ConnectedPolygon
//
// Usually a half edge will only connect with one other half edge. However,
// during the process of connecting the half edge, it may be discovered that
// the half-edge connects to more half edges due to T-junctions. The vector of
// edges in the parent polygon must remain stable during the connection
// process, because inserting new edges would break existing EdgeConnections.
// So instead the extra connections are stored in `extra_neighbors_`.
template <typename HomoPoint3Template, typename FinalPolygonTemplate,
          typename ParentTemplate>
struct ConnectedEdge : public ParentTemplate {
  using HomoPoint3Rep = HomoPoint3Template;
  using FinalPolygon = FinalPolygonTemplate;
  using Parent = ParentTemplate;
  using ConnectedEdgeRep = ConnectedEdge;

  ConnectedEdge() = default;

  ConnectedEdge(const ConnectedEdge&) = default;

  ConnectedEdge(const Parent& other) : Parent(other) { }

  using Parent::Parent;

  bool operator==(const ConnectedEdge& other) const {
    if (!Parent::operator==(other)) return false;

    if (extra_neighbors_ != other.extra_neighbors_) return false;
    return neighbor_ == other.neighbor_;
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

  const ConnectedEdge* neighbor() const {
    return neighbor_;
  }

  ConnectedEdge* neighbor() {
    return neighbor_;
  }

  size_t extra_neighbor_count() const {
    return extra_neighbors_.size();
  }

  const HomoPoint3Rep& extra_neighbor_start(size_t i) const {
    return extra_neighbors_[i].start;
  }

  const ConnectedEdge& extra_neighbor(size_t i) const {
    return *extra_neighbors_[i].neighbor;
  }

  ConnectedEdge& extra_neighbor(size_t i) {
    return *extra_neighbors_[i].neighbor;
  }

  size_t edge_index() const {
    using FinalEdgeRep = typename FinalPolygon::EdgeVector::value_type;
    return static_cast<const FinalEdgeRep *>(this) - polygon().edges().data();
  }

 protected:
  ConnectedEdge(ConnectedEdge&&) = default;

  ConnectedEdge& operator=(const ConnectedEdge&) = default;
  ConnectedEdge& operator=(ConnectedEdge&&) = default;

 private:
  template <typename ParentPolygon, typename FinalPolygon, typename EdgeParent>
  friend class ConnectedPolygon;

  struct ExtraConnection {
    HomoPoint3Rep start;
    ConnectedEdge* neighbor = nullptr;
  };

  // Through the friend statement, this field is be modified by
  // ConnectedPolygon.
  FinalPolygon* polygon_ = nullptr;

  ConnectedEdge* neighbor_ = nullptr;
  std::vector<ExtraConnection> extra_neighbors_;
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
class ConnectedPolygon : public ParentTemplate::MakeParent<
                                  FinalPolygonTemplate,
                                  ConnectedEdge<
                                    typename ParentTemplate::HomoPoint3Rep,
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
  using HomoPoint3Rep = typename ParentTemplate::HomoPoint3Rep;
  using ConnectedEdgeRep = ConnectedEdge<HomoPoint3Rep, FinalPolygon,
                                         EdgeParent>;
  using Parent = typename ParentTemplate::MakeParent<FinalPolygon,
                                                     ConnectedEdgeRep>;
  using typename Parent::EdgeRep;
  using typename Parent::SplitInfoRep;

  // Subclasses can inherit from this. `NewEdgeParent` should be the subclass's
  // EdgeInfo type.
  template <typename FinalPolygon, typename NewEdgeParent>
  using MakeParent = ConnectedPolygon<Parent, FinalPolygon, NewEdgeParent>;

  using Parent::point3_bits;

  ConnectedPolygon() = default;

  template <typename OtherPolygon,
            std::enable_if_t<std::is_constructible<Parent,
                                                   OtherPolygon>::value,
                             bool> = true>
  ConnectedPolygon(OtherPolygon&& other) :
      Parent(std::forward<OtherPolygon>(other)) {
    SetEdgeBackPointers();
  }

  // Overrides the non-virtual function from ConvexPolygon.
  std::pair<ConnectedPolygon, ConnectedPolygon> CreateSplitChildren(
      const SplitInfoRep& split) const {
    std::pair<ConnectedPolygon, ConnectedPolygon> result;
    FillInSplitChildren(*this, split, result.first, result.second);
    return result;
  }

  // Overrides the non-virtual function from ConvexPolygon.
  std::pair<ConnectedPolygon, ConnectedPolygon> CreateSplitChildren(
      SplitInfoRep&& split) && {
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

  using Parent::edge;
  using Parent::vertex_count;

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

}  // walnut

#endif // WALNUT_CONNECTED_POLYGON_H__
