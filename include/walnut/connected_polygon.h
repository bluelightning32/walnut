#ifndef WALNUT_CONNECTED_POLYGON_H__
#define WALNUT_CONNECTED_POLYGON_H__

#include "walnut/convex_polygon.h"

namespace walnut {

template <typename FinalPolygon>
struct EdgeConnection {
  FinalPolygon* polygon = nullptr;
  size_t edge_index;
};


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
  using EdgeConnectionRep = EdgeConnection<FinalPolygon>;

  struct ExtraConnection {
    HomoPoint3Rep start;
    EdgeConnectionRep neighbor;
  };

  ConnectedEdge() = default;

  ConnectedEdge(const ConnectedEdge&) = default;

  using Parent::Parent;

  bool operator==(const ConnectedEdge& other) const {
    if (!Parent::operator==(other)) return false;

    if (extra_neighbors_ != other.extra_neighbors_) return false;
    return neighbor_ == other.neighbor_;
  }

  // Allow the parent's operator== to be used for types that inherit from
  // the parent type, but not ConnectedEdge.
  using Parent::operator==;

  const EdgeConnectionRep& neighbor() const {
    return neighbor_;
  }

  const std::vector<ExtraConnection>& extra_neighbors() const {
    return extra_neighbors_;
  }

 protected:
  ConnectedEdge(ConnectedEdge&&) = default;

  ConnectedEdge& operator=(const ConnectedEdge&) = default;
  ConnectedEdge& operator=(ConnectedEdge&&) = default;

 private:
  EdgeConnectionRep neighbor_;
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

  using Parent::Parent;

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

 protected:
  using Parent::FillInSplitChildren;
};

}  // walnut

#endif // WALNUT_CONNECTED_POLYGON_H__
