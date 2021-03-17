#ifndef WALNUT_CONNECTED_POLYGON_H__
#define WALNUT_CONNECTED_POLYGON_H__

#include "gtest/gtest_prod.h"
#include "walnut/convex_polygon.h"

namespace walnut {

// A half edge of a ConnectedPolygon
//
// Usually a half edge will only connect with one other half edge. However,
// during the process of connecting the half edge, it may be discovered that
// the half-edge connects to more half edges due to T-junctions. The vector of
// edges in the parent polygon must remain stable during the connection
// process, because inserting new edges would break existing EdgeConnections.
// So instead the extra connections are stored in `extra_partners_`.
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

    if (extra_partners_ != other.extra_partners_) return false;
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

  size_t extra_partner_count() const {
    return extra_partners_.size();
  }

  const HomoPoint3Rep& extra_partner_start(size_t i) const {
    return extra_partners_[i].start;
  }

  const ConnectedEdge* extra_partner(size_t i) const {
    return extra_partners_[i].partner;
  }

  ConnectedEdge* extra_partner(size_t i) {
    return extra_partners_[i].partner;
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
  template <typename EdgeTemplate>
  friend class EdgeLineConnector;

  FRIEND_TEST(ConnectedEdge, ReversePartnerList);

  struct ExtraConnection {
    ExtraConnection() = default;
    ExtraConnection(const ExtraConnection&) = default;
    ExtraConnection(ExtraConnection&&) = default;

    ExtraConnection(const HomoPoint3Rep& start, ConnectedEdge* partner) :
      start(start), partner(partner) { }

    HomoPoint3Rep start;
    ConnectedEdge* partner = nullptr;
  };

  using ExtraConnectionIterator =
    typename std::vector<ExtraConnection>::iterator;

  void ResetPartners() {
    partner_ = nullptr;
    extra_partners_.clear();
  }

  void ReversePartnerList() {
    if (!extra_partners_.empty()) {
      // First reverse the partner pointers.
      using std::swap;
      ExtraConnectionIterator begin = extra_partners_.begin();
      ExtraConnectionIterator end = extra_partners_.end();
      --end;
      swap(partner_, end->partner);
      while (begin != end) {
        --end;
        swap(begin->partner, end->partner);
        if (begin == end) break;
        ++begin;
      }

      // Now reverse the locations.
      begin = extra_partners_.begin();
      end = extra_partners_.end();
      while (begin != end) {
        --end;
        swap(begin->start, end->start);
        if (begin == end) break;
        ++begin;
      }
    }
  }

  // Through the friend statement, this field is be modified by
  // ConnectedPolygon.
  FinalPolygon* polygon_ = nullptr;

  ConnectedEdge* partner_ = nullptr;
  std::vector<ExtraConnection> extra_partners_;
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
  using Parent =
    typename ParentTemplate::template MakeParent<FinalPolygon,
                                                 ConnectedEdgeRep>;
  using typename Parent::EdgeRep;
  using typename Parent::SplitInfoRep;
  using typename Parent::HalfSpace3Rep;

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

  template <size_t num_bits, size_t denom_bits>
  ConnectedPolygon(const HalfSpace3Rep& plane, int drop_dimension,
                   const std::vector<HomoPoint3<num_bits,
                                                denom_bits>>& vertices) :
      Parent(plane, drop_dimension, vertices) {
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

  // The edges must be mutable in order to connect them. So expose both the
  // const and non-const edge functions from the parent.
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

template <typename HomoPoint3Rep, typename FinalPolygon,
          typename Parent>
std::ostream& operator<<(std::ostream& out,
                         const ConnectedEdge<HomoPoint3Rep, FinalPolygon,
                                             Parent>& edge) {
  out << "polygon=" << &edge.polygon() << " partner=" << edge.partner();
  if (edge.extra_partner_count()) {
    out << " [ ";
    for (size_t i = 0; i < edge.extra_partner_count(); ++i) {
      if (i > 0) out << ", ";
      out << edge.extra_partner_start(i) << "=" << edge.extra_partner(i);
    }
    out << " ]";
  }
  return out;
}

}  // walnut

#endif // WALNUT_CONNECTED_POLYGON_H__
