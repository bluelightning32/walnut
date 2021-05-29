#ifndef WALNUT_CONNECTED_POLYGON_H__
#define WALNUT_CONNECTED_POLYGON_H__

#include "gtest/gtest_prod.h"
#include "walnut/convex_polygon.h"
#include "walnut/deed.h"

namespace walnut {

template <typename ParentTemplate = ConvexPolygon<>,
          typename FinalPolygonTemplate = void,
          typename EdgeParentTemplate = EdgeInfoRoot>
class ConnectedPolygon;

// A half edge of a ConnectedPolygon
//
// The DeedTarget is used to keep a pointer to the edges while sorting and
// connecting them.
template <typename FinalPolygonTemplate,
          typename ParentTemplate>
class ConnectedEdge : public ParentTemplate, public DeedTarget {
 public:
  using FinalPolygon = FinalPolygonTemplate;
  using Parent = ParentTemplate;
  using ConnectedEdgeRep = ConnectedEdge;

  ConnectedEdge() = default;

  ConnectedEdge(RValueKey<ConnectedEdge> other)
    noexcept(std::is_nothrow_constructible<Parent, RValueKey<Parent>>::value)
    : Parent(RValueKey<Parent>(other)), DeedTarget(std::move(other.get())),
      polygon_(other.get().polygon_), partner_(other.get().partner_) {
    if (partner_ != nullptr) {
      partner_->partner_ = this;
      other.get().partner_ = nullptr;
    }
  }

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

  // Returns the start vertex of this edge.
  const HomoPoint3& vertex() const {
    using FinalEdgeRep = typename FinalPolygon::EdgeRep;
    const FinalEdgeRep& final_this = static_cast<const FinalEdgeRep&>(*this);
    return final_this.vertex();
  }

  // Returns the begin location of this edge as defined by the
  // `EdgeLineConnector`.
  const HomoPoint3& GetBeginLocation(int sorted_dimension) const {
    if (IsPositive(sorted_dimension)) {
      return vertex();
    } else {
      // This is a negative edge. It starts at the start of the next edge.
      return next_vertex();
    }
  }

  // Gets the edge in the partner polygon that originates from the same source
  // vertex.
  //
  // When viewed from the outside of the polyhedron, this function returns the
  // next edge in the clockwise direction that originates from the same source
  // vertex.
  //
  // This function returns nullptr if the edge does not have a partner. This
  // may happen if the polyhedron is not closed.
  const ConnectedEdge* GetNextAroundVertex() const {
    if (!partner_) return nullptr;

    FinalPolygon& next_polygon = partner_->polygon();
    return &next_polygon.edge((partner_->edge_index() + 1) %
                              next_polygon.vertex_count());
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

  bool CanMerge(const ConnectedEdge& next) const {
    if (partner() != nullptr && next.partner() != nullptr) {
      if (&partner()->polygon() != &next.partner()->polygon()) {
        // The edges have different partner polygons. They cannot be merged.
        return false;
      }
      using FinalEdgeRep = typename FinalPolygon::EdgeRep;
      // The partner edges are in the opposite order. So my prev == other next.
      const FinalEdgeRep& next_partner =
        partner()->polygon().edge(partner()->edge_index());
      const FinalEdgeRep& partner =
        next.partner()->polygon().edge(next.partner()->edge_index());

      // Temporarily unlink the partner to prevent an infinite loop. Even
      // though the partner points to `this`, save the value because `this` is
      // a const pointer and `old_value` is non-const.
      ConnectedEdge* old_value = partner_->partner_;
      assert(old_value == this);
      partner_->partner_ = nullptr;
      bool merge_result = partner.CanMerge(/*next=*/next_partner);
      partner_->partner_ = old_value;
      if (!merge_result) return false;
    }
    return Parent::CanMerge(next);
  }

  std::ostream& Approximate(std::ostream& out) const {
    out << " next=";
    next_vertex().Approximate(out);
    out << " polygon=" << &polygon() << " partner=" << partner();
    return Parent::Approximate(out);
  }

 protected:
  ConnectedEdge& operator=(const ConnectedEdge&) = default;

  ConnectedEdge& operator=(RValueKey<ConnectedEdge> other)
      noexcept(std::is_nothrow_move_assignable<
                 AssignableWrapper<Parent>
               >::value) {
    static_cast<Parent&>(*this) = RValueKey<Parent>(other);
    static_cast<DeedTarget&>(*this) = std::move(other.get());

    std::swap(partner_, other.get().partner_);
    polygon_ = other.get().polygon_;
    if (partner_ != nullptr) {
      partner_->partner_ = this;
    }
    if (other.get().partner_ != nullptr) {
      other.get().partner_->partner_ = &other.get();
    }
    return *this;
  }

  void Merge(ConnectedEdge& next) {
    assert(CanMerge(next));
    if (next.partner() != nullptr) {
      // This signals the partner not to try and merge this polygon's edges
      // again (preventing an infinite loop).
      partner_->partner_ = nullptr;
      partner_ = next.partner_;
      partner_->partner_ = this;
      next.partner_ = nullptr;

      partner()->polygon().MergeConnectedEdge(partner()->edge_index());
    }
    Parent::Merge(next);
  }

  template <typename EdgeParent>
  void EdgeMoved(ConvexPolygon<EdgeParent>& target) {
    polygon_ = static_cast<FinalPolygon*>(&target);
    Parent::EdgeMoved(target);
  }

 private:
  template <typename ParentPolygon, typename FinalPolygon, typename EdgeParent>
  friend class ConnectedPolygon;
  template <typename EdgeTemplate>
  friend class EdgeLineConnector;

  FRIEND_TEST(ConnectedEdge, MoveAssign);
  FRIEND_TEST(ConnectedEdge, MoveConstruct);
  FRIEND_TEST(ConnectedPolygon, SplitEdge);
  FRIEND_TEST(ConnectedPolygon, Merge);

  void ResetPartners() {
    partner_ = nullptr;
  }

  // Through the friend statement, this field is be modified by
  // ConnectedPolygon.
  FinalPolygon* polygon_ = nullptr;

  ConnectedEdge* partner_ = nullptr;
};

// The value field in this class will be set to true if the template parameter,
// `Polygon` is `ConnectedPolygon` with some combination of template arguments.
template <typename Polygon>
struct IsExactlyConnectedPolygon {
  static constexpr bool value = false;
};

template <typename ParentTemplate, typename FinalPolygonTemplate,
          typename EdgeParentTemplate>
struct IsExactlyConnectedPolygon<ConnectedPolygon<ParentTemplate,
                                                  FinalPolygonTemplate,
                                                  EdgeParentTemplate>> {
  static constexpr bool value = true;
};

// The value field in this class will be set to true if the template parameter,
// `Polygon` inherits from `ConnectedPolygon` with some combination of template
// arguments.
template <typename Polygon>
struct IsConnectedPolygon {
 private:
  template <typename T>
  static std::false_type Check(...);

  template <typename T>
  static std::enable_if_t<
    IsExactlyConnectedPolygon<typename T::ConnectedPolygonRep>::value,
    std::true_type>
  Check(int);

 public:
  static constexpr bool value = decltype(Check<Polygon>(0))::value;
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
template <typename ParentTemplate, typename FinalPolygonTemplate,
          typename EdgeParentTemplate>
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
  using ConnectedPolygonRep = ConnectedPolygon;
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

  ConnectedPolygon(const ConnectedPolygon& other) : Parent(other) {
    SetEdgeBackPointers();
  }

  ConnectedPolygon(RValueKey<ConnectedPolygon> other) noexcept
    : Parent(RValueKey<Parent>(other)) {
    static_assert(
        std::is_nothrow_constructible<Parent, RValueKey<Parent>>::value,
        "The ConnectedPolygon parent type must be nothrow move constructible "
        "so that std::vector uses the move constructor.");
    SetEdgeBackPointers();
  }

  ConnectedPolygon(ConnectedPolygon&& other) noexcept
    : ConnectedPolygon(RValueKey<ConnectedPolygon>(std::move(other))) {
    static_assert(
        std::is_nothrow_constructible<
          ConnectedPolygon, RValueKey<ConnectedPolygon>
        >::value,
        "The ConnectedPolygon parent type must be nothrow move constructible "
        "so that std::vector uses the move constructor.");
  }

  template <typename OtherPolygon,
            std::enable_if_t<std::is_constructible<Parent,
                                                   OtherPolygon>::value,
                             bool> = true>
  ConnectedPolygon(OtherPolygon&& other)
    : Parent(std::forward<OtherPolygon>(other)) {
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

  static std::pair<ConnectedPolygon, ConnectedPolygon> CreateSplitChildren(
      RValueKey<ConnectedPolygon> polygon,
      ConvexPolygonSplitInfo&& split) {
    std::pair<ConnectedPolygon, ConnectedPolygon> result;
    FillInSplitChildren(std::move(polygon.get()), std::move(split),
                        result.first, result.second);
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
  using Parent::SortVertices;
  using Parent::TryMergePolygon;

  RValueKey<ConnectedPolygon> GetRValueKey() && {
    return RValueKey<ConnectedPolygon>(std::move(*this));
  }

  ConnectedPolygon& operator=(ConnectedPolygon&& other) {
    return *this = std::move(other).GetRValueKey();
  }

  ConnectedPolygon& operator=(RValueKey<ConnectedPolygon> other) {
    Parent::operator=(RValueKey<Parent>(other));
    SetEdgeBackPointers();
    return *this;
  }

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
  template <typename FinalPolygon, typename Parent>
  friend class ConnectedEdge;

  void SetEdgeBackPointers() {
    for (size_t i = 0; i < vertex_count(); ++i) {
      edge(i).polygon_ = static_cast<FinalPolygon*>(this);
    }
  }

  // Called by ConnectedEdge.
  void MergeConnectedEdge(size_t index) {
    bool merged = static_cast<FinalPolygon*>(this)->TryMergeEdge(index);
    // Fix an unused variable warning in release builds.
    (void)(merged);
    assert(merged);
  }
};

template <typename FinalPolygon, typename Parent>
inline std::ostream& operator<<(std::ostream& out,
                                const ConnectedEdge<FinalPolygon,
                                                    Parent>& edge) {
  out << " next=" << edge.next_vertex() << " polygon=" << &edge.polygon()
      << " partner=" << edge.partner();
  return out;
}

}  // walnut

#endif // WALNUT_CONNECTED_POLYGON_H__
