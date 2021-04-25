#ifndef WALNUT_BSP_POLYGON_H__
#define WALNUT_BSP_POLYGON_H__

// for std::is_base_of
#include <type_traits>
// for std::pair and std::move
#include <utility>

#include "walnut/convex_polygon.h"
#include "walnut/bsp_edge_info.h"

namespace walnut {

using BSPContentId = size_t;

template <typename ParentTemplate = ConvexPolygon<>,
          typename EdgeParentTemplate = EdgeInfoRoot>
class BSPPolygon :
  public ParentTemplate::template MakeParent<BSPPolygon<ParentTemplate>,
                                             BSPEdgeInfo<EdgeParentTemplate>
                                            > {
 public:
  using EdgeParent = EdgeParentTemplate;
  using UnspecializedParent = ParentTemplate;
  using Parent =
    typename ParentTemplate::template MakeParent<BSPPolygon<ParentTemplate>,
                                                 BSPEdgeInfo<EdgeParent>
                                                >;
  using BSPEdgeInfoRep = BSPEdgeInfo<EdgeParent>;
  using BSPPolygonRep = BSPPolygon;

  static_assert(
      std::is_base_of<ConvexPolygon<typename Parent::EdgeRep::Parent>,
                      Parent>::value,
      "The ParentTemplate must inherit from ConvexPolygon.");

  BSPPolygon() = default;

  BSPPolygon(const BSPPolygon&) = default;

  BSPPolygon(BSPPolygon&& other)
    noexcept(
        std::is_nothrow_constructible<
          BSPPolygon, RValueKey<BSPPolygon>
        >::value)
    : BSPPolygon(RValueKey<BSPPolygon>(std::move(other))) { }

  BSPPolygon(RValueKey<BSPPolygon> other)
    noexcept(std::is_nothrow_constructible<Parent, RValueKey<Parent>>::value)
    : Parent(RValueKey<Parent>(other)),
      id(other.get().id),
      on_node_plane(std::move(other.get().on_node_plane)) { }

  template <typename OtherPolygon>
  BSPPolygon(BSPContentId id, const HalfSpace3* on_node_plane, bool pos_side,
             OtherPolygon&& parent) :
    Parent(std::forward<OtherPolygon>(parent)), id(id),
    on_node_plane{on_node_plane, pos_side} { }

  BSPPolygon(const HalfSpace3* on_node_plane, bool pos_side,
             const BSPPolygon& parent) :
    Parent(parent), id(parent.id),
    on_node_plane{on_node_plane, pos_side} { }

  BSPPolygon(const HalfSpace3* on_node_plane, bool pos_side,
             BSPPolygon&& parent) :
    Parent(std::move(parent)), id(parent.id),
    on_node_plane{on_node_plane, pos_side} { }

  // Overload CreateSplitChildren to create the derived polygon type.
  std::pair<BSPPolygon, BSPPolygon> CreateSplitChildren(
      const ConvexPolygonSplitInfo& split) const {
    std::pair<BSPPolygon, BSPPolygon> result;
    FillInSplitChildren(*this, split, result.first, result.second);
    return result;
  }

  BSPPolygon& operator=(const BSPPolygon&) = default;

  BSPPolygon& operator=(BSPPolygon&& other) {
    return operator=(RValueKey<BSPPolygon>(std::move(other)));
  }

  BSPPolygon& operator=(RValueKey<BSPPolygon> other) {
    Parent::operator=(RValueKey<Parent>(other));
    id = other.get().id;
    on_node_plane = std::move(other.get().on_node_plane);
    return *this;
  }

  // Overload CreateSplitChildren to create the derived polygon type.
  std::pair<BSPPolygon, BSPPolygon> CreateSplitChildren(
      ConvexPolygonSplitInfo&& split) && {
    std::pair<BSPPolygon, BSPPolygon> result;
    FillInSplitChildren(std::move(*this), std::move(split), result.first,
                        result.second);
    return result;
  }

  static std::pair<BSPPolygon, BSPPolygon> CreateSplitChildren(
      RValueKey<BSPPolygon> polygon,
      ConvexPolygonSplitInfo&& split) {
    std::pair<BSPPolygon, BSPPolygon> result;
    FillInSplitChildren(std::move(polygon.get()), std::move(split),
                        result.first, result.second);
    return result;
  }

  BSPEdgeInfoRep& bsp_edge_info(size_t index) {
    return this->edge(index);
  }

  void ResetBSPInfo() {
    for (size_t i = 0; i < Parent::vertex_count(); ++i) {
      Parent::edge(i).ResetBSPInfo();
    }
  }

  // Sets the boundary angles in the edges and vertices in the given range to
  // `coincident_info`.
  //
  // The vertices in the range [coincident_begin, coincident_end) are updated.
  // The edges in the range [coincident_begin, coincident_end - 1) are updated.
  //
  // The caller must ensure coincident_begin <= coincident_end. The function
  // will apply the modulus on the vertex indices, so it is okay for
  // coincident_end to be greater than vertex_count().
  void SetBoundaryAngles(SplitSide coincident_info, size_t coincident_begin,
                         size_t coincident_end) {
    // Typically this function is called with 0 vertices to update. So quickly
    // handle that case first.
    if (coincident_begin == coincident_end) return;

    size_t pos = coincident_begin;
    // Edges go from source to target. So first loop through all of the edges
    // that need to be updated, and update their corresponding source vertices
    // along the way too.
    for (; pos < coincident_end - 1; ++pos) {
      BSPEdgeInfoRep& edge_info = bsp_edge_info(pos % this->vertex_count());
      if (edge_info.edge_first_coincident.split == nullptr) {
        if (on_node_plane.split != nullptr) {
          // This edge is newly created, that's why it's
          // edge_first_coincident.split is nullptr. However, the parent
          // polygon was already on a plane. So this new edge must also be on
          // that plane.
          edge_info.edge_first_coincident = on_node_plane;
        } else {
          edge_info.edge_first_coincident = coincident_info;
        }
      }
      edge_info.edge_last_coincident = coincident_info;
      edge_info.vertex_last_coincident = coincident_info;
    }
    // Update the last target vertex.
    BSPEdgeInfoRep& edge_info = bsp_edge_info(pos % this->vertex_count());
    edge_info.vertex_last_coincident = coincident_info;
  }

  // Updates the boundary angles for all of the edges outside of
  // `exclude_range`.
  void SetBoundaryAngles(SplitSide coincident_info,
                         const std::pair<size_t, size_t>& exclude_range) {
    SetBoundaryAngles(coincident_info, exclude_range.second,
                      this->vertex_count() + exclude_range.first);
  }


  // Updates the boundary angles on the BSPPolygon children created by
  // `CreateSplitChildren`.
  //
  // `FinalPolygon` must inherit from BSPPolygon.
  template <typename FinalPolygon>
  static void SetChildBoundaryAngles(std::pair<FinalPolygon,
                                               FinalPolygon>& children,
                                     const HalfSpace3& split) {
    assert(children.first.vertex_count() > 2);
    assert(children.second.vertex_count() > 2);
    // As described by the CreateSplitChildren function declaration
    // comment, the last 2 vertices of neg_poly will touch the plane.
    children.first.SetBoundaryAngles(SplitSide{&split, /*pos_child=*/false},
                                     children.first.vertex_count() - 2,
                                     children.first.vertex_count());
    // The first and last vertices of pos_poly will touch the plane.
    children.second.SetBoundaryAngles(SplitSide{&split, /*pos_child=*/true},
                                      children.second.vertex_count() - 1,
                                      children.second.vertex_count() + 1);
  }

  BSPContentId id;

  // This is the BSPNode whose split plane is coincident with this polygon's
  // plane, or nullptr if no such BSPNode exists.
  //
  // pos_side is true if this polygon is a child of the positive child of the
  // split node.
  SplitSide on_node_plane;

 protected:
  // Overrides the non-virtual function from ConvexPolygon.
  template <typename ParentRef, typename SplitInfoRef>
  static void FillInSplitChildren(ParentRef&& parent, SplitInfoRef&& split,
                                  BSPPolygon& neg_child,
                                  BSPPolygon& pos_child) {
    pos_child.id = parent.id;
    pos_child.on_node_plane = parent.on_node_plane;
    neg_child.id = parent.id;
    neg_child.on_node_plane = parent.on_node_plane;

    Parent::FillInSplitChildren(std::forward<ParentRef>(parent),
                                std::forward<SplitInfoRef>(split), neg_child,
                                pos_child);
  }
};

}  // walnut

#endif // WALNUT_BSP_POLYGON_H__
