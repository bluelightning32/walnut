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
  using Parent =
    typename ParentTemplate::template MakeParent<BSPPolygon<ParentTemplate>,
                                                 BSPEdgeInfo<EdgeParent>
                                                >;
  using BSPEdgeInfoRep = BSPEdgeInfo<EdgeParent>;

  static_assert(
      std::is_base_of<ConvexPolygon<typename Parent::EdgeRep::Parent>,
                      Parent>::value,
      "The ParentTemplate must inherit from ConvexPolygon.");

  BSPPolygon() = default;

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

  // Overload CreateSplitChildren to create the derived polygon type.
  std::pair<BSPPolygon, BSPPolygon> CreateSplitChildren(
      ConvexPolygonSplitInfo&& split) && {
    std::pair<BSPPolygon, BSPPolygon> result;
    FillInSplitChildren(std::move(*this), std::move(split), result.first,
                        result.second);
    return result;
  }

  BSPEdgeInfoRep& bsp_edge_info(size_t index) {
    return this->edge(index);
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

 private:
  template <typename OutputPolygon>
  friend class BSPNode;

  // This is called directly by BSPNode.
  void ResetBSPInfo() {
    for (size_t i = 0; i < Parent::vertex_count(); ++i) {
      Parent::edge(i).ResetBSPInfo();
    }
  }

  // This is called directly by BSPNode.
  void UpdateBoundaryAngles(SplitSide coincident_info, size_t coincident_begin,
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
      if (edge_info.edge_first_coincident_.split == nullptr) {
        if (on_node_plane.split != nullptr) {
          edge_info.edge_first_coincident_ = on_node_plane;
        } else {
          edge_info.edge_first_coincident_ = coincident_info;
        }
      }
      edge_info.edge_last_coincident_ = coincident_info;
      edge_info.vertex_last_coincident_ = coincident_info;
    }
    // Update the last target vertex.
    BSPEdgeInfoRep& edge_info = bsp_edge_info(pos % this->vertex_count());
    edge_info.vertex_last_coincident_ = coincident_info;
  }
};

}  // walnut

#endif // WALNUT_BSP_POLYGON_H__
