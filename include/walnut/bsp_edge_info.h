#ifndef WALNUT_BSP_EDGE_INFO_H__
#define WALNUT_BSP_EDGE_INFO_H__

// for std::ostream
#include <ostream>

#include "walnut/edge_info_root.h"

namespace walnut {

template <typename BSPNodeTemplate>
class BSPPolygonWrapper;

template <typename BSPNodeTemplate>
struct BSPNodeSide {
  using BSPNodeRep = BSPNodeTemplate;

  const BSPNodeRep* node = nullptr;
  bool pos_side = false;
};

template <typename BSPNodeTemplate, typename ParentTemplate = EdgeInfoRoot>
class BSPEdgeInfo : public ParentTemplate {
 public:
  using BSPNodeRep = BSPNodeTemplate;
  using BSPNodeSideRep = BSPNodeSide<BSPNodeRep>;

  BSPEdgeInfo(const EdgeInfoRoot&) { }

  // Create a new vertex on the parent's existing edge.
  //
  // Inherit the edge boundary angle from the parent, but initialize
  // the vertex boundary angle from the edge boundary angle instead of
  // inheriting the parent's vertex boundary angle.
  BSPEdgeInfo(const BSPEdgeInfo& parent, const HomoPoint3& new_source) :
    edge_first_coincident_(parent.edge_first_coincident_),
    vertex_last_coincident_(parent.edge_last_coincident_),
    edge_last_coincident_(parent.edge_last_coincident_) { }

  // Create a new line from the parent's existing vertex.
  //
  // Inherit the vertex boundary angle from the parent, but default initialize
  // the edge boundary angle.
  BSPEdgeInfo(const BSPEdgeInfo& parent, const PluckerLine& new_line) :
    vertex_last_coincident_(parent.vertex_last_coincident_) { }

  // Create a new line starting on a new vertex on the parent's existing edge.
  //
  // Initialize the vertex boundary angle from the parent's edge boundary
  // angle. Default initialize the edge boundary angle.
  BSPEdgeInfo(const BSPEdgeInfo& parent, const HomoPoint3& new_source,
              const PluckerLine& new_line) :
    vertex_last_coincident_(parent.edge_last_coincident_) { }

  bool operator==(const EdgeInfoRoot&) const {
    return true;
  }
  bool operator!=(const EdgeInfoRoot&) const {
    return false;
  }

  const BSPNodeSideRep& edge_first_coincident() const {
    return edge_first_coincident_;
  }

  const BSPNodeSideRep& vertex_last_coincident() const {
    return vertex_last_coincident_;
  }

  const BSPNodeSideRep& edge_last_coincident() const {
    return edge_last_coincident_;
  }

 private:
  friend BSPNodeRep;
  friend BSPPolygonWrapper<BSPNodeRep>;

  void ResetBSPInfo() {
    edge_first_coincident_ = BSPNodeSideRep();
    vertex_last_coincident_ = BSPNodeSideRep();
    edge_last_coincident_ = BSPNodeSideRep();
  }

  // The BSPNodeRep highest in the tree that is coincident with the entire
  // edge, or nullptr, if the edge is not coincident with any of its ancestor
  // nodes.
  //
  // This field is updated directly by BSPNodeRep.
  BSPNodeSideRep edge_first_coincident_;

  // The BSPNodeRep deepest in the tree that is coincident with the vertex, or
  // nullptr, if the vertex is not coincident with any of its ancestor nodes.
  //
  // This field is updated directly by BSPNodeRep.
  BSPNodeSideRep vertex_last_coincident_;

  // The BSPNodeRep deepest in the tree that is coincident with the entire
  // edge, or nullptr, if the edge is not coincident with any of its ancestor
  // nodes.
  //
  // This field is updated directly by BSPNodeRep.
  BSPNodeSideRep edge_last_coincident_;
};

template <typename OutputPolygonParent, typename ParentTemplate>
std::ostream& operator<<(
    std::ostream& out,
    const BSPEdgeInfo<OutputPolygonParent, ParentTemplate>& info) {
  out << "< edge_first_coincident=" << info.edge_first_coincident()
      << ", edge_last_coincident=" << info.edge_last_coincident()
      << ", vertex_last_coincident=" << info.vertex_last_coincident()
      << " >";
  return out;
}

}  // walnut

#endif // WALNUT_BSP_EDGE_INFO_H__
