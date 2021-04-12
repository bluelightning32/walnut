#ifndef WALNUT_BSP_EDGE_INFO_H__
#define WALNUT_BSP_EDGE_INFO_H__

// for std::ostream
#include <ostream>

#include "walnut/edge_info_root.h"

namespace walnut {

struct SplitSide {
  const HalfSpace3* split = nullptr;
  bool pos_side = false;
};

template <typename ParentTemplate = EdgeInfoRoot>
class BSPEdgeInfo : public ParentTemplate {
 public:
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

  const SplitSide& edge_first_coincident() const {
    return edge_first_coincident_;
  }

  const SplitSide& vertex_last_coincident() const {
    return vertex_last_coincident_;
  }

  const SplitSide& edge_last_coincident() const {
    return edge_last_coincident_;
  }

 private:
  template <typename PolygonParent, typename EdgeParent>
  friend class BSPPolygon;

  void ResetBSPInfo() {
    edge_first_coincident_ = SplitSide();
    vertex_last_coincident_ = SplitSide();
    edge_last_coincident_ = SplitSide();
  }

  // The split plane of the BSPNode highest in the tree that is coincident with
  // the entire edge, or nullptr, if the edge is not coincident with any of its
  // ancestor nodes.
  //
  // This field is updated directly by BSPPolygon.
  SplitSide edge_first_coincident_;

  // The split plane of the BSPNode deepest in the tree that is coincident with
  // the vertex, or nullptr, if the vertex is not coincident with any of its
  // ancestor nodes.
  //
  // This field is updated directly by BSPPolygon.
  SplitSide vertex_last_coincident_;

  // The split plane of the BSPNode deepest in the tree that is coincident with
  // the entire edge, or nullptr, if the edge is not coincident with any of its
  // ancestor nodes.
  //
  // This field is updated directly by BSPPolygon.
  SplitSide edge_last_coincident_;
};

template <typename ParentTemplate>
std::ostream& operator<<(std::ostream& out,
                         const BSPEdgeInfo<ParentTemplate>& info) {
  out << "< edge_first_coincident=" << info.edge_first_coincident()
      << ", edge_last_coincident=" << info.edge_last_coincident()
      << ", vertex_last_coincident=" << info.vertex_last_coincident()
      << " >";
  return out;
}

std::ostream& operator<<(std::ostream& out, const SplitSide& info) {
  if (info.split == nullptr) {
    out << "none";
  } else {
    out << "[";
    if (info.pos_side) {
      out << "pos";
    } else {
      out << "neg";
    }
    out << " " << *info.split << "]";
  }
  return out;
}

}  // walnut

#endif // WALNUT_BSP_EDGE_INFO_H__
