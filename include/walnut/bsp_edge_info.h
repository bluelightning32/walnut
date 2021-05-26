#ifndef WALNUT_BSP_EDGE_INFO_H__
#define WALNUT_BSP_EDGE_INFO_H__

// for std::ostream
#include <ostream>

#include "walnut/edge_info_root.h"

namespace walnut {

struct SplitSide {
  const HalfSpace3* split = nullptr;
  bool pos_side = false;

  bool operator==(const SplitSide& other) const {
    return split == other.split && pos_side == other.pos_side;
  }

  bool operator!=(const SplitSide& other) const {
    return split != other.split || pos_side != other.pos_side;
  }
};

template <typename ParentTemplate = EdgeInfoRoot>
class BSPEdgeInfo : public ParentTemplate {
 public:
  using Parent = ParentTemplate;

  BSPEdgeInfo(const EdgeInfoRoot&) { }

  BSPEdgeInfo(RValueKey<BSPEdgeInfo> other)
    noexcept(std::is_nothrow_constructible<Parent, RValueKey<Parent>>::value)
    : Parent(RValueKey<Parent>(other)),
      edge_created_by(std::move(other.get().edge_created_by)),
      edge_first_coincident(std::move(other.get().edge_first_coincident)),
      vertex_last_coincident(std::move(other.get().vertex_last_coincident)),
      edge_last_coincident(std::move(other.get().edge_last_coincident)) { }

  // Create a new vertex on the parent's existing edge.
  //
  // Inherit the edge boundary angle from the parent, but initialize
  // the vertex boundary angle from the edge boundary angle instead of
  // inheriting the parent's vertex boundary angle.
  BSPEdgeInfo(const BSPEdgeInfo& parent, const HomoPoint3& new_source) :
    edge_created_by(parent.edge_created_by),
    edge_first_coincident(parent.edge_first_coincident),
    vertex_last_coincident(parent.edge_last_coincident),
    edge_last_coincident(parent.edge_last_coincident) { }

  // Create a new line from the parent's existing vertex.
  //
  // Inherit the vertex boundary angle from the parent, but default initialize
  // the edge boundary angle.
  BSPEdgeInfo(const BSPEdgeInfo& parent, const PluckerLine& new_line) :
    vertex_last_coincident(parent.vertex_last_coincident) { }

  // Create a new line starting on a new vertex on the parent's existing edge.
  //
  // Initialize the vertex boundary angle from the parent's edge boundary
  // angle. Default initialize the edge boundary angle.
  BSPEdgeInfo(const BSPEdgeInfo& parent, const HomoPoint3& new_source,
              const PluckerLine& new_line) :
    vertex_last_coincident(parent.edge_last_coincident) { }

  bool operator==(const EdgeInfoRoot&) const {
    return true;
  }
  bool operator!=(const EdgeInfoRoot&) const {
    return false;
  }

  void ResetBSPInfo() {
    edge_created_by = SplitSide();
    edge_first_coincident = SplitSide();
    vertex_last_coincident = SplitSide();
    edge_last_coincident = SplitSide();
  }

  // The split plane of the BSPNode that created this edge through a split, or
  // nullptr if the edge came from the input polygon.
  SplitSide edge_created_by;

  std::ostream& Approximate(std::ostream& out) const;

  // The split plane of the BSPNode highest in the tree that is coincident with
  // the entire edge, or nullptr, if the edge is not coincident with any of its
  // ancestor nodes.
  SplitSide edge_first_coincident;

  // The split plane of the BSPNode deepest in the tree that is coincident with
  // the vertex, or nullptr, if the vertex is not coincident with any of its
  // ancestor nodes.
  SplitSide vertex_last_coincident;

  // The split plane of the BSPNode deepest in the tree that is coincident with
  // the entire edge, or nullptr, if the edge is not coincident with any of its
  // ancestor nodes.
  SplitSide edge_last_coincident;
};

template <typename ParentTemplate>
std::ostream& operator<<(std::ostream& out,
                         const BSPEdgeInfo<ParentTemplate>& info) {
  out << "< edge_created_by=" << info.edge_created_by
      << ", edge_first_coincident=" << info.edge_first_coincident
      << ", edge_last_coincident=" << info.edge_last_coincident
      << ", vertex_last_coincident=" << info.vertex_last_coincident
      << " >";
  return out;
}

inline std::ostream& operator<<(std::ostream& out, const SplitSide& info) {
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

template <typename Parent>
inline std::ostream& BSPEdgeInfo<Parent>::Approximate(
    std::ostream& out) const {
  out << " edge_first_coincident=" << edge_first_coincident;
  return Parent::Approximate(out);
}

}  // walnut

#endif // WALNUT_BSP_EDGE_INFO_H__
