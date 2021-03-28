#ifndef WALNUT_BSP_NODE_H__
#define WALNUT_BSP_NODE_H__

// for std::unique_ptr
#include <memory>
// for std::ostream
#include <ostream>
// for std::is_base_of
#include <type_traits>
// for std::pair and std::move
#include <utility>
#include <vector>

#include "walnut/aabb_convex_polygon.h"
#include "walnut/half_space3.h"
#include "walnut/mutable_convex_polygon.h"
#include "walnut/r_transformation.h"

namespace walnut {

template <typename BSPNodeTemplate, typename ParentTemplate>
class BSPEdgeInfo;

template <typename OutputPolygonParentTemplate>
class BSPNode;

template <typename OutputPolygonParentTemplate>
class BSPTree;

template <typename BSPNodeTemplate>
class BSPPolygonWrapper;

using BSPPolygonId = size_t;

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
  using PolygonRep = typename BSPNodeRep::PolygonRep;

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

template <typename BSPNodeTemplate>
class BSPPolygonWrapper :
  public BSPNodeTemplate::OutputPolygonParent::template MakeParent<
    BSPPolygonWrapper<BSPNodeTemplate>, BSPEdgeInfo<BSPNodeTemplate>
  > {
 public:
  using BSPNodeRep = BSPNodeTemplate;
  using Parent =
    typename BSPNodeTemplate::OutputPolygonParent::template MakeParent<
      BSPPolygonWrapper<BSPNodeTemplate>, BSPEdgeInfo<BSPNodeRep>
    >;
  using typename Parent::EdgeParent;
  using BSPEdgeInfoRep = BSPEdgeInfo<BSPNodeRep>;
  using BSPNodeSideRep = typename BSPEdgeInfoRep::BSPNodeSideRep;

  static_assert(std::is_base_of<BSPEdgeInfoRep, EdgeParent>::value,
                "The ConvexPolygon's EdgeParent must inherit from "
                "BSPEdgeInfo.");
  static_assert(std::is_base_of<ConvexPolygon<EdgeParent>, Parent>::value,
      "The OutputPolygonParentTemplate must inherit from ConvexPolygon.");

  BSPPolygonWrapper() = default;

  template <typename OtherPolygon>
  BSPPolygonWrapper(BSPPolygonId id, const BSPNodeRep* on_node_plane,
                    bool pos_side, OtherPolygon&& parent) :
    Parent(std::forward<OtherPolygon>(parent)), id(id),
    on_node_plane{on_node_plane, pos_side} { }

  BSPPolygonWrapper(const BSPNodeRep* on_node_plane, bool pos_side,
                    const BSPPolygonWrapper& parent) :
    Parent(parent), id(parent.id),
    on_node_plane{on_node_plane, pos_side} { }

  BSPPolygonWrapper(const BSPNodeRep* on_node_plane, bool pos_side,
                    BSPPolygonWrapper&& parent) :
    Parent(std::move(parent)), id(parent.id),
    on_node_plane{on_node_plane, pos_side} { }

  // Overload CreateSplitChildren to create the derived polygon type.
  std::pair<BSPPolygonWrapper, BSPPolygonWrapper> CreateSplitChildren(
      const ConvexPolygonSplitInfo& split) const {
    std::pair<BSPPolygonWrapper, BSPPolygonWrapper> result;
    FillInSplitChildren(*this, split, result.first, result.second);
    return result;
  }

  // Overload CreateSplitChildren to create the derived polygon type.
  std::pair<BSPPolygonWrapper, BSPPolygonWrapper> CreateSplitChildren(
      ConvexPolygonSplitInfo&& split) && {
    std::pair<BSPPolygonWrapper, BSPPolygonWrapper> result;
    FillInSplitChildren(std::move(*this), std::move(split), result.first,
                        result.second);
    return result;
  }

  BSPEdgeInfoRep& bsp_edge_info(size_t index) {
    return this->edge(index);
  }

  BSPPolygonId id;

  // This is the BSPNode whose split plane is coincident with this polygon's
  // plane, or nullptr if no such BSPNode exists.
  //
  // pos_side is true if this polygon is a child of the positive child of the
  // split node.
  BSPNodeSideRep on_node_plane;

 protected:
  // Overrides the non-virtual function from ConvexPolygon.
  template <typename ParentRef, typename SplitInfoRef>
  static void FillInSplitChildren(ParentRef&& parent, SplitInfoRef&& split,
                                  BSPPolygonWrapper& neg_child,
                                  BSPPolygonWrapper& pos_child) {
    pos_child.id = parent.id;
    pos_child.on_node_plane = parent.on_node_plane;
    neg_child.id = parent.id;
    neg_child.on_node_plane = parent.on_node_plane;

    Parent::FillInSplitChildren(std::forward<ParentRef>(parent),
                                std::forward<SplitInfoRef>(split), neg_child,
                                pos_child);
  }

 private:
  friend BSPTree<typename BSPNodeTemplate::OutputPolygonParent>;

  void ResetBSPInfo() {
    for (size_t i = 0; i < Parent::vertex_count(); ++i) {
      Parent::edge(i).ResetBSPInfo();
    }
  }
};

// This is a node within a binary space partition tree.
//
// `OutputPolygonParentTemplate` must inherit from ConvexPolygon.
template <typename OutputPolygonParentTemplate = AABBConvexPolygon<>>
class BSPNode {
 public:
  using OutputPolygonParent = OutputPolygonParentTemplate;
  using PolygonRep = BSPPolygonWrapper<BSPNode>;
  using EdgeParent = typename PolygonRep::EdgeParent;
  using EdgeRep = typename PolygonRep::EdgeRep;
  using BSPEdgeInfoRep = typename PolygonRep::BSPEdgeInfoRep;
  using BSPNodeSideRep = typename BSPEdgeInfoRep::BSPNodeSideRep;

  friend class BSPTree<OutputPolygonParent>;

  static constexpr int point3_bits = OutputPolygonParent::point3_bits;

  BSPNode() = default;

  BSPNode(const std::vector<int64_t>& pwn_by_id) : pwn_by_id_(pwn_by_id) { }

  // Convert a leaf node into an interior node.
  //
  // This may only be called on a leaf node.
  //
  // The plane of `half_space` must be different from the any of the planes
  // that ancestor nodes were split on.
  //
  // The contents of this node will be pushed into the new child nodes.
  void Split(const HalfSpace3& half_space) {
    assert(half_space.IsValid());
    MakeInterior(half_space, new BSPNode(pwn_by_id_), new BSPNode(pwn_by_id_));
    PushContentsToChildren();
  }

  bool IsLeaf() const {
    return !split().IsValid();
  }

  const HalfSpace3& split() const {
    return split_;
  }

  const BSPNode* negative_child() const {
    return negative_child_.get();
  }
  BSPNode* negative_child() {
    return negative_child_.get();
  }
  const BSPNode* positive_child() const {
    return positive_child_.get();
  }
  BSPNode* positive_child() {
    return positive_child_.get();
  }

  const std::vector<PolygonRep>& contents() const {
    return contents_;
  }

  const std::vector<PolygonRep>& border_contents() const {
    return border_contents_;
  }

  int64_t GetPWNForId(BSPPolygonId id) const {
    if (id < pwn_by_id_.size()) {
      return pwn_by_id_[id];
    } else {
      return 0;
    }
  }

 protected:
  // Push the contents of an interior node to the children.
  //
  // This may only be called on an interior node.
  void PushContentsToChildren();

  void MakeInterior(const HalfSpace3& half_space,
                    BSPNode* negative_child,
                    BSPNode* positive_child) {
    assert(IsLeaf());
    split_ = half_space;
    negative_child_ = std::unique_ptr<BSPNode>(negative_child);
    positive_child_ = std::unique_ptr<BSPNode>(positive_child);
  }

  void Reset() {
    contents_.clear();
    border_contents_.clear();
    split_ = HalfSpace3();
    negative_child_.reset();
    positive_child_.reset();
    pwn_by_id_.clear();
  }

 private:
  // Push the contents all the way down to descendant leaf nodes. Call
  // `leaf_callback` on each leaf node that the contents were pushed to (and
  // possibly more nodes that the contents were not pushed to).
  //
  // This may be called on leaf or interior nodes.
  template <typename LeafCallback>
  void PushContentsToLeaves(LeafCallback leaf_callback);

  // Update the negative and positive child for the crossing (if any) that
  // occurs at a vertex.
  //
  // `edge_comparison` should be -1 if the edge boundary angle is clockwise
  // from the split_.normal(), or it should be 1 if it is counter-clockwise.
  // This function should not be called if the vectors are the same.
  //
  // crossing_flip should be 1 if the vertex_to_edge direction can be used
  // directly to update the PWN, or crossing_flip should be -1 if the opposite
  // should be used.
  void PushVertexPWNToChildren(BSPPolygonId polygon_id, int edge_comparison,
                               const BSPNodeSideRep& edge_last_coincident,
                               const EdgeRep& vertex_edge, int crossing_flip);

  // Update the children's PWN based on this node's contents.
  //
  // This may only be called on an interior node.
  void PushContentPWNToChildren();

  // Update the boundary angles in the edges and vertices of `polygon` that are
  // coincident with `split_`.
  //
  // The vertices in the range [coincident_begin, coincident_end) are updated.
  // The edges in the range [coincident_begin, coincident_end - 1) are updated.
  //
  // The caller must ensure coincident_begin <= coincident_end. The function
  // will apply the modulus on the vertex indices, so it is okay for
  // coincident_end to be greater than polygon.vertex_count().
  void UpdateBoundaryAngles(bool pos_child, PolygonRep& polygon,
      size_t coincident_begin, size_t coincident_end);

  // For a leaf node, these are the polygons that are inside the cell. They may
  // possibly be touching the cell border, but they are not on the border.
  //
  // For a finished interior node, this will be empty. When new contents are
  // added to the tree, this will be temporarily non-empty, until the new
  // contents are psuhed to the children.
  std::vector<PolygonRep> contents_;

  // For a leaf node, these are the polygons that are on the cell border.
  //
  // For a finished interior node, this will be empty. When new contents are
  // added to the tree, this will be temporarily non-empty, until the new
  // contents are psuhed to the children.
  std::vector<PolygonRep> border_contents_;

  // The plane that splits an interior node.
  //
  // For a leaf node, !split_.IsValid().
  HalfSpace3 split_;

  std::unique_ptr<BSPNode> negative_child_;
  std::unique_ptr<BSPNode> positive_child_;

  std::vector<int64_t> pwn_by_id_;
};

template <typename OutputPolygonParent>
void BSPNode<OutputPolygonParent>::PushVertexPWNToChildren(
    BSPPolygonId polygon_id, int edge_comparison,
    const BSPNodeSideRep& edge_last_coincident, const EdgeRep& vertex_edge,
    int crossing_flip) {
  const BSPNodeSideRep& vertex_last_coincident =
    vertex_edge.vertex_last_coincident();
  const int vertex_comparison =
    RXYCompareBivector(split_.normal(),
                       vertex_last_coincident.node->split().normal()) *
    (vertex_last_coincident.pos_side ? -1 : 1);

  // For a crossing at the vertex, the edge boundary angle and the
  // vertex angle must be on opposite sides of the split normal. That
  // means that `edge_comparison` and `vertex_comparison` will have
  // different signs.
  if (edge_comparison + vertex_comparison == 0) {
    assert((edge_comparison < 0 && vertex_comparison > 0) ||
           (edge_comparison > 0 && vertex_comparison < 0));
    const int vertex_edge_side_mult =
      (vertex_last_coincident.pos_side ^ edge_last_coincident.pos_side) ?
      -1 : 1;
    const int vertex_to_edge =
      RXYCompareBivector(vertex_last_coincident.node->split().normal(),
                         edge_last_coincident.node->split().normal()) *
      vertex_edge_side_mult;
    // Since the vertex boundary angle and edge boundary angle are on
    // opposite sides of the split normal,
    //   vertex_boundary_angle != edge_boundary_angle
    //   Compare(vertex_boundary_angle, edge_boundary_angle) != 0
    assert(vertex_to_edge != 0);
    BSPNode<OutputPolygonParent>* push_to_child;
    int side_comparison = vertex_comparison * vertex_to_edge;
    if (side_comparison >= 0) {
      assert (side_comparison == 1);
      // The split normal to vertex rotation is the same direction as
      // the vertex to current edge rotation.
      assert((vertex_comparison >= 0) == (vertex_to_edge >= 0));
      assert((edge_comparison >= 0) != (vertex_to_edge >= 0));
      push_to_child = negative_child();
    } else {
      assert (side_comparison == -1);
      // The split normal to vertex rotation is the opposite direction
      // as the vertex to current edge rotation.
      assert((vertex_comparison >= 0) != (vertex_to_edge >= 0));
      assert((edge_comparison >= 0) == (vertex_to_edge >= 0));
      push_to_child = positive_child();
    }
    assert (vertex_to_edge == 1 || vertex_to_edge == -1);
    int side_comparison2 = split_.Compare(vertex_edge.vertex());
    if (side_comparison * side_comparison2 >= 0) {
      // min_max_comparison will be positive if the following vectors are
      // arranged in counter-clockwise order:
      // * vertex_boundary_angle
      // * edge_boundary_angle
      // * split_.normal()
      //
      BigIntWord min_max_comparison = split_.normal().Dot(
          vertex_last_coincident.node->split().normal().Cross(
            edge_last_coincident.node->split().normal())).GetSign();
      // * Starting the list of vectors at a different offset does not
      //   affect the sign.
      // * Negating split_.normal() negates the sign.
      // * swapping vertex_boundary_angle and edge_boundary_angle
      //   negates the sign.
      //
      // split_.normal() is negated if (vertex_comparison ^
      // vertex_to_edge) < 0.
      //
      // vertex_boundary_angle and edge_boundary_angle are swapped if
      // vertex_to_edge < 0.
      //
      // So the adjusted sign of min_max_comparison is:
      //   min_max_comparison ^ (vertex_comparison ^
      //                         vertex_to_edge) ^ vertex_to_edge
      // = min_max_comparison ^ vertex_comparison
      if ((min_max_comparison ^ vertex_comparison ^
           vertex_edge_side_mult) >= 0) {
        if (push_to_child->pwn_by_id_.size() <= polygon_id) {
          push_to_child->pwn_by_id_.resize(polygon_id + 1);
        }
        push_to_child->pwn_by_id_[polygon_id] +=
          vertex_to_edge * crossing_flip;
      }
    }
  }
}

template <typename OutputPolygonParent>
void BSPNode<OutputPolygonParent>::PushContentPWNToChildren() {
  for (PolygonRep& polygon : contents_) {
    for (size_t i = 0; i < polygon.vertex_count(); ++i) {
      const EdgeRep& current_edge = polygon.const_edge(i);

      const BSPNodeSideRep& edge_last_coincident =
        current_edge.edge_last_coincident();
      if (edge_last_coincident.node == nullptr) continue;
      int edge_comparison =
        RXYCompareBivector(split_.normal(),
                           edge_last_coincident.node->split().normal()) *
        (edge_last_coincident.pos_side ? -1 : 1);
      if (edge_comparison == 0) continue;

      PushVertexPWNToChildren(polygon.id, edge_comparison,
                              edge_last_coincident, current_edge,
                              /*crossing_flip=*/1);

      const EdgeRep& next_edge =
        polygon.const_edge((i + 1)%polygon.vertex_count());
      PushVertexPWNToChildren(polygon.id, edge_comparison,
                              edge_last_coincident, next_edge,
                              /*crossing_flip=*/-1);
    }
  }
}

template <typename OutputPolygonParent>
void BSPNode<OutputPolygonParent>::UpdateBoundaryAngles(
    bool pos_child, PolygonRep& polygon, size_t coincident_begin,
    size_t coincident_end) {
  // Typically this function is called with 0 vertices to update. So quickly
  // handle that case first.
  if (coincident_begin == coincident_end) return;

  typename BSPEdgeInfoRep::BSPNodeSideRep coincident_info{this, pos_child};
  size_t pos = coincident_begin;
  // Edges go from source to target. So first loop through all of the edges
  // that need to be updated, and update their corresponding source vertices
  // along the way too.
  for (; pos < coincident_end - 1; ++pos) {
    BSPEdgeInfoRep& edge_info = polygon.bsp_edge_info(
        pos % polygon.vertex_count());
    if (edge_info.edge_first_coincident_.node == nullptr) {
      if (polygon.on_node_plane.node != nullptr) {
        edge_info.edge_first_coincident_ = polygon.on_node_plane;
      } else {
        edge_info.edge_first_coincident_ = coincident_info;
      }
    }
    edge_info.edge_last_coincident_ = coincident_info;
    edge_info.vertex_last_coincident_ = coincident_info;
  }
  // Update the last target vertex.
  BSPEdgeInfoRep& edge_info = polygon.bsp_edge_info(
      pos % polygon.vertex_count());
  edge_info.vertex_last_coincident_ = coincident_info;
}

template <typename OutputPolygonParent>
void BSPNode<OutputPolygonParent>::PushContentsToChildren() {
  PushContentPWNToChildren();

  for (PolygonRep& polygon : contents_) {
    assert(polygon.vertex_count() > 0);
    ConvexPolygonSplitInfo info = polygon.GetSplitInfo(split_);
    assert(info.IsValid(polygon.vertex_count()));

    if (info.ShouldEmitNegativeChild()) {
      if (info.ShouldEmitPositiveChild()) {
        std::pair<PolygonRep, PolygonRep> children =
          std::move(polygon).CreateSplitChildren(std::move(info));
        assert(children.first.vertex_count() > 2);
        assert(children.second.vertex_count() > 2);
        // As described by the CreateSplitChildren function declaration
        // comment, the last 2 vertices of neg_poly will touch the plane.
        UpdateBoundaryAngles(/*pos_child=*/false, children.first,
                            children.first.vertex_count() - 2,
                            children.first.vertex_count());
        // The first and last vertices of pos_poly will touch the plane.
        UpdateBoundaryAngles(/*pos_child=*/true, children.second,
                            children.second.vertex_count() - 1,
                            children.second.vertex_count() + 1);

        negative_child_->contents_.push_back(std::move(children.first));
        positive_child_->contents_.push_back(std::move(children.second));
      } else {
        UpdateBoundaryAngles(/*pos_child=*/false, polygon,
                            info.neg_range().second,
                            polygon.vertex_count() + info.neg_range().first);
        negative_child_->contents_.push_back(std::move(polygon));
      }
    } else if (info.ShouldEmitPositiveChild()) {
      UpdateBoundaryAngles(/*pos_child=*/true, polygon,
                          info.pos_range().second,
                          polygon.vertex_count() + info.pos_range().first);
      positive_child_->contents_.push_back(std::move(polygon));
    } else {
      assert(info.ShouldEmitOnPlane());
      // If polygon.plane().normal() and split_.normal() point in the same
      // direction, put polygon in the negative child.
      const int drop_dimension = polygon.drop_dimension();
      bool pos_child =
        polygon.plane().normal().components()[drop_dimension].HasDifferentSign(
            split_.normal().components()[drop_dimension]);
      UpdateBoundaryAngles(pos_child, polygon, /*coincident_begin=*/0,
                           /*coincident_end=*/polygon.vertex_count() + 1);
      if (pos_child) {
        positive_child_->border_contents_.emplace_back(this, /*pos_side=*/true,
                                                       std::move(polygon));
      } else {
        negative_child_->border_contents_.emplace_back(this,
                                                       /*pos_side=*/false,
                                                       std::move(polygon));
      }
    }
  }

  for (PolygonRep& polygon : border_contents_) {
    assert(polygon.vertex_count() > 0);
    ConvexPolygonSplitInfo info = polygon.GetSplitInfo(split_);

    if (info.ShouldEmitNegativeChild()) {
      if (info.ShouldEmitPositiveChild()) {
        std::pair<PolygonRep, PolygonRep> children =
          std::move(polygon).CreateSplitChildren(std::move(info));
        // As described by the CreateSplitChildren function declaration
        // comment, the last 2 vertices of neg_poly will touch the plane.
        UpdateBoundaryAngles(/*pos_child=*/false, children.first,
                            children.first.vertex_count() - 2,
                            children.first.vertex_count());
        // The first and last vertices of pos_poly will touch the plane.
        UpdateBoundaryAngles(/*pos_child=*/true, children.second,
                            children.second.vertex_count() - 1,
                            children.second.vertex_count() + 1);
        negative_child_->border_contents_.push_back(std::move(children.first));
        positive_child_->border_contents_.push_back(
            std::move(children.second));
      } else {
        negative_child_->border_contents_.push_back(std::move(polygon));
      }
    } else if (info.ShouldEmitPositiveChild()) {
      positive_child_->border_contents_.push_back(std::move(polygon));
    } else {
      assert(info.ShouldEmitOnPlane());
      // This branch only runs if this node is being split on the same plane as
      // an ancestor node, which should not happen.
      assert(false);
      // If polygon.plane().normal() and split_.normal() point in the same
      // direction, put polygon in the negative child.
      const int drop_dimension = polygon.drop_dimension();
      bool pos_child =
        polygon.plane().normal().components()[drop_dimension].HasDifferentSign(
            split_.normal().components()[drop_dimension]);
      UpdateBoundaryAngles(pos_child, polygon, /*coincident_begin=*/0,
                           /*coincident_end=*/polygon.vertex_count() + 1);
      if (pos_child) {
        positive_child_->border_contents_.push_back(std::move(polygon));
      } else {
        negative_child_->border_contents_.push_back(std::move(polygon));
      }
    }
  }
}

template <typename OutputPolygonParent>
template <typename LeafCallback>
void BSPNode<OutputPolygonParent>::PushContentsToLeaves(
    LeafCallback leaf_callback) {
  if (contents_.empty() && border_contents_.empty()) {
    return;
  }
  if (IsLeaf()) {
    leaf_callback(*this);
  } else {
    PushContentsToChildren();
    negative_child_->PushContentsToLeaves(leaf_callback);
    positive_child_->PushContentsToLeaves(leaf_callback);
  }
}

template <typename BSPNodeTemplate>
std::ostream& operator<<(std::ostream& out,
                         const BSPNodeSide<BSPNodeTemplate>& info) {
  if (info.node == nullptr) {
    out << "none";
  } else {
    out << "[";
    if (info.pos_side) {
      out << "pos";
    } else {
      out << "neg";
    }
    out << " " << info.node->split().normal() << "]";
  }
  return out;
}

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

#endif // WALNUT_BSP_NODE_H__
