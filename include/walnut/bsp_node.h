#ifndef WALNUT_BSP_NODE_H__
#define WALNUT_BSP_NODE_H__

// for std::unique_ptr
#include <memory>
// for std::pair and std::move
#include <utility>
#include <vector>

#include "walnut/aabb_convex_polygon.h"
#include "walnut/bsp_edge_info.h"
#include "walnut/bsp_polygon.h"
#include "walnut/half_space3.h"
#include "walnut/r_transformation.h"

namespace walnut {

// This is a node within a binary space partition tree.
//
// `OutputPolygonParentTemplate` must inherit from ConvexPolygon.
template <typename OutputPolygonParentTemplate = AABBConvexPolygon<>>
class BSPNode {
 public:
  using OutputPolygonParent = OutputPolygonParentTemplate;
  using PolygonRep = BSPPolygon<BSPNode, OutputPolygonParent>;
  using EdgeParent = typename PolygonRep::EdgeParent;
  using EdgeRep = typename PolygonRep::EdgeRep;
  using BSPEdgeInfoRep = typename PolygonRep::BSPEdgeInfoRep;
  using BSPNodeSideRep = typename BSPEdgeInfoRep::BSPNodeSideRep;

  BSPNode() = default;

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
    MakeInterior(half_space, new BSPNode(), new BSPNode());
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

  int64_t GetPWNForId(BSPContentId id) const {
    if (id < pwn_by_id_.size()) {
      return pwn_by_id_[id];
    } else {
      return 0;
    }
  }

  // Push the contents of an interior node to the children.
  //
  // This is a no-op for leaves.
  void PushContentsToChildren();

  // Push the contents all the way down to descendant leaf nodes. Call
  // `leaf_callback` on each leaf node that the contents were pushed to (and
  // possibly more nodes that the contents were not pushed to).
  //
  // This may be called on leaf or interior nodes.
  template <typename LeafCallback>
  void PushContentsToLeaves(LeafCallback leaf_callback);

 protected:
  void MakeInterior(const HalfSpace3& half_space,
                    BSPNode* negative_child,
                    BSPNode* positive_child) {
    assert(IsLeaf());
    split_ = half_space;
    negative_child_ = std::unique_ptr<BSPNode>(negative_child);
    positive_child_ = std::unique_ptr<BSPNode>(positive_child);

    negative_child->SetPWN(pwn_by_id_);
    positive_child->SetPWN(pwn_by_id_);
  }

  void Reset() {
    contents_.clear();
    border_contents_.clear();
    split_ = HalfSpace3();
    negative_child_.reset();
    positive_child_.reset();
    pwn_by_id_.clear();
  }

  // Adds a polygon to a root node.
  //
  // Even if the node is an interior node, the polygon is not pushed to the
  // child nodes. PushContentsToChildren should be called afterwards if
  // necessary to do so.
  template <typename InputConvexPolygon>
  void AddRootContent(BSPContentId id, InputConvexPolygon&& polygon) {
    contents_.emplace_back(id, /*on_node_plane=*/nullptr, /*pos_side=*/false,
                           std::forward<InputConvexPolygon>(polygon));
    using InputConvexPolygonNoRef =
      typename std::remove_reference<InputConvexPolygon>::type;
    using InputEdgeParent = typename InputConvexPolygonNoRef::EdgeParent;
    if (std::is_base_of<EdgeParent, InputEdgeParent>::value) {
      contents_.back().ResetBSPInfo();
    }
  }

 private:
  template <typename OutputPolygonParent>
  friend class BSPTree;

  void SetPWN(const std::vector<int64_t>& pwn_by_id) {
    pwn_by_id_ = pwn_by_id;
  }

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
  void PushVertexPWNToChildren(BSPContentId polygon_id, int edge_comparison,
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
    BSPContentId polygon_id, int edge_comparison,
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
  if (IsLeaf()) {
    return;
  }
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

}  // walnut

#endif // WALNUT_BSP_NODE_H__
