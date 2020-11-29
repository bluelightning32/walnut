#ifndef WALNUT_BSP_NODE_H__
#define WALNUT_BSP_NODE_H__

#include <memory>
#include <ostream>

#include "walnut/convex_polygon.h"
#include "walnut/half_space3.h"

namespace walnut {

template <int point3_bits_template = 32>
class BSPNode;

template <int point3_bits_template>
class BSPTree;

template <int point3_bits_template = 32>
class BSPEdgeInfo {
 public:
  using BSPNodeRep = BSPNode<point3_bits_template>;

  BSPEdgeInfo() = default;
  BSPEdgeInfo(const NoVertexData&) { }

  bool operator==(const NoVertexData&) const {
    return true;
  }
  bool operator!=(const NoVertexData&) const {
    return false;
  }

  BSPNodeRep* split_by() {
    return split_by_;
  }

  const BSPNodeRep* split_by() const {
    return split_by_;
  }

  void set_split_by(BSPNodeRep* split_by) {
    split_by_ = split_by;
  }

 private:
  BSPNodeRep* split_by_ = nullptr;
};

// This is a node within a binary space partition tree.
template <int point3_bits_template>
class BSPNode {
 public:
  using EdgeInfo = BSPEdgeInfo<point3_bits_template>;
  using ConvexPolygonRep = ConvexPolygon<point3_bits_template, EdgeInfo>;
  using HalfSpace3Rep =
    typename HalfSpace3FromPoint3Builder<point3_bits_template>::HalfSpace3Rep;
  friend class BSPTree<point3_bits_template>;

  static constexpr int point3_bits = point3_bits_template;

  // Convert a leaf node into an interior node.
  //
  // This may only be called on a leaf node.
  //
  // The plane of `half_space` must be different from the any of the planes
  // that ancestor nodes were split on.
  //
  // The contents of this node will be pushed into the new child nodes.
  void Split(const HalfSpace3Rep& half_space);

  bool IsLeaf() const {
    return !split().IsValid();
  }

  const HalfSpace3Rep& split() const {
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

  const std::vector<ConvexPolygonRep>& contents() const {
    return contents_;
  }

  const std::vector<ConvexPolygonRep>& border_contents() const {
    return border_contents_;
  }

 private:
  // Push the contents of an interior node to the children.
  //
  // This may only be called on an interior node.
  void PushContentsToChildren();

  // Push the contents all the way down to descendant leaf nodes. Call
  // `leaf_callback` on each leaf node that the contents were pushed to (and
  // possibly more nodes that the contents were not pushed to).
  //
  // This may be called on leaf or interior nodes.
  template <typename LeafCallback>
  void PushContentsToLeaves(LeafCallback leaf_callback);

  // For a leaf node, these are the polygons that are inside the cell. They may
  // possibly be touching the cell border, but they are not on the border.
  //
  // For a finished interior node, this will be empty. When new contents are
  // added to the tree, this will be temporarily non-empty, until the new
  // contents are psuhed to the children.
  std::vector<ConvexPolygonRep> contents_;

  // For a leaf node, these are the polygons that are on the cell border.
  //
  // For a finished interior node, this will be empty. When new contents are
  // added to the tree, this will be temporarily non-empty, until the new
  // contents are psuhed to the children.
  std::vector<ConvexPolygonRep> border_contents_;

  // The plane that splits an interior node.
  //
  // For a leaf node, !split_.IsValid().
  HalfSpace3Rep split_;

  std::unique_ptr<BSPNode> negative_child_;
  std::unique_ptr<BSPNode> positive_child_;
};

template <int point3_bits>
void BSPNode<point3_bits>::Split(const HalfSpace3Rep& half_space) {
  assert(IsLeaf());

  split_ = half_space;
  positive_child_ = std::unique_ptr<BSPNode>(new BSPNode());
  negative_child_ = std::unique_ptr<BSPNode>(new BSPNode());

  PushContentsToChildren();
}

template <int point3_bits>
void BSPNode<point3_bits>::PushContentsToChildren() {
  for (ConvexPolygonRep& polygon : contents_) {
    typename ConvexPolygonRep::template SplitKey<ConvexPolygonRep&&> key =
      std::move(polygon).GetSplitKey(split_);

    if (key.ShouldEmitNegativeChild()) {
      if (key.ShouldEmitPositiveChild()) {
        negative_child_->contents_.emplace_back();
        positive_child_->contents_.emplace_back();
        ConvexPolygonRep& neg_poly = negative_child_->contents_.back();
        ConvexPolygonRep& pos_poly = positive_child_->contents_.back();
        key.CreateSplitChildren(neg_poly, pos_poly);
        // As described by the CreateSplitChildren function declaration
        // comment, the last 2 vertices of neg_poly will touch the plane. So
        // the first of those 2 vertices is the edge source.
        neg_poly.vertex_data(neg_poly.vertex_count() - 2).set_split_by(this);
        // The first and last vertices of pos_poly will touch the plane. So the
        // first of those 2 vertices is the edge source.
        pos_poly.vertex_data(neg_poly.vertex_count() - 1).set_split_by(this);
      } else {
        negative_child_->contents_.push_back(key.CreateNegativeChild());
      }
    } else if (key.ShouldEmitPositiveChild()) {
      positive_child_->contents_.push_back(key.CreatePositiveChild());
    } else {
      assert(key.ShouldEmitOnPlane());
      // If polygon.plane().normal() and split_.normal() point in the same
      // direction, put polygon in the negative child.
      const int drop_dimension = polygon.drop_dimension();
      if (polygon.plane().normal().components()[drop_dimension].GetAbsMult(
            split_.normal().components()[drop_dimension]) >= 0) {
        negative_child_->border_contents_.push_back(key.CreateOnPlaneChild());
      } else {
        positive_child_->border_contents_.push_back(key.CreateOnPlaneChild());
      }
    }
  }

  for (ConvexPolygonRep& polygon : border_contents_) {
    typename ConvexPolygonRep::template SplitKey<ConvexPolygonRep&&> key =
      std::move(polygon).GetSplitKey(split_);

    if (key.ShouldEmitNegativeChild()) {
      if (key.ShouldEmitPositiveChild()) {
        negative_child_->border_contents_.emplace_back();
        positive_child_->border_contents_.emplace_back();
        ConvexPolygonRep& neg_poly = negative_child_->border_contents_.back();
        ConvexPolygonRep& pos_poly = positive_child_->border_contents_.back();
        key.CreateSplitChildren(neg_poly, pos_poly);
        // As described by the CreateSplitChildren function declaration
        // comment, the last 2 vertices of neg_poly will touch the plane. So
        // the first of those 2 vertices is the edge source.
        neg_poly.vertex_data(neg_poly.vertex_count() - 2).set_split_by(this);
        // The first and last vertices of pos_poly will touch the plane. So the
        // first of those 2 vertices is the edge source.
        pos_poly.vertex_data(neg_poly.vertex_count() - 1).set_split_by(this);
      } else {
        negative_child_->border_contents_.push_back(key.CreateNegativeChild());
      }
    } else if (key.ShouldEmitPositiveChild()) {
      positive_child_->border_contents_.push_back(key.CreatePositiveChild());
    } else {
      assert(key.ShouldEmitOnPlane());
      // This branch only runs if this node is being split on the same plane as
      // an ancestor node, which should not happen.
      assert(false);
      // If polygon.plane().normal() and split_.normal() point in the same
      // direction, put polygon in the negative child.
      const int drop_dimension = polygon.drop_dimension();
      if (polygon.plane().normal().components()[drop_dimension].GetAbsMult(
            split_.normal().components()[drop_dimension]) >= 0) {
        negative_child_->border_contents_.push_back(key.CreateOnPlaneChild());
      } else {
        positive_child_->border_contents_.push_back(key.CreateOnPlaneChild());
      }
    }
  }
}

template <int point3_bits>
template <typename LeafCallback>
void BSPNode<point3_bits>::PushContentsToLeaves(LeafCallback leaf_callback) {
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

template <int point3_bits_template>
std::ostream& operator<<(std::ostream& out,
                         const BSPEdgeInfo<point3_bits_template>& info) {
  out << "split_by=" << info.split_by();
  return out;
}

}  // walnut

#endif // WALNUT_BSP_NODE_H__
