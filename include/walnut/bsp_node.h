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

#include "walnut/convex_polygon.h"
#include "walnut/half_space3.h"

namespace walnut {

template <typename ConvexPolygonTemplate>
struct BSPEdgeInfo;

template <typename ConvexPolygonTemplate>
class BSPNode;

template <typename ConvexPolygonTemplate>
class BSPTree;

template <int point3_bits_template>
class BSPDefaultPolygon;

template <typename ConvexPolygonTemplate = BSPDefaultPolygon<32>>
class BSPEdgeInfo {
 public:
  using ConvexPolygonRep = ConvexPolygonTemplate;
  using BSPNodeRep = BSPNode<ConvexPolygonRep>;

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

template <int point3_bits>
class BSPDefaultPolygon :
  public ConvexPolygon<point3_bits,
                       BSPEdgeInfo<BSPDefaultPolygon<point3_bits>>> {
 public:
  using Parent = ConvexPolygon<point3_bits,
                               BSPEdgeInfo<BSPDefaultPolygon<point3_bits>>>;
  using typename Parent::SplitInfoRep;

  // Inherit all of the parent class's constructors.
  using Parent::Parent;

  // Overload CreateSplitChildren to create the derived polygon type.
  std::pair<BSPDefaultPolygon, BSPDefaultPolygon> CreateSplitChildren(
      const SplitInfoRep& split) const {
    std::pair<BSPDefaultPolygon, BSPDefaultPolygon> result;
    Parent::FillInSplitChildren(*this, split, result.first, result.second);
    return result;
  }

  // Overload CreateSplitChildren to create the derived polygon type.
  std::pair<BSPDefaultPolygon, BSPDefaultPolygon> CreateSplitChildren(
      SplitInfoRep&& split) && {
    std::pair<BSPDefaultPolygon, BSPDefaultPolygon> result;
    Parent::FillInSplitChildren(std::move(*this), std::move(split),
                                result.first, result.second);
    return result;
  }
};

// This is a node within a binary space partition tree.
//
// `ConvexPolygonTemplate` must inherit from ConvexPolygon.
// `ConvexPolygonTemplate::VertexData` must inherit from BSPEdgeInfo.
template <typename ConvexPolygonTemplate = BSPDefaultPolygon<32>>
class BSPNode {
 public:
  using ConvexPolygonRep = ConvexPolygonTemplate;
  static_assert(std::is_base_of<BSPEdgeInfo<ConvexPolygonRep>,
                                typename ConvexPolygonRep::VertexData>::value,
                "The ConvexPolygon's VertexData must inherit from "
                "BSPEdgeInfo.");
  static_assert(
      std::is_base_of<ConvexPolygon<ConvexPolygonRep::point3_bits,
                                    typename ConvexPolygonRep::VertexData>,
                      ConvexPolygonRep>::value,
      "The ConvexPolygonTemplate must inherit from ConvexPolygon.");

  using HalfSpace3Rep = typename HalfSpace3FromPoint3Builder<
    ConvexPolygonRep::point3_bits>::HalfSpace3Rep;
  friend class BSPTree<ConvexPolygonRep>;

  static constexpr int point3_bits = ConvexPolygonRep::point3_bits;

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

template <typename ConvexPolygonTemplate>
void BSPNode<ConvexPolygonTemplate>::Split(const HalfSpace3Rep& half_space) {
  assert(IsLeaf());

  split_ = half_space;
  positive_child_ = std::unique_ptr<BSPNode>(new BSPNode());
  negative_child_ = std::unique_ptr<BSPNode>(new BSPNode());

  PushContentsToChildren();
}

template <typename ConvexPolygonTemplate>
void BSPNode<ConvexPolygonTemplate>::PushContentsToChildren() {
  for (ConvexPolygonRep& polygon : contents_) {
    typename ConvexPolygonRep::SplitInfoRep info =
      polygon.GetSplitInfo(split_);

    if (info.ShouldEmitNegativeChild()) {
      if (info.ShouldEmitPositiveChild()) {
        std::pair<ConvexPolygonRep, ConvexPolygonRep> children =
          std::move(polygon).CreateSplitChildren(std::move(info));
        // As described by the CreateSplitChildren function declaration
        // comment, the last 2 vertices of neg_poly will touch the plane. So
        // the first of those 2 vertices is the edge source.
        children.first.vertex_data(
            children.first.vertex_count() - 2).set_split_by(this);
        // The first and last vertices of pos_poly will touch the plane. So the
        // first of those 2 vertices is the edge source.
        children.second.vertex_data(
            children.second.vertex_count() - 1).set_split_by(this);
        negative_child_->contents_.push_back(std::move(children.first));
        positive_child_->contents_.push_back(std::move(children.second));
      } else {
        negative_child_->contents_.push_back(std::move(polygon));
      }
    } else if (info.ShouldEmitPositiveChild()) {
      positive_child_->contents_.push_back(std::move(polygon));
    } else {
      assert(info.ShouldEmitOnPlane());
      // If polygon.plane().normal() and split_.normal() point in the same
      // direction, put polygon in the negative child.
      const int drop_dimension = polygon.drop_dimension();
      if (polygon.plane().normal().components()[drop_dimension].GetAbsMult(
            split_.normal().components()[drop_dimension]) >= 0) {
        negative_child_->border_contents_.push_back(std::move(polygon));
      } else {
        positive_child_->border_contents_.push_back(std::move(polygon));
      }
    }
  }

  for (ConvexPolygonRep& polygon : border_contents_) {
    typename ConvexPolygonRep::SplitInfoRep info =
      polygon.GetSplitInfo(split_);

    if (info.ShouldEmitNegativeChild()) {
      if (info.ShouldEmitPositiveChild()) {
        std::pair<ConvexPolygonRep, ConvexPolygonRep> children =
          std::move(polygon).CreateSplitChildren(std::move(info));
        // As described by the CreateSplitChildren function declaration
        // comment, the last 2 vertices of neg_poly will touch the plane. So
        // the first of those 2 vertices is the edge source.
        children.first.vertex_data(
            children.first.vertex_count() - 2).set_split_by(this);
        // The first and last vertices of pos_poly will touch the plane. So the
        // first of those 2 vertices is the edge source.
        children.second.vertex_data(
            children.second.vertex_count() - 1).set_split_by(this);
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
      if (polygon.plane().normal().components()[drop_dimension].GetAbsMult(
            split_.normal().components()[drop_dimension]) >= 0) {
        negative_child_->border_contents_.push_back(std::move(polygon));
      } else {
        positive_child_->border_contents_.push_back(std::move(polygon));
      }
    }
  }
}

template <typename ConvexPolygonTemplate>
template <typename LeafCallback>
void BSPNode<ConvexPolygonTemplate>::PushContentsToLeaves(
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

template <typename ConvexPolygonTemplate>
std::ostream& operator<<(std::ostream& out,
                         const BSPEdgeInfo<ConvexPolygonTemplate>& info) {
  out << "split_by=" << info.split_by();
  return out;
}

}  // walnut

#endif // WALNUT_BSP_NODE_H__
