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

template <typename InputPolygonTemplate>
struct BSPEdgeInfo;

template <typename BSPNodeTemplate>
class BSPNode;

template <typename InputPolygonTemplate>
class BSPTree;

template <int point3_bits_template>
class BSPDefaultPolygon;

template <typename BSPNodeTemplate>
struct BSPEdgeInfo {
 public:
  using BSPNodeRep = BSPNodeTemplate;

  BSPEdgeInfo() = default;
  BSPEdgeInfo(const NoVertexData&) { }

  bool operator==(const NoVertexData&) const {
    return true;
  }
  bool operator!=(const NoVertexData&) const {
    return false;
  }

  const BSPNodeRep* split_by = nullptr;
};

template <int point3_bits>
class BSPDefaultPolygon :
  public ConvexPolygon<point3_bits,
                       BSPEdgeInfo<BSPNode<BSPDefaultPolygon<point3_bits>>>> {
 public:
  using Parent =
    ConvexPolygon<point3_bits,
                  BSPEdgeInfo<BSPNode<BSPDefaultPolygon<point3_bits>>>>;
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

template <typename BSPNodeTemplate>
class BSPPolygonWrapper : public BSPNodeTemplate::InputPolygon {
 public:
  using BSPNodeRep = BSPNodeTemplate;
  using Parent = typename BSPNodeTemplate::InputPolygon;
  using typename Parent::SplitInfoRep;
  using typename Parent::VertexData;

  static_assert(std::is_base_of<BSPEdgeInfo<BSPNodeRep>, VertexData>::value,
                "The ConvexPolygon's VertexData must inherit from "
                "BSPEdgeInfo.");
  static_assert(std::is_base_of<ConvexPolygon<Parent::point3_bits, VertexData>,
                                Parent>::value,
      "The InputPolygonTemplate must inherit from ConvexPolygon.");

  template <typename OtherPolygon>
  BSPPolygonWrapper(const BSPNodeRep* on_node_plane, OtherPolygon&& parent) :
    Parent(std::forward<OtherPolygon>(parent)),
    on_node_plane(on_node_plane) { }

  // Overload CreateSplitChildren to create the derived polygon type.
  std::pair<BSPPolygonWrapper, BSPPolygonWrapper> CreateSplitChildren(
      const SplitInfoRep& split) const {
    std::pair<Parent, Parent> parent_result =
      Parent::CreateSplitChildren(split);
    return std::make_pair(BSPPolygonWrapper(on_node_plane,
                                            std::move(parent_result.first)),
                          BSPPolygonWrapper(on_node_plane,
                                            std::move(parent_result.second)));
  }

  // Overload CreateSplitChildren to create the derived polygon type.
  std::pair<BSPPolygonWrapper, BSPPolygonWrapper> CreateSplitChildren(
      SplitInfoRep&& split) && {
    std::pair<Parent, Parent> parent_result =
      static_cast<Parent&&>(*this).CreateSplitChildren(std::move(split));
    return std::make_pair(BSPPolygonWrapper(on_node_plane,
                                            std::move(parent_result.first)),
                          BSPPolygonWrapper(on_node_plane,
                                            std::move(parent_result.second)));
  }

  // This is the BSPNode whose split plane is coincident with this polygon's
  // plane, or nullptr if no such BSPNode exists.
  const BSPNodeRep* on_node_plane = nullptr;
};

// This is a node within a binary space partition tree.
//
// `InputPolygonTemplate` must inherit from ConvexPolygon.
// `InputPolygonTemplate::VertexData` must inherit from BSPEdgeInfo.
template <typename InputPolygonTemplate = BSPDefaultPolygon<32>>
class BSPNode {
 public:
  using InputPolygon = InputPolygonTemplate;
  using PolygonRep = BSPPolygonWrapper<BSPNode>;

  using HalfSpace3Rep = typename HalfSpace3FromPoint3Builder<
    InputPolygon::point3_bits>::HalfSpace3Rep;
  friend class BSPTree<InputPolygon>;

  static constexpr int point3_bits = InputPolygon::point3_bits;

  // Convert a leaf node into an interior node.
  //
  // This may only be called on a leaf node.
  //
  // The plane of `half_space` must be different from the any of the planes
  // that ancestor nodes were split on.
  //
  // The contents of this node will be pushed into the new child nodes.
  void Split(const HalfSpace3Rep& half_space) {
    MakeInterior(half_space, new BSPNode(), new BSPNode());
    PushContentsToChildren();
  }

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

  const std::vector<PolygonRep>& contents() const {
    return contents_;
  }

  const std::vector<PolygonRep>& border_contents() const {
    return border_contents_;
  }

 protected:
  // Push the contents of an interior node to the children.
  //
  // This may only be called on an interior node.
  void PushContentsToChildren();

  void MakeInterior(const HalfSpace3Rep& half_space,
                    BSPNode* negative_child,
                    BSPNode* positive_child) {
    assert(IsLeaf());
    split_ = half_space;
    negative_child_ = std::unique_ptr<BSPNode>(negative_child);
    positive_child_ = std::unique_ptr<BSPNode>(positive_child);
  }

 private:
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
  HalfSpace3Rep split_;

  std::unique_ptr<BSPNode> negative_child_;
  std::unique_ptr<BSPNode> positive_child_;
};

template <typename InputPolygonTemplate>
void BSPNode<InputPolygonTemplate>::PushContentsToChildren() {
  for (PolygonRep& polygon : contents_) {
    typename PolygonRep::SplitInfoRep info = polygon.GetSplitInfo(split_);

    if (info.ShouldEmitNegativeChild()) {
      if (info.ShouldEmitPositiveChild()) {
        std::pair<PolygonRep, PolygonRep> children =
          std::move(polygon).CreateSplitChildren(std::move(info));
        // As described by the CreateSplitChildren function declaration
        // comment, the last 2 vertices of neg_poly will touch the plane. So
        // the first of those 2 vertices is the edge source.
        children.first.vertex_data(
            children.first.vertex_count() - 2).split_by = this;
        // The first and last vertices of pos_poly will touch the plane. So the
        // first of those 2 vertices is the edge source.
        children.second.vertex_data(
            children.second.vertex_count() - 1).split_by = this;
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
        negative_child_->border_contents_.emplace_back(this,
                                                       std::move(polygon));
      } else {
        positive_child_->border_contents_.emplace_back(this,
                                                       std::move(polygon));
      }
    }
  }

  for (PolygonRep& polygon : border_contents_) {
    typename InputPolygon::SplitInfoRep info = polygon.GetSplitInfo(split_);

    if (info.ShouldEmitNegativeChild()) {
      if (info.ShouldEmitPositiveChild()) {
        std::pair<PolygonRep, PolygonRep> children =
          std::move(polygon).CreateSplitChildren(std::move(info));
        // As described by the CreateSplitChildren function declaration
        // comment, the last 2 vertices of neg_poly will touch the plane. So
        // the first of those 2 vertices is the edge source.
        children.first.vertex_data(
            children.first.vertex_count() - 2).split_by = this;
        // The first and last vertices of pos_poly will touch the plane. So the
        // first of those 2 vertices is the edge source.
        children.second.vertex_data(
            children.second.vertex_count() - 1).split_by = this;
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

template <typename InputPolygonTemplate>
template <typename LeafCallback>
void BSPNode<InputPolygonTemplate>::PushContentsToLeaves(
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

template <typename InputPolygonTemplate>
std::ostream& operator<<(std::ostream& out,
                         const BSPEdgeInfo<InputPolygonTemplate>& info) {
  out << "split_by=" << info.split_by;
  return out;
}

}  // walnut

#endif // WALNUT_BSP_NODE_H__
