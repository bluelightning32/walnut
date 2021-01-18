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
#include "walnut/greatest_angle_tracker.h"
#include "walnut/half_space3.h"

namespace walnut {

template <typename BSPNodeTemplate, typename NormalRepTemplate>
class BSPEdgeInfo;

template <typename BSPNodeTemplate>
class BSPNode;

template <typename InputPolygonTemplate>
class BSPTree;

template <size_t point3_bits_template>
class BSPDefaultPolygon;

using BSPPolygonId = size_t;

template <typename BSPNodeTemplate, typename NormalRepTemplate>
class BSPEdgeInfo {
 public:
  using BSPNodeRep = BSPNodeTemplate;
  using NormalRep = NormalRepTemplate;

  BSPEdgeInfo(const NoVertexData&) { }

  // Create a new vertex on the parent's existing edge.
  //
  // Inherit the edge trackers from the parent, but initialize the vertex
  // tracker from the edge trackers.
  template <size_t num_bits, size_t denom_bits>
  BSPEdgeInfo(const BSPEdgeInfo& parent,
              const HomoPoint3<num_bits, denom_bits>& new_source) :
    split_by(parent.split_by),
    cw_edge_angle_tracker_(parent.cw_edge_angle_tracker_),
    vertex_angle_tracker_(parent.cw_edge_angle_tracker_) { }

  // Create a new line from the parent's existing vertex.
  //
  // Inherit the vertex trackers from the parent, but default initialize the
  // edge trackers.
  template <size_t d_bits, size_t m_bits>
  BSPEdgeInfo(const BSPEdgeInfo& parent,
              const PluckerLine<d_bits, m_bits>& new_line) :
    vertex_angle_tracker_(parent.vertex_angle_tracker_) { }

  // Create a new line starting on a new vertex on the parent's existing edge.
  //
  // Initialize the vertex tracker from the parent's edge trackers. Default
  // initialize the edge trackers.
  template <size_t num_bits, size_t denom_bits, size_t d_bits, size_t m_bits>
  BSPEdgeInfo(const BSPEdgeInfo& parent,
              const HomoPoint3<num_bits, denom_bits>& new_source,
              const PluckerLine<d_bits, m_bits>& new_line) :
    vertex_angle_tracker_(parent.cw_edge_angle_tracker_) { }

  bool operator==(const NoVertexData&) const {
    return true;
  }
  bool operator!=(const NoVertexData&) const {
    return false;
  }

  const GreatestAngleTracker</*most_ccw=*/false, NormalRep::component_bits>&
  cw_edge_angle_tracker() const {
    return cw_edge_angle_tracker_;
  }

  const GreatestAngleTracker</*most_ccw=*/false, NormalRep::component_bits>&
  vertex_angle_tracker() const {
    return vertex_angle_tracker_;
  }

  const BSPNodeRep* split_by = nullptr;

 private:
  friend BSPNodeRep;

  GreatestAngleTracker</*most_ccw=*/false, NormalRep::component_bits>
    cw_edge_angle_tracker_;

  GreatestAngleTracker</*most_ccw=*/false, NormalRep::component_bits>
    vertex_angle_tracker_;
};

template <size_t point3_bits>
class BSPDefaultPolygon :
  public ConvexPolygon<point3_bits,
    BSPEdgeInfo<BSPNode<BSPDefaultPolygon<point3_bits>>,
                typename ConvexPolygon<point3_bits>::NormalRep>> {
 public:
  using NormalRep = typename ConvexPolygon<point3_bits>::NormalRep;
  using Parent =
    ConvexPolygon<point3_bits,
                  BSPEdgeInfo<BSPNode<BSPDefaultPolygon<point3_bits>>,
                              NormalRep>>;
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
  using typename Parent::NormalRep;
  using typename Parent::SplitInfoRep;
  using typename Parent::VertexData;

  static_assert(std::is_base_of<BSPEdgeInfo<BSPNodeRep, NormalRep>,
                                VertexData>::value,
                "The ConvexPolygon's VertexData must inherit from "
                "BSPEdgeInfo.");
  static_assert(std::is_base_of<ConvexPolygon<Parent::point3_bits, VertexData>,
                                Parent>::value,
      "The InputPolygonTemplate must inherit from ConvexPolygon.");

  template <typename OtherPolygon>
  BSPPolygonWrapper(BSPPolygonId id, const BSPNodeRep* on_node_plane,
                    OtherPolygon&& parent) :
    Parent(std::forward<OtherPolygon>(parent)), id(id), 
    on_node_plane(on_node_plane) { }

  BSPPolygonWrapper(const BSPNodeRep* on_node_plane,
                    const BSPPolygonWrapper& parent) :
    Parent(parent), id(parent.id), 
    on_node_plane(on_node_plane) { }

  BSPPolygonWrapper(const BSPNodeRep* on_node_plane,
                    BSPPolygonWrapper&& parent) :
    Parent(std::move(parent)), id(parent.id), 
    on_node_plane(on_node_plane) { }

  // Overload CreateSplitChildren to create the derived polygon type.
  std::pair<BSPPolygonWrapper, BSPPolygonWrapper> CreateSplitChildren(
      const SplitInfoRep& split) const {
    std::pair<Parent, Parent> parent_result =
      Parent::CreateSplitChildren(split);
    return std::make_pair(BSPPolygonWrapper(id, on_node_plane,
                                            std::move(parent_result.first)),
                          BSPPolygonWrapper(id, on_node_plane,
                                            std::move(parent_result.second)));
  }

  // Overload CreateSplitChildren to create the derived polygon type.
  std::pair<BSPPolygonWrapper, BSPPolygonWrapper> CreateSplitChildren(
      SplitInfoRep&& split) && {
    std::pair<Parent, Parent> parent_result =
      static_cast<Parent&&>(*this).CreateSplitChildren(std::move(split));
    return std::make_pair(BSPPolygonWrapper(id, on_node_plane,
                                            std::move(parent_result.first)),
                          BSPPolygonWrapper(id, on_node_plane,
                                            std::move(parent_result.second)));
  }

  BSPPolygonId id;

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
  using VertexData = typename PolygonRep::VertexData;
  using EdgeRep = typename PolygonRep::EdgeRep;

  using HalfSpace3Rep = typename HalfSpace3FromPoint3Builder<
    InputPolygon::point3_bits>::HalfSpace3Rep;
  friend class BSPTree<InputPolygon>;

  static constexpr int point3_bits = InputPolygon::point3_bits;

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
  void Split(const HalfSpace3Rep& half_space) {
    assert(half_space.IsValid());
    MakeInterior(half_space, new BSPNode(pwn_by_id_), new BSPNode(pwn_by_id_));
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

  // Update the children's PWN based on this node's contents.
  //
  // This may only be called on an interior node.
  void PushContentPWNToChildren();

  // Update the angle trackers in the edges and vertices of `polygon` that are
  // coincident with `split_`.
  //
  // If `pos_child_` is false, the trackers are updated with split_.normal(),
  // otherwise they are updated with -split_.normal().
  //
  // The vertices in the range [coincident_begin, coincident_end) are updated.
  // The edges in the range [coincident_begin, coincident_end - 1) are updated.
  //
  // The caller must ensure coincident_begin <= coincident_end. The function
  // will apply the modulus on the vertex indices, so it is okay for
  // coincident_end to be greater than polygon.vertex_count().
  void UpdateAngleTrackers(bool pos_child, PolygonRep& polygon,
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
  HalfSpace3Rep split_;

  std::unique_ptr<BSPNode> negative_child_;
  std::unique_ptr<BSPNode> positive_child_;

  std::vector<int64_t> pwn_by_id_;
};

template <typename InputPolygonTemplate>
void BSPNode<InputPolygonTemplate>::PushContentPWNToChildren() {
  for (PolygonRep& polygon : contents_) {
    for (size_t i = 0; i < polygon.vertex_count(); ++i) {
      const EdgeRep& current_edge = polygon.edge(i);
      const VertexData& current_vertex_data = current_edge.data();
      int edge_comparison = RXYCompareBivector(split_.normal(),
          current_vertex_data.cw_edge_angle_tracker_.current());
      if (edge_comparison == 0) continue;

      const EdgeRep& next_edge =
        polygon.edge((i + 1)%polygon.vertex_count());
      const VertexData& next_vertex_data = next_edge.data();
      const int before_vertex_comparison = RXYCompareBivector(split_.normal(),
          current_vertex_data.vertex_angle_tracker_.current());
      const int after_vertex_comparison = RXYCompareBivector(split_.normal(),
          next_vertex_data.vertex_angle_tracker_.current());

      BSPNode<InputPolygonTemplate>* push_to_child;
      int change = 0;
      if (edge_comparison < 0) {
        push_to_child = negative_child();
        if (before_vertex_comparison > 0 &&
            split_.Compare(current_edge.vertex) >= 0) {
          BigIntWord min_max_comparison =
            split_.normal().Dot(
                current_vertex_data.cw_edge_angle_tracker_.current().Cross(
                  current_vertex_data.vertex_angle_tracker_.current()))
            .GetSign();
          if (min_max_comparison < 0) {
            // I-path is entering the polyhedron.
            ++change;
          }
        }
        if (after_vertex_comparison > 0 &&
            split_.Compare(next_edge.vertex) >= 0) {
          BigIntWord min_max_comparison =
            split_.normal().Dot(
                current_vertex_data.cw_edge_angle_tracker_.current().Cross(
                  next_vertex_data.vertex_angle_tracker_.current())).GetSign();
          if (min_max_comparison < 0) {
            // I-path is exiting the polyhedron.
            --change;
          }
        }
      } else {
        push_to_child = positive_child();
        if (before_vertex_comparison < 0 &&
            split_.Compare(current_edge.vertex) <= 0) {
          BigIntWord min_max_comparison =
            split_.normal().Dot(
                current_vertex_data.cw_edge_angle_tracker_.current().Cross(
                  current_vertex_data.vertex_angle_tracker_.current()))
            .GetSign();
          if (min_max_comparison > 0) {
            // I-path is entering the polyhedron.
            ++change;
          }
        }
        if (after_vertex_comparison < 0 &&
            split_.Compare(next_edge.vertex) <= 0) {
          BigIntWord min_max_comparison =
            split_.normal().Dot(
                current_vertex_data.cw_edge_angle_tracker_.current().Cross(
                  next_vertex_data.vertex_angle_tracker_.current())).GetSign();
          if (min_max_comparison > 0) {
            // I-path is exiting the polyhedron.
            --change;
          }
        }
      }
      if (push_to_child->pwn_by_id_.size() <= polygon.id) {
        push_to_child->pwn_by_id_.resize(polygon.id + 1);
      }
      push_to_child->pwn_by_id_[polygon.id] += change;
    }
  }
}

template <typename InputPolygonTemplate>
void BSPNode<InputPolygonTemplate>::UpdateAngleTrackers(
    bool pos_child, PolygonRep& polygon, size_t coincident_begin,
    size_t coincident_end) {
  // Typically this function is called with 0 vertices to update. So quickly
  // handle that case first.
  if (coincident_begin == coincident_end) return;

  auto normal = pos_child ? -split_.normal() : split_.normal();
  size_t pos = coincident_begin;
  // Edges go from source to target. So first loop through all of the edges
  // that need to be updated, and update their corresponding source vertices
  // along the way too.
  for (; pos < coincident_end - 1; ++pos) {
    VertexData& vertex_data = polygon.vertex_data(pos % polygon.vertex_count());
    vertex_data.cw_edge_angle_tracker_.Receive(normal);
    vertex_data.vertex_angle_tracker_.Receive(normal);
  }
  // Update the last target vertex.
  VertexData& vertex_data = polygon.vertex_data(pos % polygon.vertex_count());
  vertex_data.vertex_angle_tracker_.Receive(normal);
}

template <typename InputPolygonTemplate>
void BSPNode<InputPolygonTemplate>::PushContentsToChildren() {
  PushContentPWNToChildren();

  for (PolygonRep& polygon : contents_) {
    assert(polygon.vertex_count() > 0);
    typename PolygonRep::SplitInfoRep info = polygon.GetSplitInfo(split_);
    assert(info.IsValid());

    if (info.ShouldEmitNegativeChild()) {
      if (info.ShouldEmitPositiveChild()) {
        std::pair<PolygonRep, PolygonRep> children =
          std::move(polygon).CreateSplitChildren(std::move(info));
        assert(children.first.vertex_count() > 2);
        assert(children.second.vertex_count() > 2);
        // As described by the CreateSplitChildren function declaration
        // comment, the last 2 vertices of neg_poly will touch the plane. So
        // the first of those 2 vertices is the edge source.
        children.first.vertex_data(
            children.first.vertex_count() - 2).split_by = this;
        UpdateAngleTrackers(/*pos_child=*/false, children.first,
                            children.first.vertex_count() - 2,
                            children.first.vertex_count());
        // The first and last vertices of pos_poly will touch the plane. So the
        // first of those 2 vertices is the edge source.
        children.second.vertex_data(
            children.second.vertex_count() - 1).split_by = this;
        UpdateAngleTrackers(/*pos_child=*/true, children.second,
                            children.second.vertex_count() - 1,
                            children.second.vertex_count() + 1);

        negative_child_->contents_.push_back(std::move(children.first));
        positive_child_->contents_.push_back(std::move(children.second));
      } else {
        UpdateAngleTrackers(/*pos_child=*/false, polygon,
                            info.neg_range().second,
                            polygon.vertex_count() + info.neg_range().first);
        negative_child_->contents_.push_back(std::move(polygon));
      }
    } else if (info.ShouldEmitPositiveChild()) {
      UpdateAngleTrackers(/*pos_child=*/true, polygon,
                          info.pos_range().second,
                          polygon.vertex_count() + info.pos_range().first);
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
    assert(polygon.vertex_count() > 0);
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

template <typename InputPolygonTemplate, typename NormalRep>
std::ostream& operator<<(
    std::ostream& out,
    const BSPEdgeInfo<InputPolygonTemplate, NormalRep>& info) {
  out << "< split_by=" << info.split_by
      << ", cw_edge_angle_tracker=" << info.cw_edge_angle_tracker().current()
      << ", vertex_angle_tracker=" << info.vertex_angle_tracker().current()
      << " >";
  return out;
}

}  // walnut

#endif // WALNUT_BSP_NODE_H__
