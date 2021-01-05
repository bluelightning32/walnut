#ifndef WALNUT_BSP_TREE_H__
#define WALNUT_BSP_TREE_H__

#include <type_traits>

#include "walnut/bsp_node.h"
#include "walnut/rectangular_prism.h"

namespace walnut {

// InputPolygonTemplate must have a VertexData that inherits from BSPEdgeInfo.
template <typename InputPolygonTemplate = BSPDefaultPolygon<32>>
class BSPTree {
 public:
  using InputPolygon = InputPolygonTemplate;
  using BSPNodeRep = BSPNode<InputPolygon>;
  using OutputPolygon = typename BSPNodeRep::PolygonRep;
  using VertexData = typename BSPNodeRep::VertexData;

  static constexpr int point3_bits = InputPolygon::point3_bits;

  // Add a new polygon to this node.
  //
  // For an interior node, the contents will be pushed to the children.
  // `leaf_callback` will be called for every leaf node that some pieces of the
  // new contents settle in. There may also be spurious calls to
  // `leaf_callback` for leaves where the contents did not land in.
  template <typename InputConvexPolygon, typename LeafCallback>
  void AddContent(InputConvexPolygon&& polygon, BSPPolygonId id,
                  LeafCallback leaf_callback) {
    using InputConvexPolygonNoRef =
      typename std::remove_reference<InputConvexPolygon>::type;
    using InputVertexData = typename InputConvexPolygonNoRef::VertexData;
    if (std::is_base_of<VertexData, InputVertexData>::value) {
      // The input polygon came from a different BSPTree. Make a copy of the
      // input polygon that does not have the vertex and edge trackers from the
      // previous tree.
      ConvexPolygon<InputConvexPolygonNoRef::point3_bits> stripped(
          std::forward<InputConvexPolygon>(polygon));
      root.contents_.emplace_back(id, /*on_node_plane=*/nullptr,
                                  std::move(stripped));
    } else {
      root.contents_.emplace_back(id, /*on_node_plane=*/nullptr,
                                  std::forward<InputConvexPolygon>(polygon));
    }
    root.PushContentsToLeaves(leaf_callback);
  }

  template <typename InputConvexPolygon, typename LeafCallback>
  void AddContent(InputConvexPolygon&& polygon, LeafCallback leaf_callback) {
    AddContent(std::forward<InputConvexPolygon>(polygon), AllocateId(),
               leaf_callback);
  }

  // Returns the polyhedron boundary of a BSPNode.
  //
  // The path to the BSPNode is specified as a sequence of bools, indicating
  // whether to go to the negative child (false) or the positive child (true).
  // The sequence of bools is specified through the iterators `node_path_begin`
  // and `node_path_end`.
  //
  // The result is a vector of ConvexPolygons that define the polyhedron of the
  // boundary. Each returned polygon indicates which BSPNode division it came
  // from.
  //
  // In case the BSPNode border is unbounded, `bounding_box` provides an upper
  // bound for how far the ConvexPolygons will extend.
  template<typename Iterator>
  std::vector<OutputPolygon> GetNodeBorder(
      Iterator node_path_begin,
      Iterator node_path_end,
      const RectangularPrism<point3_bits>& bounding_box) const;

  BSPPolygonId AllocateId() {
    return next_id_++;
  }

  BSPPolygonId next_id() const {
    return next_id_;
  }

  BSPNodeRep root;

 private:
  BSPPolygonId next_id_ = 0;
};

template <typename ConvexPolygonTemplate>
template <typename Iterator>
std::vector<typename BSPTree<ConvexPolygonTemplate>::OutputPolygon>
BSPTree<ConvexPolygonTemplate>::GetNodeBorder(
    Iterator node_path_begin, Iterator node_path_end,
    const RectangularPrism<point3_bits>& bounding_box) const {
  struct MappedBSPNode : public BSPNodeRep {
   public:
    using Parent = BSPNodeRep;
    using typename Parent::HalfSpace3Rep;

    MappedBSPNode(const BSPNodeRep* original) : original(original) { }

    // Note that this overload hides the `Parent::Split` method that only takes
    // 1 argument.
    void Split(const HalfSpace3Rep& half_space,
               const BSPNodeRep* original_neg_child,
               const BSPNodeRep* original_pos_child) {
      Parent::MakeInterior(half_space, new MappedBSPNode(original_neg_child),
                           new MappedBSPNode(original_pos_child));
      Parent::PushContentsToChildren();
    }

    MappedBSPNode* negative_child() {
      return static_cast<MappedBSPNode*>(Parent::negative_child());
    }

    MappedBSPNode* positive_child() {
      return static_cast<MappedBSPNode*>(Parent::positive_child());
    }

    const BSPNodeRep* original = nullptr;
  };

  MappedBSPNode mapped_root(&root);
  for (auto& polygon : bounding_box.GetWalls()) {
    assert(polygon.vertex_count() > 0);
    mapped_root.contents_.emplace_back(/*id=*/0, /*on_node_plane=*/nullptr,
                                       std::move(polygon));
  }
  const BSPNodeRep* original_node = &root;
  // Add the split partitions from the node_path of from the original root into
  // mapped_root.
  for (Iterator pos = node_path_begin; pos != node_path_end; ++pos) {
    if (*pos) {
      mapped_root.contents_.emplace_back(
          /*id=*/0, /*on_node_plane=*/nullptr,
          bounding_box.IntersectPlane(-original_node->split()));
      assert(mapped_root.contents_.back().vertex_count() > 0);
      original_node = original_node->positive_child();
    } else {
      mapped_root.contents_.emplace_back(
          /*id=*/0, /*on_node_plane=*/nullptr,
          bounding_box.IntersectPlane(original_node->split()));
      assert(mapped_root.contents_.back().vertex_count() > 0);
      original_node = original_node->negative_child();
    }
  }

  // Split the mapped_root the same way as the original root along the node
  // path.
  MappedBSPNode* mapped_node = &mapped_root;
  original_node = &root;
  for (Iterator pos = node_path_begin; pos != node_path_end; ++pos) {
    mapped_node->Split(original_node->split(), original_node->negative_child(),
                       original_node->positive_child());
    if (*pos) {
      original_node = original_node->positive_child();
      mapped_node = mapped_node->positive_child();
    } else {
      original_node = original_node->negative_child();
      mapped_node = mapped_node->negative_child();
    }
  }

  // Convert `on_node_plane` in the polygons in mapped_root to pointers in
  // root+. Also convert split_by in all of the edges from pointers in
  // mapped_root to pointers in root_.
  for (OutputPolygon& output : mapped_node->contents_) {
    if (output.on_node_plane) {
      output.on_node_plane =
        static_cast<const MappedBSPNode*>(output.on_node_plane)->original;
    }
    for (size_t i = 0; i < output.vertex_count(); ++i) {
      auto& edge_info = output.vertex_data(i);
      if (edge_info.split_by) {
        edge_info.split_by =
          static_cast<const MappedBSPNode*>(edge_info.split_by)->original;
      }
    }
  }
  // Return the converted polygons.
  std::vector<OutputPolygon> result(std::move(mapped_node->border_contents_));
  result.insert(result.end(), mapped_node->contents_.begin(),
                mapped_node->contents_.end());
  return result;
}

}  // walnut

#endif // WALNUT_BSP_TREE_H__
