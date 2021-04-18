#ifndef WALNUT_BSP_TREE_H__
#define WALNUT_BSP_TREE_H__

#include <type_traits>

#include "walnut/aabb_convex_polygon.h"
#include "walnut/bsp_node.h"
#include "walnut/bsp_traverser.h"

namespace walnut {

// OutputPolygonParentTemplate must have a EdgeParent that inherits from
// BSPEdgeInfo.
template <typename OutputPolygonParentTemplate = AABBConvexPolygon<>>
class BSPTree {
 public:
  using OutputPolygonParent = OutputPolygonParentTemplate;
  using BSPNodeRep = BSPNode<OutputPolygonParent>;
  using OutputPolygon = typename BSPNodeRep::PolygonRep;
  using EdgeParent = typename BSPNodeRep::EdgeParent;

  // BSPNode that represents and references a node copied from another tree.
  class MappedBSPNode : public BSPNodeRep {
   public:
    using Parent = BSPNodeRep;

    MappedBSPNode() = default;

    MappedBSPNode(const BSPNodeRep* original) : original_(original) { }

    void Reset(const BSPNodeRep* original) {
      Parent::Reset();
      original_ = original;
    }

    // Note that this overload hides the `Parent::Split` method that only takes
    // 1 argument.
    void Split(const HalfSpace3& half_space,
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

    const BSPNodeRep* original() const {
     return original_;
    }

   private:
    const BSPNodeRep* original_ = nullptr;
  };

  // Add a new polygon to this node.
  //
  // For an interior node, the contents will be pushed to the children.
  template <typename InputConvexPolygon>
  void AddContent(BSPContentId id, InputConvexPolygon&& polygon) {
    root.AddRootContent(id, std::forward<InputConvexPolygon>(polygon));
    root.PushContentsToLeaves();
  }

  template <typename InputConvexPolygon>
  void AddContent(InputConvexPolygon&& polygon) {
    AddContent(AllocateId(), std::forward<InputConvexPolygon>(polygon));
  }

  template <typename InputConvexPolygon>
  void AddContents(BSPContentId id,
                   const std::vector<InputConvexPolygon>& polygons) {
    for (const InputConvexPolygon& polygon : polygons) {
      root.AddRootContent(id, polygon);
    }
    root.PushContentsToLeaves();
  }

  // Traverses the tree and sends the accepted border polygons to `visitor`.
  //
  // `visitor.IsInside` controls which branches of the tree are visited, and
  // which border polygons are accepted from the leaf nodes. As the tree is
  // traversed, if any leaf nodes with non-border polygons are encountered,
  // they are split before being passed to `visitor`.
  template <typename VisitorPolygon>
  void Traverse(BSPVisitor<VisitorPolygon>& visitor) {
    BSPTraverser<BSPNodeRep, VisitorPolygon> traverser;
    traverser.Run(root, visitor);
  }

  // Returns a MappedBSPNode containing the polyhedron boundary of a BSPNode.
  //
  // The path to the BSPNode from the root is specified as a sequence of bools,
  // indicating whether to go to the negative child (false) or the positive
  // child (true). The sequence of bools is specified through the iterators
  // `node_path_begin` and `node_path_end`.
  //
  // The result is a MappedBSPNode that is split the same way as the input
  // BSPNode. The combination of the border polygons and content polygons in
  // that mapped node define the polyhedron of the boundary of the input node.
  // Each returned polygon indicates which BSPNode division it came from
  // through the on_node_plane->original field.
  //
  // In case the BSPNode border is unbounded, `bounding_box` provides an upper
  // bound for how far the ConvexPolygons will extend.
  template<typename Iterator>
  MappedBSPNode* GetNodeBorder(
      Iterator node_path_begin, Iterator node_path_end,
      const AABB& bounding_box, MappedBSPNode& mapped_root) const;

  BSPContentId AllocateId() {
    return next_id_++;
  }

  BSPContentId next_id() const {
    return next_id_;
  }

  BSPNodeRep root;

 private:
  BSPContentId next_id_ = 0;
};

template <typename ConvexPolygonTemplate>
template <typename Iterator>
typename BSPTree<ConvexPolygonTemplate>::MappedBSPNode*
BSPTree<ConvexPolygonTemplate>::GetNodeBorder(
    Iterator node_path_begin, Iterator node_path_end,
    const AABB& bounding_box,
    MappedBSPNode& mapped_root) const {
  mapped_root.Reset(&root);
  for (auto& polygon : bounding_box.GetWalls()) {
    assert(polygon.vertex_count() > 0);
    mapped_root.AddRootContent(/*id=*/0, std::move(polygon));
  }
  const BSPNodeRep* original_node = &root;
  // Add the split partitions from the node_path of from the original root into
  // mapped_root.
  for (Iterator pos = node_path_begin; pos != node_path_end; ++pos) {
    if (*pos) {
      mapped_root.AddRootContent(
          /*id=*/0, bounding_box.IntersectPlane(-original_node->split()));
      assert(mapped_root.contents().back().vertex_count() > 0);
      original_node = original_node->positive_child();
    } else {
      mapped_root.AddRootContent(
          /*id=*/0, bounding_box.IntersectPlane(original_node->split()));
      assert(mapped_root.contents().back().vertex_count() > 0);
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
  return mapped_node;
}

}  // walnut

#endif // WALNUT_BSP_TREE_H__
