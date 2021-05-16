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

  // Returns a BSPNodeRep containing the polyhedron boundary of a BSPNode.
  //
  // The path to the BSPNode from the root is specified as a sequence of bools,
  // indicating whether to go to the negative child (false) or the positive
  // child (true). The sequence of bools is specified through the iterators
  // `node_path_begin` and `node_path_end`.
  //
  // The result is a BSPNode that is split the same way as the input BSPNode.
  // The combination of the border polygons and content polygons in that mapped
  // node define the polyhedron of the boundary of the input node.
  //
  // In case the BSPNode border is unbounded, `bounding_box` provides an upper
  // bound for how far the ConvexPolygons will extend.
  template<typename Iterator>
  BSPNodeRep* GetNodeBorder(
      Iterator node_path_begin, Iterator node_path_end,
      const AABB& bounding_box, BSPNodeRep& mapped_root) const;

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
typename BSPTree<ConvexPolygonTemplate>::BSPNodeRep*
BSPTree<ConvexPolygonTemplate>::GetNodeBorder(
    Iterator node_path_begin, Iterator node_path_end,
    const AABB& bounding_box,
    BSPNodeRep& mapped_root) const {
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
  BSPNodeRep* mapped_node = &mapped_root;
  original_node = &root;
  for (Iterator pos = node_path_begin; pos != node_path_end; ++pos) {
    mapped_node->Split(original_node->split());
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
