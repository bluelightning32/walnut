#ifndef WALNUT_BSP_TREE_H__
#define WALNUT_BSP_TREE_H__

#include <cassert>
#include <iostream>
#include <set>
#include <type_traits>

#include "walnut/aabb_convex_polygon.h"
#include "walnut/bsp_node.h"
#include "walnut/bsp_traverser.h"
#include "walnut/connecting_visitor.h"

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

  template <typename InputConvexPolygon>
  void AddContents(BSPContentId id,
                   std::vector<InputConvexPolygon>&& polygons) {
    for (InputConvexPolygon& polygon : polygons) {
      root.AddRootContent(id, std::move(polygon));
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
  void Traverse(BSPVisitor<VisitorPolygon>& visitor,
                size_t kd_strategy_poly_lower_bound = 6) {
    BSPTraverser<BSPNodeRep, VisitorPolygon> traverser;
    traverser.Run(root, visitor, kd_strategy_poly_lower_bound);
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
  std::vector<ConnectingVisitorOutputPolygon<>>
  GetNodeBorder(Iterator node_path_begin, Iterator node_path_end,
                const AABB& bounding_box) const;

  // Returns the border of the BSP node at the given path.
  //
  // The polygons of the border are truncated at `bounding_box`, but the walls
  // of the bounding box are not included. So for example, if the BSP node at
  // the path has one border but is otherwise infinite, the result would be the
  // one border truncated to fit within `bounding_box`.
  template<typename Iterator>
  std::vector<MutableConvexPolygon<>>
  GetNodeBorderNoBoundWalls(Iterator node_path_begin, Iterator node_path_end,
                            const AABB& bounding_box) const;

  BSPContentId AllocateId() {
    return next_id_++;
  }

  BSPContentId next_id() const {
    return next_id_;
  }

  void Reset() {
    next_id_ = 0;
    root.Reset();
  }

  BSPNodeRep root;

 private:
  BSPContentId next_id_ = 0;
};

template <typename ConvexPolygonTemplate>
template <typename Iterator>
std::vector<ConnectingVisitorOutputPolygon<>>
BSPTree<ConvexPolygonTemplate>::GetNodeBorder(Iterator node_path_begin,
                                              Iterator node_path_end,
                                              const AABB& bounding_box) const {
  BSPNodeRep mapped_root;
  std::set<HalfSpace3, HalfSpace3Compare> bounding_box_planes;
  BSPContentId bounding_box_id = 0;
  for (auto& polygon : bounding_box.GetWalls()) {
    assert(polygon.vertex_count() > 0);
    bounding_box_planes.insert(polygon.plane());
    mapped_root.AddRootContent(bounding_box_id, std::move(polygon));
  }
  BSPContentId cell_border_id = 1;
  const BSPNodeRep* original_node = &root;
  // Add the split partitions from the node_path of from the original root into
  // mapped_root.
  for (Iterator pos = node_path_begin; pos != node_path_end; ++pos) {
    if (*pos) {
      if (bounding_box_planes.find(-original_node->split()) ==
          bounding_box_planes.end()) {
        mapped_root.AddRootContent(
            cell_border_id,
            bounding_box.IntersectPlane(-original_node->split()));
        assert(mapped_root.contents().back().vertex_count() > 0);
      }
      original_node = original_node->positive_child();
    } else {
      if (bounding_box_planes.find(original_node->split()) ==
          bounding_box_planes.end()) {
        mapped_root.AddRootContent(
            cell_border_id,
            bounding_box.IntersectPlane(original_node->split()));
        assert(mapped_root.contents().back().vertex_count() > 0);
      }
      original_node = original_node->negative_child();
    }
  }

  auto error_log = [](const std::string& error) {
    std::cerr << error << std::endl;
    assert(false);
  };
  auto filter = MakeXORFilter(OddPolygonFilter(bounding_box_id),
                              OddPolygonFilter(cell_border_id));
  ConnectingVisitor<decltype(filter)> visitor(filter, error_log);

  // Split the mapped_root the same way as the original root along the node
  // path.
  original_node = &root;
  std::vector<BSPNodeRep*> mapped_nodes{&mapped_root};
  for (Iterator pos = node_path_begin; pos != node_path_end; ++pos) {
    mapped_nodes.back()->Split(original_node->split());
    BSPNodeRep* mapped_child;
    if (*pos) {
      original_node = original_node->positive_child();
      mapped_child = mapped_nodes.back()->positive_child();
    } else {
      original_node = original_node->negative_child();
      mapped_child = mapped_nodes.back()->negative_child();
    }
    mapped_nodes.push_back(mapped_child);
  }

  BSPContentInfo node_bounding_box_info =
    mapped_nodes.back()->GetContentInfoForId(bounding_box_id);
  if (node_bounding_box_info.pwn < 1 &&
      !node_bounding_box_info.has_polygons()) {
    // The cell is outside of or adjacent to the bounding box. Return an empty
    // vector to avoid trying to run the traverser when the bounding box is
    // adjacent to the cell.
    return visitor.TakePolygons();
  }

  for (auto it = mapped_nodes.begin(); it + 1 != mapped_nodes.end(); ++it) {
    visitor.EnterInteriorNode(/*from_partitioner=*/false,
                              (*it)->split());
  }

  // If `bounding_box` is smaller than the last node, then the last node will
  // not be fully split. However, it must be fully split for the
  // ConnectingVisitor to work.
  BSPTraverser<BSPNodeRep, ConnectingVisitorOutputPolygon<>> traverser;
  traverser.Run(*mapped_nodes.back(), visitor);

  mapped_nodes.pop_back();
  for (auto it = mapped_nodes.rbegin(); it != mapped_nodes.rend(); ++it) {
    visitor.LeaveInteriorNode(/*from_partitioner=*/false,
                              (*it)->split());
  }
  return visitor.TakePolygons();
}

template <typename ConvexPolygonTemplate>
template <typename Iterator>
std::vector<MutableConvexPolygon<>>
BSPTree<ConvexPolygonTemplate>::GetNodeBorderNoBoundWalls(
    Iterator node_path_begin, Iterator node_path_end,
    const AABB& bounding_box) const {
  BSPNodeRep mapped_root;
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
  original_node = &root;
  BSPNodeRep* mapped_pos = &mapped_root;
  for (Iterator pos = node_path_begin; pos != node_path_end; ++pos) {
    mapped_pos->Split(original_node->split());
    BSPNodeRep* mapped_child;
    if (*pos) {
      original_node = original_node->positive_child();
      mapped_child = mapped_pos->positive_child();
    } else {
      original_node = original_node->negative_child();
      mapped_child = mapped_pos->negative_child();
    }
    mapped_pos = mapped_child;
  }

  std::vector<MutableConvexPolygon<>> result;
  result.insert(result.end(), mapped_pos->border_contents().begin(),
                mapped_pos->border_contents().end());
  result.insert(result.end(), mapped_pos->contents().begin(),
                mapped_pos->contents().end());

  return result;
}

}  // walnut

#endif // WALNUT_BSP_TREE_H__
