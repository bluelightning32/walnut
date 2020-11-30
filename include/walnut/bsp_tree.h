#ifndef WALNUT_BSP_TREE_H__
#define WALNUT_BSP_TREE_H__

#include "walnut/bsp_node.h"

namespace walnut {

// InputPolygonTemplate must have a VertexData that inherits from BSPEdgeInfo.
template <typename InputPolygonTemplate = BSPDefaultPolygon<32>>
class BSPTree {
 public:
  using InputPolygon = InputPolygonTemplate;
  using BSPNodeRep = BSPNode<InputPolygon>;

  static constexpr int point3_bits = InputPolygon::point3_bits;

  // Add a new polygon to this node.
  //
  // For an interior node, the contents will be pushed to the children.
  // `leaf_callback` will be called for every leaf node that some pieces of the
  // new contents settle in. There may also be spurious calls to
  // `leaf_callback` for leaves were the contents did not land in.
  template <typename InputConvexPolygon, typename LeafCallback>
  void AddContent(InputConvexPolygon&& polygon,
                  LeafCallback leaf_callback) {
    root.contents_.emplace_back(std::forward<InputConvexPolygon>(polygon));
    root.PushContentsToLeaves(leaf_callback);
  }

  BSPNodeRep root;
};

}  // walnut

#endif // WALNUT_BSP_TREE_H__
