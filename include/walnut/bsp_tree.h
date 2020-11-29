#ifndef WALNUT_BSP_TREE_H__
#define WALNUT_BSP_TREE_H__

#include "walnut/bsp_node.h"

namespace walnut {

template <int point3_bits_template = 32>
class BSPTree {
 public:
  using InputConvexPolygon = ConvexPolygon<point3_bits_template>;
  using BSPNodeRep = BSPNode<point3_bits_template>;

  static constexpr int point3_bits = point3_bits_template;

  // Add new polygons to this node.
  //
  // The iterators should produce `InputConvexPolygon`s. For an interior node,
  // the contents will be pushed to the children. `leaf_callback` will be
  // called for every leaf node that some pieces of the new contents settle in.
  // There may also be spurious calls to `leaf_callback` for leaves were the
  // contents did not land in.
  template <typename Iterator, typename LeafCallback>
  void AddContents(Iterator first, Iterator last, LeafCallback leaf_callback) {
    root.contents_.insert(root.contents_.end(), first, last);
    root.PushContentsToLeaves(leaf_callback);
  }

  // Add a new polygon to this node.
  //
  // For an interior node, the contents will be pushed to the children.
  // `leaf_callback` will be called for every leaf node that some pieces of the
  // new contents settle in. There may also be spurious calls to
  // `leaf_callback` for leaves were the contents did not land in.
  template <typename LeafCallback>
  void AddContent(const InputConvexPolygon& polygon,
                  LeafCallback leaf_callback) {
    root.contents_.emplace_back(polygon);
    root.PushContentsToLeaves(leaf_callback);
  }

  BSPNodeRep root;
};

}  // walnut

#endif // WALNUT_BSP_TREE_H__
