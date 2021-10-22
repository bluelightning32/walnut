#ifndef WALNUT_BSP_TRAVERSER_H__
#define WALNUT_BSP_TRAVERSER_H__

#include "walnut/bsp_node.h"
#include "walnut/plane_partitioner.h"

namespace walnut {

template <typename BSPNodeRep, typename OutputPolygon>
class BSPTraverser {
 public:
  void Run(BSPNodeRep& node, BSPVisitor<OutputPolygon>& visitor) {
    std::pair<bool, bool> match = visitor.IsInside(node.content_info_by_id());
    if (!match.second) {
      // Only polygons that change the sideness should be emitted. The sideness
      // is fixed for this set of polygons, which means that none of them will
      // be emitted.
      return;
    }

    if (!node.contents().empty()) {
      if (node.IsLeaf()) {
        //std::cout << "Splitting leaf. Had contents=" << node.contents().size() << " " << node.border_contents().size()
         // << " pwn=" << node.GetContentInfoForId(0).pwn << std::endl;
        node.Split();
        //std::cout << "Neg child has contents=" << node.negative_child()->contents().size() << " " << node.negative_child()->border_contents().size()
          //<< " pwn=" << node.negative_child()->GetContentInfoForId(0).pwn << std::endl;
        //std::cout << "Pos child has contents=" << node.positive_child()->contents().size() << " " << node.positive_child()->border_contents().size()
          //<< " pwn=" << node.positive_child()->GetContentInfoForId(0).pwn << std::endl;
        //std::cout << "Parent now has contents=" << node.contents().size() << " " << node.border_contents().size()
          //<< " pwn=" << node.GetContentInfoForId(0).pwn << std::endl;
      } else {
        node.PushContentsToChildren();
      }
    }

    if (!node.IsLeaf()) {
      visitor.EnterInteriorNode(/*from_partitioner=*/false, node.split());
      Run(*node.negative_child(), visitor);
      Run(*node.positive_child(), visitor);
      visitor.LeaveInteriorNode(/*from_partitioner=*/false, node.split());
      return;
    }

    assert(node.IsLeaf());
    assert(node.contents().empty());
    assert(border_map_.empty());
    for (const InputPolygon& border : node.border_contents()) {
      assert(border.on_node_plane.split != nullptr);
      border_map_.emplace(border.on_node_plane.split, &border);
    }
    BorderMapIterator range_start = border_map_.begin();
    for(BorderMapIterator pos = border_map_.begin(); pos != border_map_.end();
        ++pos) {
      if (pos->first != range_start->first) {
        ProcessPolygonsOnPlane(node.content_info_by_id(), range_start, pos,
                               visitor);
        range_start = pos;
      }
    }
    if (range_start != border_map_.end()) {
      ProcessPolygonsOnPlane(node.content_info_by_id(), range_start,
                             border_map_.end(), visitor);
    }
    border_map_.clear();
  }

 private:
  using InputPolygon = typename BSPNodeRep::PolygonRep;
  using BorderMap = std::unordered_multimap<const HalfSpace3*,
                                            const InputPolygon*>;
  using BorderMapIterator = typename BorderMap::iterator;

  void ProcessPolygonsOnPlane(
      const std::vector<BSPContentInfo>& content_info_by_id,
      BorderMapIterator begin, BorderMapIterator end,
      BSPVisitor<OutputPolygon>& visitor) {
    assert(begin != end);
    const InputPolygon& first_polygon = *begin->second;
    const int drop_dimension = first_polygon.drop_dimension();
    const Vector3& first_normal = first_polygon.plane().normal();
    const bool pos_normal =
      first_normal.components()[drop_dimension].GetSign() < 0;
    using BorderMapValue = typename BorderMap::value_type;
    auto transform = [](const BorderMapValue& entry) -> const InputPolygon& {
      return *entry.second;
    };
    using Transformer =
      TransformIterator<BorderMapIterator, decltype(transform)>;
    partitioner_.Run(Transformer(begin, transform),
                     Transformer(end, transform), drop_dimension, pos_normal,
                     content_info_by_id, visitor);
  }

  BorderMap border_map_;
  PlanePartitioner<OutputPolygon> partitioner_;
};

}  // walnut

#endif // WALNUT_BSP_TRAVERSER_H__
