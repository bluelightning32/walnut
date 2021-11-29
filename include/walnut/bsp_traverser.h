#ifndef WALNUT_BSP_TRAVERSER_H__
#define WALNUT_BSP_TRAVERSER_H__

// for std::forward_as_tuple
#include <tuple>
// for std::pair
#include <utility>

#include "walnut/bsp_node.h"
#include "walnut/plane_partitioner.h"

namespace walnut {

template <typename BSPNodeRep, typename OutputPolygon>
class BSPTraverser {
 public:
  class SplitStrategy {
   public:
    SplitStrategy(BSPTraverser& parent, size_t kd_strategy_poly_lower_bound)
    : parent_(parent),
      kd_strategy_poly_lower_bound_(kd_strategy_poly_lower_bound) { }

    ~SplitStrategy() {
      if (kd_strategy_poly_lower_bound_ != static_cast<size_t>(-1)) {
        FreeEventLists();
      }
    }

    std::pair<SplitStrategy, SplitStrategy> Split(BSPNodeRep& node) {
      std::pair<SplitStrategy, SplitStrategy> result(
          std::piecewise_construct,
          std::forward_as_tuple(parent_, kd_strategy_poly_lower_bound_),
          std::forward_as_tuple(parent_, kd_strategy_poly_lower_bound_));

      if (node.contents().size() <= kd_strategy_poly_lower_bound_) {
        node.Split();
        result.first.kd_strategy_poly_lower_bound_ = static_cast<size_t>(-1);
        result.second.kd_strategy_poly_lower_bound_ = static_cast<size_t>(-1);
        return result;
      }

      if (event_points_[0].empty()) {
        InitializeEventPoints(node);
      }

      std::pair<BSPContentId, size_t> exclude = node.GetCostExclude();
      assert(exclude.second <= node.contents().size());
      PolygonEventPointPartition best =
        GetLowestCost(exclude.first, exclude.second, node.contents(),
                      event_points_[0].data());
      int best_dimension = 0;
      for (int i = 1; i < 3; ++i) {
        PolygonEventPointPartition alt =
          GetLowestCost(exclude.first, exclude.second, node.contents(),
                        event_points_[i].data());
        if (alt.cost < best.cost) {
          best = alt;
          best_dimension = i;
        }
      }
      MLogNEstimator estimator;
      // Reject the axis aligned split if the cost is worse than the parent
      // cost, taking into account the excluded count. Also reject the axis
      // aligned split if it is more expensive than splitting at the 15/16
      // point.
      //
      // Unless the axis aligned split is coincident with content polygons,
      // because then it needs to be split in that plane eventually anyway.
      size_t cutoff_minority_side = node.contents().size() / 16;
      size_t cutoff_majority_side = node.contents().size() - cutoff_minority_side;
      size_t cost_cutoff = std::min(
          estimator.Estimate(node.contents().size(),
                             1 + node.contents().size() - exclude.second),
          estimator.Estimate(cutoff_majority_side, 1 + cutoff_majority_side) +
            estimator.Estimate(cutoff_minority_side,
                               1 + cutoff_minority_side));
      best.DiscountBorderPolygons(event_points_[best_dimension].data(),
                                  node.contents().size());
      if (best.cost >= cost_cutoff && best.border_poly_count == 0) {
        node.Split();
        result.first.FreeEventLists();
        result.second.FreeEventLists();
        result.first.kd_strategy_poly_lower_bound_ = static_cast<size_t>(-1);
        result.second.kd_strategy_poly_lower_bound_ = static_cast<size_t>(-1);
        return result;
      }
      result.first.event_points_[best_dimension] =
        parent_.AllocateEventPoints();
      node.Split(best, best_dimension, event_points_[best_dimension],
                 parent_.polygon_index_map_,
                 result.first.event_points_[best_dimension]);
      result.second.event_points_[best_dimension] =
        std::move(event_points_[best_dimension]);
      for (int i = 1; i < 3; ++i) {
        int dimension = (best_dimension + i) % 3;
        result.first.event_points_[dimension] = parent_.AllocateEventPoints();
        result.first.event_points_[dimension].resize(
            best.GetNegEventPointCount());
        result.second.event_points_[dimension] = parent_.AllocateEventPoints();
        result.second.event_points_[dimension].resize(
            best.GetPosEventPointCount());
        best.ApplySecondary(dimension, parent_.polygon_index_map_.size(),
                            node.negative_child()->contents(),
                            node.positive_child()->contents(),
                            parent_.polygon_index_map_.data(),
                            event_points_[dimension].data(),
                            result.first.event_points_[dimension].data(),
                            result.second.event_points_[dimension].data(),
                            parent_.merge_heap_);
        parent_.FreeEventPoints(std::move(event_points_[dimension]));
      }
      return result;
    }

   private:
    void FreeEventLists() {
      for (std::vector<PolygonEventPoint>& event_point : event_points_) {
        if (event_point.capacity()) {
          parent_.FreeEventPoints(std::move(event_point));
        }
      }
    }

    void InitializeEventPoints(const BSPNodeRep& node) {
      for (int i = 0; i < 3; ++i) {
        event_points_[i] = parent_.AllocateEventPoints();
        event_points_[i].resize(node.contents().size()*2);
        MakeEventPoints(/*dimension=*/i, node.contents(),
                        event_points_[i].data());
      }
    }

    BSPTraverser& parent_;
    // If a node contains this main polygons or fewer, do not attempt a k-d
    // tree split and instead use a different BSP strategy to split the tree.
    size_t kd_strategy_poly_lower_bound_ = 6;
    std::vector<PolygonEventPoint> event_points_[3];
  };

  // Traverses `node`.
  //
  // If during the traversal a node contains `kd_strategy_poly_lower_bound`
  // polygons or fewer, the k-d tree split strategy is disabled for that node
  // and a backup BSP tree strategy is used. Use -1 to disable the k-d split
  // strategy.
  void Run(BSPNodeRep& node, BSPVisitor<OutputPolygon>& visitor,
           size_t kd_strategy_poly_lower_bound = 6) {
    Run(node, visitor, SplitStrategy(*this, kd_strategy_poly_lower_bound));
  }

  void Run(BSPNodeRep& node, BSPVisitor<OutputPolygon>& visitor,
           SplitStrategy&& strategy) {
    std::pair<bool, bool> match = visitor.IsInside(node.content_info_by_id());
    if (!match.second) {
      // Only polygons that change the sideness should be emitted. The sideness
      // is fixed for this set of polygons, which means that none of them will
      // be emitted.
      return;
    }

    if (node.IsLeaf()) {
      RunLeaf(node, visitor, std::move(strategy));
      return;
    }

    if (!node.contents().empty()) {
      node.PushContentsToChildren();
    }

    visitor.EnterInteriorNode(/*from_partitioner=*/false, node.split());
    Run(*node.negative_child(), visitor);
    Run(*node.positive_child(), visitor);
    visitor.LeaveInteriorNode(/*from_partitioner=*/false, node.split());
  }

 private:
  using InputPolygon = typename BSPNodeRep::PolygonRep;
  using BorderMap = std::unordered_multimap<const HalfSpace3*,
                                            const InputPolygon*>;
  using BorderMapIterator = typename BorderMap::iterator;

  void RunLeaf(BSPNodeRep& node, BSPVisitor<OutputPolygon>& visitor,
               SplitStrategy&& split_strategy) {
    assert(node.IsLeaf());
    std::pair<bool, bool> match = visitor.IsInside(node.content_info_by_id());
    if (!match.second) {
      // Only polygons that change the sideness should be emitted. The sideness
      // is fixed for this set of polygons, which means that none of them will
      // be emitted.
      return;
    }

    if (!node.contents().empty()) {
      std::pair<SplitStrategy, SplitStrategy> child_splits =
        split_strategy.Split(node);

      assert(!node.IsLeaf());
      visitor.EnterInteriorNode(/*from_partitioner=*/false, node.split());
      RunLeaf(*node.negative_child(), visitor, std::move(child_splits.first));
      RunLeaf(*node.positive_child(), visitor, std::move(child_splits.second));
      visitor.LeaveInteriorNode(/*from_partitioner=*/false, node.split());
      return;
    }

    VisitLeaf(node, visitor);
  }

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

  void VisitLeaf(BSPNodeRep& node, BSPVisitor<OutputPolygon>& visitor) {
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

  std::vector<PolygonEventPoint> AllocateEventPoints() {
    if (event_points_free_list_.empty()) {
      return std::vector<PolygonEventPoint>();
    } else {
      std::vector<PolygonEventPoint> result =
        std::move(event_points_free_list_.back());
      event_points_free_list_.pop_back();
      return result;
    }
  }

  void FreeEventPoints(std::vector<PolygonEventPoint>&& event_points) {
    event_points.clear();
    event_points_free_list_.push_back(std::move(event_points));
  }

  BorderMap border_map_;
  PlanePartitioner<OutputPolygon> partitioner_;

  std::vector<PolygonMergeEvent> merge_heap_;
  std::vector<size_t> polygon_index_map_;
  std::vector<std::vector<PolygonEventPoint>> event_points_free_list_;
};

}  // walnut

#endif // WALNUT_BSP_TRAVERSER_H__
