#ifndef WALNUT_CONNECTING_VISITOR_H__
#define WALNUT_CONNECTING_VISITOR_H__

// For std::remove_if
#include <algorithm>
#include <stack>
#include <unordered_map>
// For std::pair
#include <utility>
#include <vector>

#include "walnut/bsp_polygon.h"
#include "walnut/bsp_visitor.h"
#include "walnut/connected_polygon.h"
#include "walnut/convex_polygon.h"
#include "walnut/edge_line_connector.h"

namespace walnut {

// This visitor accepts polygons produced by BSPTraverser. It then connects
// adjacent facets together using EdgeLineConnector. Redundant facets are
// merged together.
template <typename FilterTemplate, typename ConnectedParent = ConvexPolygon<>>
class ConnectingVisitor
  : public BSPVisitor<BSPPolygon<ConnectedPolygon<ConnectedParent>>> {
 public:
  using Filter = FilterTemplate;
  using PolygonRep = BSPPolygon<ConnectedPolygon<ConnectedParent>>;

  ConnectingVisitor(Filter filter,
                    std::function<void(const std::string&)> error_log)
    : filter_(std::move(filter)), error_log_(std::move(error_log)) { }

  std::pair<bool, bool> IsInside(
      const std::vector<BSPContentInfo>& content_info_by_id) override {
    return filter_(content_info_by_id);
  }

  // Returns the facets of the resulting polyhedron.
  //
  // Call `FilterEmptyPolygons` first to remove facets that were emptied as
  // part of a merge.
  const std::vector<PolygonRep>& polygons() const {
    return polygons_;
  }

  // Removes facets that were emptied as part of a merge.
  //
  // When two polygons are merged together, one of them ends up with all of the
  // edges and the other ends up empty but still in the output vector. This
  // function removes those empty polygons.
  void FilterEmptyPolygons() {
    polygons_.erase(std::remove_if(polygons_.begin(), polygons_.end(),
                                   [](PolygonRep& polygon) {
                                     return polygon.vertex_count() == 0;
                                   }),
                    polygons_.end());
  }

  // Moves out the facets of the resulting polyhedron.
  //
  // Call `FilterEmptyPolygons` first to remove facets that were emptied as
  // part of a merge.
  std::vector<PolygonRep> TakePolygons() {
    assert(interior_nodes_.empty());
    return std::move(polygons_);
  }

  void Accept(PolygonRep&& polygon) override {
    for (size_t i = 0; i < polygon.vertex_count(); ++i) {
      auto& edge = polygon.edge(i);
      auto it = interior_nodes_.find(edge.edge_first_coincident.split);
      assert(it != interior_nodes_.end());
      it->second.first_coincident_edges.emplace_back(&edge);
      if (it->second.temporary) {
        edge.edge_first_coincident.split = nullptr;
      }

      it = interior_nodes_.find(edge.vertex_last_coincident.split);
      assert(it != interior_nodes_.end());
      if (it->second.temporary) {
        edge.vertex_last_coincident.split = nullptr;
      }

      it = interior_nodes_.find(edge.edge_last_coincident.split);
      assert(it != interior_nodes_.end());
      if (it->second.temporary) {
        edge.edge_last_coincident.split = nullptr;
      }
    }
    polygons_.push_back(std::move(polygon));
  }

  void EnterInteriorNode(bool from_partitioner,
                         const HalfSpace3& split) override {
    if (edge_vector_freelist_.empty()) {
      edge_vector_freelist_.emplace();
    }
    assert(!edge_vector_freelist_.empty());
    bool inserted = interior_nodes_.emplace(&split,
        NodeInfo{
          /*temporary=*/from_partitioner,
          /*first_coincident_edges=*/std::move(edge_vector_freelist_.top())
        }).second;
    // Fix an unused variable warning in release builds.
    (void)(inserted);
    assert(inserted);
    edge_vector_freelist_.pop();
  }

  bool IsValidState() {
    for (const std::pair<const HalfSpace3* const, NodeInfo>& node_pair :
         interior_nodes_) {
      const NodeInfo& node_info = node_pair.second;
      for (const Deed<EdgeRep>& edge : node_info.first_coincident_edges) {
        if (edge.empty()) continue;

        if (edge->polygon().vertex_count() < 3) {
          return false;
        }

        if (edge->partner() != nullptr) {
          const PolygonRep& partner_polygon = edge->partner()->polygon();
          if (partner_polygon.vertex_count() < 3) {
            return false;
          }
          size_t partner_edge_index = edge->partner()->edge_index();
          const EdgeRep& partner = partner_polygon.edge(partner_edge_index);
          if (edge->line() != -partner.line()) {
            return false;
          }
          if (edge->vertex() !=
              partner_polygon.vertex((partner_edge_index + 1) %
                                     partner_polygon.vertex_count())) {
            return false;
          }
          if (partner.vertex() !=
              edge->polygon().vertex((edge->edge_index() + 1) %
                                     edge->polygon().vertex_count())) {
            return false;
          }
        }
      }
    }

    return true;
  }

  void LeaveInteriorNode(bool from_partitioner,
                         const HalfSpace3& split) override {
    assert(IsValidState());
    NodeMapIterator it = interior_nodes_.find(&split);
    assert(it != interior_nodes_.end());
    NodeInfo& node_info = it->second;
    // Some edges edges in node_info.first_coincident_edges could be nullptr.
    // So calculate `drop_dimension` from the split plane, instead of copying
    // it from one of the edge's polygons.
    int drop_dimension = it->first->normal().GetFirstNonzeroDimension();
    edge_connector_.ConnectUnsorted(node_info.first_coincident_edges.begin(),
                                    node_info.first_coincident_edges.end(),
                                    drop_dimension, error_log_);
    assert(IsValidState());
    for (EdgeVectorIterator it = node_info.first_coincident_edges.begin();
         it != node_info.first_coincident_edges.end(); ++it) {
      EdgeRep* edge = it->get();
      if (edge != nullptr && edge->partner() != nullptr) {
        int sorted_dimension =
          (edge->line().d().components()[(drop_dimension + 1) % 3].IsZero() ?
           drop_dimension + 2 : drop_dimension + 1) % 3;
        edge->polygon().TryMergePolygon(sorted_dimension, edge->edge_index(),
                                        edge->partner()->polygon(),
                                        edge->partner()->edge_index());
        assert(IsValidState());
      }
    }
    node_info.first_coincident_edges.clear();
    edge_vector_freelist_.push(std::move(node_info.first_coincident_edges));
    interior_nodes_.erase(it);
    assert(IsValidState());
  }

 private:
  using EdgeRep = typename PolygonRep::EdgeRep;
  using EdgeVector = std::vector<Deed<EdgeRep>>;
  using EdgeVectorIterator = typename EdgeVector::iterator;
  struct NodeInfo {
    bool temporary;
    EdgeVector first_coincident_edges;
  };
  using NodeMap = std::unordered_map<const HalfSpace3*, NodeInfo>;
  using NodeMapIterator = typename NodeMap::iterator;

  Filter filter_;
  std::vector<PolygonRep> polygons_;
  std::stack<EdgeVector> edge_vector_freelist_;
  NodeMap interior_nodes_;
  EdgeLineConnector<EdgeRep> edge_connector_;
  std::function<void(const std::string&)> error_log_;
};

}  // walnut

#endif // WALNUT_CONNECTING_VISITOR_H__
