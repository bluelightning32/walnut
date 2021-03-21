#ifndef WALNUT_EDGE_LINE_CONNECTOR_H__
#define WALNUT_EDGE_LINE_CONNECTOR_H__

// For std::function
#include <functional>
#include <map>
// For ostringstream
#include <sstream>
// For is_base_of
#include <type_traits>

#include "walnut/connected_polygon.h"
#include "walnut/rational.h"
#include "walnut/vector2.h"

namespace walnut {

// Given a sorted range of std::reference_wrapper<ConnectedEdge>s all on the
// same line, this in place algorithm connects the half-edges together.
//
// Typically one instantiates the class, then calls the () operator to connect
// the edges. This algorithm exists as a class so that the std::vector
// intermediate data structures can be stored as member variables and reused
// between calls to the algorithm.
template <typename EdgeTemplate = ConnectedPolygon<>::EdgeRep>
class EdgeLineConnector {
 public:
  using EdgeRep = EdgeTemplate;
  using HomoPoint3Rep = typename EdgeRep::HomoPoint3Rep;
  using PolygonRep = typename EdgeRep::FinalPolygon;
  static_assert(
      std::is_base_of<ConnectedEdge<HomoPoint3Rep,
                                    PolygonRep,
                                    typename EdgeRep::ConnectedEdgeRep::Parent
                                   >,
                                   EdgeTemplate
                     >::value,
                     "EdgeRep must inherit from ConnectedEdge.");
  using NumInt = typename HomoPoint3Rep::NumInt;
  using DenomInt = typename HomoPoint3Rep::DenomInt;
  using NormalRep = typename PolygonRep::NormalRep;
  using ProjectedNormalRep = Vector2<NormalRep::component_bits>;

  // Connects the adjacent edges in the range of ConnectedEdges.
  //
  // From `edges_begin` up to `edges_end` defines a range of
  // std::reference_wrapper<ConnectedEdge>s.
  //
  // All of the half-edges in the range must be coincident with the same line.
  // The `sorted_dimension` component of that line must be positive. Half-edges
  // that point in the same direction as that line are considered positive
  // edges, and the other edges are considered negative edges.
  //
  // The input range of iterators must be sorted by the sorted_dimension
  // component of their endpoints with the lowest value. For positive
  // half-edges, the endpoint in that half-edge is the closest endpoint. For
  // negative half-edges, the endpoint of the next edge in the cycle is the
  // closest endpoint.
  //
  // Two half-edges are only connected if their line segments overlap. When
  // there are more than two half-edges overlapping the same line segment, then
  // each half-edge is connected to the half-edge that is closest rotationally
  // around the common line. Positive half-edges are connected to the next
  // half-edge by going counter-clockwise around the common line. Negative
  // edges are connected to the next half-edge by going clockwise around the
  // common line.
  //
  // `error` is called if the half-edges come from an open polyhedron. Even if
  // `error` is called, as many half-edges as possible will be connected. The
  // ones that cannot be connected will have nullptr partners.
  template <typename Iterator>
  void operator()(Iterator edges_begin, const Iterator& edges_end,
                  int sorted_dimension,
                  const std::function<void(const std::string&)>& error) {
    assert(end_events_.empty());
    assert(need_partners_.empty());

    ActiveEdgeMap active_edges((RotationCompare(sorted_dimension)));
    EndEventsCompare end_events_compare(sorted_dimension);

    while (edges_begin != edges_end) {
      const HomoPoint3Rep* current_location;
      if (end_events_.empty() ||
          IsLocationLessThan(
            GetBeginLocation(*edges_begin, sorted_dimension),
            GetEndLocation(*end_events_.front()->first, sorted_dimension),
            sorted_dimension)) {
        current_location = &GetBeginLocation(*edges_begin, sorted_dimension);
      } else {
        current_location =
          &GetEndLocation(*end_events_.front()->first, sorted_dimension);
        ProcessEndEvents(sorted_dimension, active_edges, *current_location,
                         end_events_compare);
      }

      while (edges_begin != edges_end &&
             GetBeginLocation(*edges_begin,
                              sorted_dimension) == *current_location) {
        assert(&edges_begin->get() != nullptr);
        auto add_info = active_edges.emplace(&edges_begin->get(), nullptr);
        assert(add_info.second);
        ++edges_begin;
        add_info.first->first->ResetPartners();
        end_events_.push_back(add_info.first);
        std::push_heap(end_events_.begin(), end_events_.end(),
                       end_events_compare);
        need_partners_.push_back(add_info.first);
      }

      ProcessNeedPartners(sorted_dimension, active_edges, *current_location,
                          error);
    }

    while (!end_events_.empty()) {
      const HomoPoint3Rep& current_location =
        GetEndLocation(*end_events_.front()->first, sorted_dimension);
      ProcessEndEvents(sorted_dimension, active_edges, current_location,
                       end_events_compare);
      ProcessNeedPartners(sorted_dimension, active_edges, current_location,
                          error);
    }

    assert(active_edges.empty());
    assert(need_partners_.empty());
  }

 private:
  // Compares EdgeRep pointers based on a 2D projection of the connected
  // polygon's normal.
  //
  // The vectors are compared using Vector2::RotationCompare. That comparison
  // has the transitivity property, which is necessary for using the Compare in
  // an std::set. RotationCompare gets transitivity by comparing the vectors
  // based on their counter clockwise angles from the x axis.
  struct RotationCompare {
    RotationCompare(int sorted_dimension) :
      sorted_dimension(sorted_dimension) { }

    ProjectedNormalRep GetProjectedNormal(const EdgeRep& edge) const {
      const NormalRep& polygon_normal = edge.polygon().normal();
      ProjectedNormalRep result =
        polygon_normal.DropDimension(sorted_dimension);
      if (!IsPositiveEdge(edge, sorted_dimension)) {
        // This is a negative edge.
        result.Negate();
      }
      return result;
    }

    bool operator()(const EdgeRep* e1, const EdgeRep* e2) const {
      ProjectedNormalRep e1_normal = GetProjectedNormal(*e1);
      ProjectedNormalRep e2_normal = GetProjectedNormal(*e2);
      if (!e1_normal.IsSameDir(e2_normal)) {
        return e1_normal.IsRotationLessThan(e2_normal);
      }
      bool e1_pos = IsPositiveEdge(*e1, sorted_dimension);
      bool e2_pos = IsPositiveEdge(*e2, sorted_dimension);
      if (e1_pos != e2_pos) {
        // Positive edges come before negative edges.
        return e2_pos < e1_pos;
      }
      return e1 < e2;
    }

    int sorted_dimension;
  };

  // Points from an EdgeRep to a its current pointer, or nullptr if the edge
  // does not have a pointer yet.
  //
  // If the edge points back to itself, that special value indicates that the
  // edge is marked for removal.
  using ActiveEdgeMap = std::map<EdgeRep*, EdgeRep*, RotationCompare>;
  using ActiveEdge = typename ActiveEdgeMap::iterator;

  // Compares active edges based on their end point in the sorted_dimension.
  struct EndEventsCompare {
    EndEventsCompare(int sorted_dimension) :
      sorted_dimension(sorted_dimension) { }

    // Returns true if the endpoint of e1 is strictly greater than the endpoint
    // of e2.
    bool operator()(const ActiveEdge& e1, const ActiveEdge& e2) const {
      return IsLocationLessThan(GetEndLocation(*e2->first, sorted_dimension),
                                GetEndLocation(*e1->first, sorted_dimension),
                                sorted_dimension);
    }

    int sorted_dimension;
  };

  using EndEventsIterator = typename std::vector<ActiveEdge>::iterator;

  static bool IsPositiveEdge(const EdgeRep& edge, int sorted_dimension) {
    return edge.line().d().components()[sorted_dimension].GetSign() >= 0;
  }

  static const HomoPoint3Rep& GetBeginLocation(const EdgeRep& edge,
                                               int sorted_dimension) {
    if (IsPositiveEdge(edge, sorted_dimension)) {
      // This is a positive edge. It starts at the start point of the edge.
      return edge.vertex();
    } else {
      // This is a negative edge. It starts at the start of the next edge.
      return GetNextEdge(edge).vertex();
    }
  }

  static const HomoPoint3Rep& GetEndLocation(const EdgeRep& edge,
                                             int sorted_dimension) {
    if (IsPositiveEdge(edge, sorted_dimension)) {
      // This is a positive edge. It ends at the start of the next edge.
      return GetNextEdge(edge).vertex();
    } else {
      // This is a negative edge. It ends at the start point of the edge.
      return edge.vertex();
    }
  }

  // Returns true if l1[dim]/l1.w() < l2[dim]/l2.w()
  static bool IsLocationLessThan(const HomoPoint3Rep& l1,
                                 const HomoPoint3Rep& l2,
                                 int sorted_dimension) {
    return rational::IsLessThan(
        l1.vector_from_origin().components()[sorted_dimension], l1.w(),
        l2.vector_from_origin().components()[sorted_dimension], l2.w());
  }

  static const EdgeRep& GetNextEdge(const EdgeRep& from) {
    return from.polygon().edge((from.edge_index() + 1) %
                               from.polygon().vertex_count());
  }

  // Process elements from `end_events_` that match `location`.
  //
  // The processed elements are removed from `end_events_` and `active_edges`.
  // If the edges had partners which were not deleted, they are added to
  // `need_partners_.`
  void ProcessEndEvents(int sorted_dimension, ActiveEdgeMap& active_edges,
                        const HomoPoint3Rep& location,
                        const EndEventsCompare& compare) {
    // Process events from end_events_ as long as their location matches
    // `location`.
    //
    // In this step, the events only moved after heap_end in end_events_, that
    // is the part of end_events_ which is not a heap.
    EndEventsIterator heap_end = end_events_.end();
    while (end_events_.begin() != heap_end &&
           GetEndLocation(*end_events_.front()->first,
                          sorted_dimension) == location) {
      ActiveEdge remove = end_events_.front();
      // If the active edge is still pointing to another edge, mark put that
      // partner in the list of edges that need a new partner.
      //
      // Typically that partner will itself be marked for removal shortly
      // afterwards. That case is handled by
      // RemoveFinishedEdgesFromNeedPartners below.
      if (remove->second != nullptr) {
        ActiveEdge found = active_edges.find(remove->second);
        assert(found != active_edges.end());
        assert(found->second == remove->first);
        found->second = nullptr;
        need_partners_.push_back(found);
      }
      // Set the active edge to point to itself as a way to mark it for
      // removal.
      remove->second = remove->first;
      std::pop_heap(end_events_.begin(), heap_end, compare);
      --heap_end;
    }

    // Typically the edges will be removed in pairs. In that case, the partner
    // will be in need_partners_, and it must be removed before the pointer is
    // invalidated by removing it from `active_edges`.
    RemoveFinishedEdgesFromNeedPartners();

    // Look at all the entries in end_events_ at or after heap_end, and remove
    // them from active_events and end_events_.
    while (end_events_.end() != heap_end) {
      if (!IsPositiveEdge(*end_events_.back()->first, sorted_dimension)) {
        // The partner list is built in reverse order for negative half-edges.
        // Now that this negative half-edge is no longer active, reverse the
        // partner list to put it in the proper order.
        end_events_.back()->first->ReversePartnerList();
      }
      assert(end_events_.back()->first == end_events_.back()->second);
      active_edges.erase(end_events_.back());
      end_events_.pop_back();
    }
  }

  // Repartner everything in need_partners_, and clear needs_partners_.
  void ProcessNeedPartners(
      int sorted_dimension, ActiveEdgeMap& active_edges,
      const HomoPoint3Rep& location,
      const std::function<void(const std::string&)>& error) {
    while (!need_partners_.empty()) {
      ActiveEdge needs_partner = need_partners_.back();
      need_partners_.pop_back();
      bool pos_edge = IsPositiveEdge(*needs_partner->first, sorted_dimension);
      // This starts out pointing to needs_partner. It is then moved to the
      // location of the new partner.
      ActiveEdge new_partner = needs_partner;
      if (pos_edge) {
        // Find the next edge in the clockwise direction.
        if (new_partner == active_edges.begin()) {
          new_partner = active_edges.end();
        }
        --new_partner;
      } else {
        // Find the next edge in the counter-clockwise direction.
        ++new_partner;
        if (new_partner == active_edges.end()) {
          new_partner = active_edges.begin();
        }
      }
      assert(new_partner != active_edges.end());
      assert(new_partner->first != nullptr);
      if (needs_partner->second == new_partner->first) {
        // needs_partner already points to its new target.
        continue;
      }
      if (needs_partner->second != nullptr) {
        assert(needs_partner->second != needs_partner->first);
        ActiveEdge old_target = active_edges.find(needs_partner->second);
        assert(old_target != active_edges.end());
        assert(old_target != new_partner);
        need_partners_.push_back(old_target);
        needs_partner->second = nullptr;
      }
      bool partner_pos_edge =
        IsPositiveEdge(*new_partner->first, sorted_dimension);
      if (pos_edge == partner_pos_edge) {
        std::ostringstream out;
        out << "The closest edge has the same polarity as its neighbor"
            << ", needs_partner=" << *needs_partner->first
            << ", pos_edge=" << pos_edge
            << ", closest=" << *new_partner->first
            << " pos_edge=" << partner_pos_edge << ".";
        error(out.str());
        AddPartner(sorted_dimension, *needs_partner->first, pos_edge, location,
                   nullptr);
      } else {
        Repartner(sorted_dimension, needs_partner, pos_edge, location,
                  new_partner);
      }
    }
  }

  // Set source's current partner to target, unless it already points to
  // target.
  void Repartner(int sorted_dimension, ActiveEdge source, bool source_pos,
                 const HomoPoint3Rep& location, ActiveEdge target) {
    if (target->second != source->first) {
      need_partners_.push_back(target);
    }
    source->second = target->first;
    AddPartner(sorted_dimension, *source->first, source_pos, location,
               target->first);
  }

  // Adds `target` as a new partner on `edge`.
  //
  // If `location` refers to the begin location (for postive edges) or the end
  // location (for negative edges), then this is the first partner for `edge`,
  // and edge.partner_ is set to `target`. Otherwise, `target` is added to
  // edge.extra_partners_.
  //
  // Note that the location check is necessary because with malformed input,
  // the first partner could be nullptr, followed by a non-null partner.
  void AddPartner(int sorted_dimension, EdgeRep& edge, bool pos_edge,
                  const HomoPoint3Rep& location, EdgeRep* target) {
    const HomoPoint3Rep& compare_endpoint =
      pos_edge ? edge.vertex() : GetNextEdge(edge).vertex();
    if (rational::Equals(
          location.vector_from_origin().components()[sorted_dimension],
          location.w(),
          compare_endpoint.vector_from_origin().components()[sorted_dimension],
          compare_endpoint.w())) {
      assert(edge.partner_ == nullptr);
      edge.partner_ = target;
    } else {
      edge.extra_partners_.emplace_back(location, target);
    }
  }

  // Remove all of the elements from `need_partners_` that have been marked for
  // removal.
  void RemoveFinishedEdgesFromNeedPartners() {
    auto new_end = std::remove_if(need_partners_.begin(), need_partners_.end(),
                                  [](ActiveEdge edge) {
                                    return edge->second == edge->first;
                                  });
    need_partners_.erase(new_end, need_partners_.end());
  }

  // Every edge in `active_edges` has an entry in here for when it should be
  // removed from `active_edges`.
  //
  // This vector is sorted as a min_heap through the edge end locations. It is
  // accessed through heap functions like push_heap.
  std::vector<ActiveEdge> end_events_;

  // A list of ActiveEdges that need to be partnered to another ActiveEdge.
  std::vector<ActiveEdge> need_partners_;
};

}  // walnut

#endif // WALNUT_EDGE_LINE_CONNECTOR_H__
