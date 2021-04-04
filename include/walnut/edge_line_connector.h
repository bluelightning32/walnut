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

// Given a sorted range of Deed<EdgeRep>s all on the same line, this in place
// algorithm connects the half-edges together.
//
// Typically one instantiates the class, calls `SortEdgesInPlane` to sort a
// range of edges, splits up the edges based on the lines, then calls `Connect`
// to connect each group of edges on the same line. This algorithm exists as a
// class so that the std::vector intermediate data structures can be stored as
// member variables and reused between calls to the algorithm.
template <typename EdgeTemplate = ConnectedPolygon<>::EdgeRep>
class EdgeLineConnector {
 public:
  using EdgeRep = EdgeTemplate;
  using PolygonRep = typename EdgeRep::FinalPolygon;
  static_assert(
      std::is_base_of<ConnectedEdge<PolygonRep,
                                    typename EdgeRep::ConnectedEdgeRep::Parent
                                   >,
                                   EdgeTemplate
                     >::value,
                     "EdgeRep must inherit from ConnectedEdge.");

  // Connects the adjacent edges in the range of ConnectedEdges.
  //
  // From `edges_begin` up to `edges_end` defines a range of
  // Deed<EdgeRep>s. All edges must lie in a common plane. `drop_dimension`
  // indicates which component of the normal of that plane is non-zero.
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
  void ConnectUnsorted(Iterator edges_begin, const Iterator& edges_end,
                       int drop_dimension,
                       const std::function<void(const std::string&)>& error) {
    SortEdgesInPlane(edges_begin, edges_end, drop_dimension);
    while (edges_begin != edges_end) {
      std::pair<Iterator, int> range_end =
        FindNextLineStart(edges_begin, edges_end, drop_dimension);
      ConnectSorted(edges_begin, range_end.first,
                    /*sorted_dimension=*/range_end.second, error);
      edges_begin = std::move(range_end.first);
    }
  }

  // Connects the adjacent edges in the range of ConnectedEdges.
  //
  // From `edges_begin` up to `edges_end` defines a range of
  // Deed<EdgeRep>s.
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
  void ConnectSorted(Iterator edges_begin, const Iterator& edges_end,
                     int sorted_dimension,
                     const std::function<void(const std::string&)>& error) {
    assert(end_events_.empty());
    assert(need_partners_.empty());

    ActiveEdgeMap active_edges;
    EndEventsCompare end_events_compare(sorted_dimension);

    const HomoPoint3* prev_location = nullptr;
    while (edges_begin != edges_end) {
      const HomoPoint3* current_location;
      if (end_events_.empty() ||
          IsLocationLessThan(
            edges_begin->get()->GetBeginLocation(sorted_dimension),
            end_events_.front()->second.edge->GetEndLocation(sorted_dimension),
            sorted_dimension)) {
        current_location =
          &edges_begin->get()->GetBeginLocation(sorted_dimension);
      } else {
        current_location =
          &end_events_.front()->second.edge->GetEndLocation(sorted_dimension);
        ProcessEndEvents(sorted_dimension, active_edges, *current_location,
                         end_events_compare);
      }
      assert(prev_location == nullptr ||
             IsLocationLessThan(*prev_location, *current_location,
                                sorted_dimension));
      prev_location = current_location;

      while (edges_begin != edges_end &&
             edges_begin->get()->GetBeginLocation(sorted_dimension) ==
               *current_location) {
        Vector2 projected_normal =
          edges_begin->get()->polygon().normal()
          .DropDimension(sorted_dimension);
        bool pos_edge = edges_begin->get()->IsPositive(sorted_dimension);
        if (!pos_edge) {
          // This is a negative edge.
          projected_normal.Negate();
        }
        auto add_info =
          active_edges.emplace(ActiveEdgeKey{pos_edge,
                                             std::move(projected_normal)},
                               ActiveEdgeValue{edges_begin->get(),
                                               active_edges.end()});
        if (add_info.second) {
          add_info.first->second.edge->ResetPartners();
          end_events_.push_back(add_info.first);
          std::push_heap(end_events_.begin(), end_events_.end(),
                         end_events_compare);
          need_partners_.push_back(add_info.first);
        } else {
          std::ostringstream out;
          out << "Two active edges have the same polygon normal and polarity"
              << ", normal=" << edges_begin->get()->polygon().normal()
              << ", new_edge=" << edges_begin->get()
              << ", existing_edge=" << add_info.first->second.edge.get();
          error(out.str());
        }
        ++edges_begin;
      }

      ProcessNeedPartners(sorted_dimension, active_edges, *current_location,
                          error);
    }

    while (!end_events_.empty()) {
      const HomoPoint3& current_location =
        end_events_.front()->second.edge->GetEndLocation(sorted_dimension);
      ProcessEndEvents(sorted_dimension, active_edges, current_location,
                       end_events_compare);
      ProcessNeedPartners(sorted_dimension, active_edges, current_location,
                          error);
    }

    assert(active_edges.empty());
    assert(need_partners_.empty());
  }

  // In place sorts a range of ConnectedEdges in such a fashion that they are
  // safe to group by line and then pass to `operator()`.
  //
  // From `edges_begin` up to `edges_end` defines a range of
  // Deed<EdgeRep>s. All of those edges must be coincident with the same plane.
  // The `drop_dimension` component of that plane's normal must be non-zero.
  template <typename Iterator>
  static void SortEdgesInPlane(const Iterator& edges_begin,
                               const Iterator& edges_end, int drop_dimension) {
    struct EdgeCompare {
      EdgeCompare(int drop_dimension) : drop_dimension(drop_dimension) { }

      bool operator()(const Deed<EdgeRep> &e1, const Deed<EdgeRep> &e2) const {
        const PluckerLine& e1_line = e1->line();
        const PluckerLine& e2_line = e2->line();
        const auto e1_2dline = e1_line.d().DropDimension(drop_dimension);
        const auto e2_2dline = e2_line.d().DropDimension(drop_dimension);
        if (!e1_2dline.IsSameOrOppositeDir(e2_2dline)) {
          return e1_2dline.IsHalfRotationLessThan(e2_2dline);
        }
        int sorted_dimension = (
            e1_line.d().components()[(drop_dimension + 1) % 3].IsZero() ?
            drop_dimension + 2 : drop_dimension + 1) % 3;
        auto scaled_e1_dist = e1_line.m().components()[drop_dimension] *
                              e2_line.d().components()[sorted_dimension];
        auto scaled_e2_dist = e2_line.m().components()[drop_dimension] *
                              e1_line.d().components()[sorted_dimension];
        if (scaled_e1_dist != scaled_e2_dist) {
          assert(!e1_line.d().components()[sorted_dimension].IsZero());
          assert(!e2_line.d().components()[sorted_dimension].IsZero());
          return scaled_e1_dist.LessThan(
              /*flip=*/(e2_line.d().components()[sorted_dimension] < 0) ^
                       (e1_line.d().components()[sorted_dimension] < 0),
              scaled_e2_dist);
        }
        return IsLocationLessThan(e1->GetBeginLocation(sorted_dimension),
                                  e2->GetBeginLocation(sorted_dimension),
                                  sorted_dimension);
      }

      int drop_dimension;
    };
    std::sort(edges_begin, edges_end, EdgeCompare(drop_dimension));
  }

  // Takes a sorted range of ConnectedEdges and returns the end iterator for
  // the subrange sharing the same line.
  //
  // The edges must be sorted in the same way as what `SortEdgesInPlane`
  // provides. The return value will point to an edge that has a different line
  // than the first edge in the input range, or the end iterator if there are
  // no more remaining subranges. Edges that are coincident with the same
  // infinite line are considered part of the same subrange, even if there is a
  // gap between the edges such that they do not overlap.
  //
  // In addition to the end of the first subrange, the sorted dimension is also
  // returned.
  //
  // From `edges_begin` up to `edges_end` defines a range of
  // Deed<EdgeRep>s. All of those edges must be coincident with the same plane.
  // The `drop_dimension` component of that plane's normal must be non-zero.
  template <typename Iterator>
  static std::pair<Iterator, int> FindNextLineStart(
      const Iterator& edges_begin, const Iterator& edges_end,
      int drop_dimension) {
    assert (edges_begin != edges_end);
    HalfSpace2 line = edges_begin->get()->line().Project2D(drop_dimension);
    int sorted_dimension =
      (edges_begin->get()->line().d().components()[
       (drop_dimension + 1) % 3].IsZero() ?
       drop_dimension + 2 : drop_dimension + 1) % 3;
    Iterator pos = edges_begin;
    ++pos;
    while (pos != edges_end) {
      if (!pos->get()->line().Project2D(drop_dimension).HasSameLine(line)) {
        return std::make_pair(pos, sorted_dimension);
      }
      ++pos;
    }
    return std::make_pair(pos, sorted_dimension);
  }

  // Returns true if l1[dim]/l1.w() < l2[dim]/l2.w()
  static bool IsLocationLessThan(const HomoPoint3& l1, const HomoPoint3& l2,
                                 int sorted_dimension) {
    return rational::IsLessThan(
        l1.vector_from_origin().components()[sorted_dimension], l1.w(),
        l2.vector_from_origin().components()[sorted_dimension], l2.w());
  }

 private:
  struct ActiveEdgeKey {
    // Compares EdgeRep pointers based on a 2D projection of the connected
    // polygon's normal.
    //
    // The vectors are compared using Vector2::RotationCompare. That comparison
    // has the transitivity property, which is necessary for using the Compare
    // in an std::set. RotationCompare gets transitivity by comparing the
    // vectors based on their counter clockwise angles from the x axis.
    bool operator<(const ActiveEdgeKey &other) const {
      if (!projected_polygon_normal.IsSameDir(
            other.projected_polygon_normal)) {
        return projected_polygon_normal.IsRotationLessThan(
            other.projected_polygon_normal);
      }
      if (pos_edge != other.pos_edge) {
        // Positive edges come before negative edges.
        return other.pos_edge < pos_edge;
      }
      return false;
    }

    bool pos_edge;
    Vector2 projected_polygon_normal;
  };

  struct ActiveEdgeValue {
    using ActiveEdge = typename std::map<ActiveEdgeKey,
                                         ActiveEdgeValue>::iterator;

    Deed<EdgeRep> edge;
    // Points to the partner for this active edge, or map::end if the edge
    // does not currently have a partner.
    //
    // If the edge points back to itself, that special value indicates that the
    // edge is marked for removal.
    ActiveEdge partner;
  };

  // The active edges are indexed by the projected normals of their polygons.
  // The values contain a deed to the edge along with an iterator pointing to
  // the edge's current partner, or end if the edge does not have a partner
  // yet.
  //
  // If the value's iterator points back to the same entry, that special value
  // indicates that the edge is marked for removal.
  using ActiveEdgeMap = std::map<ActiveEdgeKey, ActiveEdgeValue>;
  using ActiveEdge = typename ActiveEdgeMap::iterator;

  // Compares active edges based on their end point in the sorted_dimension.
  struct EndEventsCompare {
    EndEventsCompare(int sorted_dimension) :
      sorted_dimension(sorted_dimension) { }

    // Returns true if the endpoint of e1 is strictly greater than the endpoint
    // of e2.
    bool operator()(const ActiveEdge& e1, const ActiveEdge& e2) const {
      return IsLocationLessThan(
          e2->second.edge->GetEndLocation(sorted_dimension),
          e1->second.edge->GetEndLocation(sorted_dimension), sorted_dimension);
    }

    int sorted_dimension;
  };

  using EndEventsIterator = typename std::vector<ActiveEdge>::iterator;

  // Process elements from `end_events_` that match `location`.
  //
  // The processed elements are removed from `end_events_` and `active_edges`.
  // If the edges had partners which were not deleted, they are added to
  // `need_partners_.`
  void ProcessEndEvents(int sorted_dimension, ActiveEdgeMap& active_edges,
                        const HomoPoint3& location,
                        const EndEventsCompare& compare) {
    // Process events from end_events_ as long as their location matches
    // `location`.
    //
    // In this step, the events only moved after heap_end in end_events_, that
    // is the part of end_events_ which is not a heap.
    EndEventsIterator heap_end = end_events_.end();
    while (end_events_.begin() != heap_end &&
           end_events_.front()->second.edge->
             GetEndLocation(sorted_dimension) == location) {
      ActiveEdge remove = end_events_.front();
      // If the active edge is still pointing to another edge, mark put that
      // partner in the list of edges that need a new partner.
      //
      // Typically that partner will itself be marked for removal shortly
      // afterwards. That case is handled by
      // RemoveFinishedEdgesFromNeedPartners below.
      if (remove->second.partner != active_edges.end()) {
        ActiveEdge partner = remove->second.partner;
        assert(partner->second.partner == remove);
        need_partners_.push_back(partner);
        partner->second.partner = active_edges.end();
        need_partners_.push_back(partner);
      }
      // Set the active edge to point to itself as a way to mark it for
      // removal.
      remove->second.partner = remove;
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
      if (!end_events_.back()->second.edge->IsPositive(sorted_dimension)) {
        // The partner list is built in reverse order for negative half-edges.
        // Now that this negative half-edge is no longer active, reverse the
        // partner list to put it in the proper order.
        end_events_.back()->second.edge->ReversePartnerList();
      }
      assert(end_events_.back()->second.partner == end_events_.back());
      active_edges.erase(end_events_.back());
      end_events_.pop_back();
    }
  }

  // Repartner everything in need_partners_, and clear needs_partners_.
  void ProcessNeedPartners(
      int sorted_dimension, ActiveEdgeMap& active_edges,
      const HomoPoint3& location,
      const std::function<void(const std::string&)>& error) {
    while (!need_partners_.empty()) {
      ActiveEdge needs_partner = need_partners_.back();
      need_partners_.pop_back();
      bool pos_edge = needs_partner->second.edge->IsPositive(sorted_dimension);
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
      assert(new_partner->second.edge != nullptr);
      if (needs_partner->second.partner == new_partner) {
        // needs_partner already points to its new target.
        continue;
      }
      if (needs_partner->second.partner != active_edges.end()) {
        ActiveEdge old_target = needs_partner->second.partner;
        assert(old_target != needs_partner);
        assert(old_target != new_partner);
        need_partners_.push_back(old_target);
        needs_partner->second.partner = active_edges.end();
      }
      bool partner_pos_edge =
        new_partner->second.edge->IsPositive(sorted_dimension);
      if (pos_edge == partner_pos_edge) {
        std::ostringstream out;
        out << "The closest edge has the same polarity as its neighbor"
            << ", needs_partner=" << *needs_partner->second.edge
            << ", pos_edge=" << pos_edge
            << ", closest=" << *new_partner->second.edge
            << " pos_edge=" << partner_pos_edge << ".";
        error(out.str());
        AddPartner(sorted_dimension, *needs_partner->second.edge, pos_edge,
                   location, nullptr);
      } else {
        Repartner(sorted_dimension, needs_partner, pos_edge, location,
                  new_partner);
      }
    }
  }

  // Set source's current partner to target, unless it already points to
  // target.
  void Repartner(int sorted_dimension, ActiveEdge source, bool source_pos,
                 const HomoPoint3& location, ActiveEdge target) {
    if (target->second.partner != source) {
      need_partners_.push_back(target);
    }
    source->second.partner = target;
    AddPartner(sorted_dimension, *source->second.edge, source_pos, location,
               target->second.edge.get());
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
                  const HomoPoint3& location, EdgeRep* target) {
    const HomoPoint3& compare_endpoint =
      pos_edge ? edge.vertex() : edge.next_vertex();
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
                                    return edge->second.partner == edge;
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
