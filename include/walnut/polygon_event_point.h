#ifndef WALNUT_POLYGON_EVENT_POINT_H__
#define WALNUT_POLYGON_EVENT_POINT_H__

// For std::make_heap
#include <algorithm>

#include "walnut/aabb_convex_polygon.h"
#include "walnut/bsp_polygon.h"
#include "walnut/m_log_n_estimator.h"

namespace walnut {

struct PolygonEventPoint {
  // Returns the polygon of this event point.
  //
  // This function only works when the event point is in its typical form,
  // where `partner` is set for start events and `content` is set for end
  // events.
  template <typename ParentPolygon=ConvexPolygon<>>
  const BSPPolygon<AABBConvexPolygon<ParentPolygon>>& GetPolygon(
      const PolygonEventPoint* event_points,
      const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
        polygons) const {
    if (start) {
      return polygons[event_points[index.partner].index.content];
    } else {
      return polygons[index.content];
    }
  }

  // Returns the location of this event point.
  //
  // This function only works when the event point is in its typical form,
  // where `partner` is set for start events and `content` is set for end
  // events.
  template <typename ParentPolygon=ConvexPolygon<>>
  const HomoPoint3& GetLocation(
      size_t dimension,
      const PolygonEventPoint* event_points,
      const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
        polygons) const {
    if (start) {
      assert(!event_points[index.partner].start);
      assert(event_points[index.partner].index.content < polygons.size());
      return polygons[event_points[index.partner].index.content].min_vertex(
          dimension);
    } else {
      return polygons[index.content].max_vertex(dimension);
    }
  }

  // True if this is the start of a new group of end points at the same
  // location.
  bool new_location;
  // True if this represents the minimum vertex of a polygon, or false if it
  // represents the maximum vertex of a polygon.
  bool start;
  union {
    // The index of the end point of the opposite type for the same polygon.
    //
    // Typically this branch is valid when `start` is true.
    size_t partner;

    // The index of the polygon.
    //
    // Typically this branch is valid when `start` is false.
    size_t content;
  } index;
};

struct PolygonMergeEvent {
  // True if this is a start event.
  bool start;
  // Set to true if this event is for the positive child, or false for the
  // negative child.
  bool pos_child;
  // The index of the polygon in the respective child polygon list.
  size_t polygon_index;
  // If this is an end event, then `partner_index` points to the start event in
  // the corresponding child event array. If this is a start event,
  // `partner_index` may point to the end event inside the parent event array,
  // or it may be -1 if a new end event needs to be added to the heap after
  // processing the start event.
  size_t partner_index;

  template <typename ParentPolygon=ConvexPolygon<>>
  const HomoPoint3& GetLocation(
      size_t dimension,
      const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
        neg_polygons,
      const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
        pos_polygons) const {
    const BSPPolygon<AABBConvexPolygon<ParentPolygon>>& polygon =
      pos_child ? pos_polygons[polygon_index] : neg_polygons[polygon_index];
    if (start) {
      return polygon.min_vertex(dimension);
    } else {
      return polygon.max_vertex(dimension);
    }
  }
};

template <typename ParentPolygon=ConvexPolygon<>>
class PolygonMergeEventCompare {
 public:
  PolygonMergeEventCompare(
      size_t dimension,
      const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
        neg_polygons,
      const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
        pos_polygons)
  : dimension_(dimension), neg_polygons_(neg_polygons),
    pos_polygons_(pos_polygons) { }

  // Returns true if a > b
  bool operator()(const PolygonMergeEvent& a,
                  const PolygonMergeEvent& b) const {
    int compare = a.GetLocation(dimension_, neg_polygons_,
        pos_polygons_).CompareComponent(dimension_,
                                        b.GetLocation(dimension_,
                                                      neg_polygons_,
                                                      pos_polygons_));
    if (compare > 0) return true;
    if (compare < 0) return false;
    // Start events come after end events.
    return a.start > b.start;
  }

 private:
  size_t dimension_;
  const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
    neg_polygons_;
  const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
    pos_polygons_;
};

struct PolygonEventPointPartition {
  // The index of the event point that defines the split plane.
  //
  // This is also the number of events from the beginning of the event point
  // array should go to the negative child only.
  size_t split_index;

  size_t cost;
  size_t neg_cost;
  size_t pos_cost;

  // The number of polygons that go to the negative child (including polygons
  // that are split in two).
  size_t neg_poly_count;

  // The number of polygons that go to the positive child (including polygons
  // that are split in two).
  size_t pos_poly_count;

  // The number of border polygons that will be removed by the partition.
  size_t border_poly_count = 0;

  // The index of the first event that shares the same location as
  // `split_index`.
  size_t split_location_begin;
  // The index of the first event that has the location after `split_index`.
  size_t split_location_end;

  size_t GetExtraPolygonCount(size_t parent_poly_count) const {
    return neg_poly_count + pos_poly_count + border_poly_count -
           parent_poly_count;
  }

  size_t GetNegEventPointCount() const {
    return neg_poly_count * 2;
  }

  size_t GetPosEventPointCount() const {
    return pos_poly_count * 2;
  }

  // Removes the border polygons from `neg_poly_count` and `pos_poly_count`.
  //
  // Also sets `split_location_begin` and `split_location_end`.
  //
  // `cost` is not updated.
  void DiscountBorderPolygons(const PolygonEventPoint* event_points,
                              size_t parent_poly_count);

  template <typename ParentPolygon=ConvexPolygon<>>
  HalfSpace3 GetSplitPlane(
      int dimension,
      const PolygonEventPoint* event_points,
      const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
        polygons) const {
    assert(!event_points[split_index].start);
    const HomoPoint3 split_location =
      event_points[split_index].GetLocation(dimension, event_points, polygons);
    BigInt numerator =
      split_location.vector_from_origin().components()[dimension];
    BigInt denominator = split_location.w();
    if (denominator.IsNegative()) {
      numerator.Negate();
      denominator.Negate();
    }
    return HalfSpace3::GetAxisAligned(dimension, std::move(numerator),
                                      std::move(denominator));
  }

  // Splits the parent polygon list into two children.
  //
  // `dimension` must match the value passed to `GetLowestCost` to get this
  // `PolygonEventPointPartition`.
  //
  // `DiscountBorderPolygons` must be called after `GetLowestCost` and before
  // this function.
  //
  // On input `event_points` are the event points for that dimension. On
  // output, it becomes event points for the positive child. Only the first
  // `split.GetPosEventPointCount` entries are valid on output.
  //
  // `neg_event_points` must be allocated with `split.GetNegEventPointCount` entries.
  //
  // On output `polygon_index_map` maps indices from `polygons` into indices in
  // `neg_polygons` and/or `pos_polygons`. `polygon_index_map` must be
  // allocated with `polygons.size()` entries. `polygon_index_map` is indexed
  // by the original polygon index. If a polygon was split into to both
  // children, then its entry will be < GetExtraPolygonCount. Else if the
  // polygon was split into the negative child only, then the entry will be <
  // neg_polygons.size(). Else the entry is an index for the positive_child +
  // neg_polygons.size().
  //
  // On output, `polygons` will be cleared.
  template <typename ParentPolygon=ConvexPolygon<>>
  void ApplyPrimary(
      int dimension,
      PolygonEventPoint* event_points,
      std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>& polygons,
      size_t* polygon_index_map,
      PolygonEventPoint* neg_event_points,
      std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>& neg_polygons,
      std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>& pos_polygons,
      std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
        neg_border_polygons,
      std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
        pos_border_polygons) const;

  // Splits the parent polygon list into two children.
  //
  // `dimension` should be the dimension of the `event_points`, not the
  // dimension passed to `GetLowestCost` to get this
  // `PolygonEventPointPartition`.
  //
  // `neg_polygons`, `pos_polygons`, and `polygon_index_map` must come from
  // `ApplyPrimary`.
  //
  // On input `event_points` are the event points for that dimension. They are
  // used as temporary storage by the function and left in an unspecified
  // state.
  //
  // `neg_event_points` must be allocated with `split.GetNegEventPointCount`
  // entries. `pos_event_points` must have a corresponding number of entries.
  //
  // `merge_heap` is used for temporary storage. It should be empty on input
  // and it will be empty on output.
  template <typename ParentPolygon=ConvexPolygon<>>
  void ApplySecondary(
      int dimension, size_t parent_poly_count,
      const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
        neg_polygons,
      const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
        pos_polygons,
      const size_t* polygon_index_map,
      PolygonEventPoint* event_points,
      PolygonEventPoint* neg_event_points,
      PolygonEventPoint* pos_event_points,
      std::vector<PolygonMergeEvent>& merge_heap) const;

 private:
  // Compares the locations of a merge event and a main event.
  //
  // Returns <0 if the location of the merge_event is before the main event.
  // Returns 0 if the location of the merge_event is the same as the main event
  // (ignoring the start/end attribute).
  // Returns >0 if the location of the main event is before the merge event.
  template <typename ParentPolygon=ConvexPolygon<>>
  static int CompareHeapLocationToMain(
      int dimension, const HomoPoint3& heap_location,
      const PolygonEventPoint& main_event,
      size_t extra_poly_count,
      const PolygonEventPoint* event_points,
      const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
        neg_polygons,
      const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
        pos_polygons,
      const size_t* polygon_index_map,
      const PolygonEventPoint* neg_event_points,
      const PolygonEventPoint* pos_event_points);
};

// Fills in `event_points` with the sorted order of `polygons` in `dimension`.
//
// Every entry in `polygons` must have 1 or more vertices.
//
// On input, `event_points` must have polygons.size()*2 entries in it. On
// output, all entries of `event_points` will be initialized.
//
// The event_points are first ordered by their location. Between event_points
// that share the same location, end event_points are ordered before start
// points.
template <typename ParentPolygon=ConvexPolygon<>>
void MakeEventPoints(
    size_t dimension,
    const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>& polygons,
    PolygonEventPoint* event_points) {
  // Create a heap of the remaining event points. Initially this contains only
  // the end events. As the end events are removed from the heap, their partner
  // start events are added.
  //
  // Specifically `heap_last` points to one after the end of the heap.
  PolygonEventPoint* heap_last = event_points + polygons.size();
  // Fill in the end events in an unsorted order.
  for (size_t i = 0; i < polygons.size(); ++i) {
    event_points[i].start = false;
    event_points[i].index.content = i;
  }

  // A Compare that ignores `start` and assumes everything is an end event.
  auto end_point_compare =
    [polygons, dimension](const PolygonEventPoint& a,
                          const PolygonEventPoint& b) -> bool {
    assert(a.start == false);
    assert(b.start == false);
    return polygons[a.index.content].max_vertex(dimension).IsLessThanComponent(
        dimension,
        polygons[b.index.content].max_vertex(dimension));
  };
  std::make_heap(event_points, heap_last, end_point_compare);

  // A Compare that works for start and end events.
  auto event_point_compare =
    [event_points, polygons, dimension](const PolygonEventPoint& a,
                                        const PolygonEventPoint& b) -> bool {
    const HomoPoint3& a_location =
      a.GetLocation(dimension, event_points, polygons);
    const HomoPoint3& b_location =
      b.GetLocation(dimension, event_points, polygons);
    const int location_compare =
      a_location.CompareComponent(dimension, b_location);
    if (location_compare == 0) {
      return a.start < b.start;
    } else {
      return location_compare < 0;
    }
  };
  // Points to the first vertex in `event_points` that is in its final
  // position.
  PolygonEventPoint* finished_begin = event_points + 2*polygons.size();

  while (finished_begin != event_points) {
    --finished_begin;
    assert (event_points != heap_last);
    std::pop_heap(event_points, heap_last, event_point_compare);
    --heap_last;
    *finished_begin = *heap_last;
    if (!finished_begin->start) {
      // Add the start point that corresponds to the endpoint that was just
      // removed.
      heap_last->start = true;
      heap_last->index.partner = finished_begin - event_points;
      ++heap_last;
      std::push_heap(event_points, heap_last, event_point_compare);
    }
  }
  assert(heap_last == event_points);

  // Now set `new_location` to the correct value.
  if (polygons.size() > 0) {
    event_points[0].new_location = true;
    for (size_t i = 1; i < polygons.size()*2; ++i) {
      event_points[i].new_location =
        event_points[i - 1].GetLocation(dimension, event_points, polygons).
        IsLessThanComponent(dimension,
            event_points[i].GetLocation(dimension, event_points, polygons));
    }
  }
}

// Finds the split location with the lowest estimated cost.
// 
// Returns the event point index of a point coincident with the lowest cost
// split plane, along with the estimated cost. If there is at least one event
// point, then the split index will point to an end event point.
//
// `exclude_id` may be the id of a mesh to exclude from the log_2 part of the
// cost estimation, in which case `exclude_count` must be the number of
// polygons with that id in the mesh. `exclude_id` may be set to -1 to not
// exclude any meshes.
template <typename ParentPolygon=ConvexPolygon<>>
PolygonEventPointPartition GetLowestCost(
    size_t exclude_id, size_t exclude_count,
    const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>& polygons,
    const PolygonEventPoint* event_points) {
  size_t neg_total = 0;
  size_t pos_total = polygons.size();
  size_t neg_log_count = 1;
  size_t pos_log_count = polygons.size() - exclude_count + 1;
  PolygonEventPointPartition best;
  best.neg_cost = -1;
  best.pos_cost = -1;
  best.cost = -1;
  best.split_index = -1;
  MLogNEstimator neg_estimator, pos_estimator;
  bool last_event_was_end = false;
  size_t i;
  for (i = 0; i < polygons.size()*2; ++i) {
    const PolygonEventPoint& event_point = event_points[i];
    if (event_point.start) {
      if (last_event_was_end) {
        const size_t neg_cost = neg_estimator.Estimate(neg_total, neg_log_count);
        const size_t pos_cost = pos_estimator.Estimate(pos_total, pos_log_count);
        const size_t cost = neg_cost + pos_cost;
        if (cost < best.cost) {
          best.cost = cost;
          best.neg_cost = neg_cost;
          best.pos_cost = pos_cost;
          best.split_index = i - 1;
          best.neg_poly_count = neg_total;
          best.pos_poly_count = pos_total;
        }
        last_event_was_end = false;
      }
      ++neg_total;
      if (polygons[event_points[event_point.index.partner].index.content].id !=
          exclude_id) {
        ++neg_log_count;
      }
    } else {
      --pos_total;
      if (polygons[event_point.index.content].id != exclude_id) {
        --pos_log_count;
      }
      last_event_was_end = true;
    }
  }
  if (last_event_was_end) {
    const size_t neg_cost = neg_estimator.Estimate(neg_total, neg_log_count);
    const size_t pos_cost = pos_estimator.Estimate(pos_total, pos_log_count);
    const size_t cost = neg_cost + pos_cost;
    if (cost < best.cost) {
      best.cost = cost;
      best.neg_cost = neg_cost;
      best.pos_cost = pos_cost;
      best.split_index = i;
      best.neg_poly_count = neg_total;
      best.pos_poly_count = pos_total;
    }
  }
  return best;
}

inline void PolygonEventPointPartition::DiscountBorderPolygons(
    const PolygonEventPoint* event_points,
    size_t parent_poly_count) {
  // Go through all of the events before the split. `split_index` itself points
  // to the last end event in the neg child.
  split_location_begin = split_index;
  while (!event_points[split_location_begin].new_location) {
    assert(split_location_begin > 0);
    --split_location_begin;
    if (event_points[split_location_begin].start) {
      // This event points to a border polygon, not a neg polygon.
      --neg_poly_count;
      ++border_poly_count;
    }
  }
  split_location_end = split_index + 1;
  while (split_location_end < parent_poly_count*2 &&
         !event_points[split_location_end].new_location) {
    if (!event_points[split_location_end].start) {
      --pos_poly_count;
      ++border_poly_count;
    }
    ++split_location_end;
  }
}

template <typename ParentPolygon>
void PolygonEventPointPartition::ApplyPrimary(
    int dimension,
    PolygonEventPoint* event_points,
    std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>& polygons,
    size_t* polygon_index_map,
    PolygonEventPoint* neg_event_points,
    std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>& neg_polygons,
    std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>& pos_polygons,
    std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
      neg_border_polygons,
    std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
      pos_border_polygons) const {
  using PolygonRep = BSPPolygon<AABBConvexPolygon<ParentPolygon>>;
  assert(split_index < polygons.size()*2);
  assert(!event_points[split_index].start);

  neg_polygons.reserve(neg_poly_count);
  pos_polygons.reserve(pos_poly_count);
  const size_t extra_count = GetExtraPolygonCount(polygons.size());
  neg_polygons.resize(extra_count);
  pos_polygons.resize(extra_count);
  size_t used_extra = 0;
  const HalfSpace3 split_plane =
    GetSplitPlane(dimension, event_points, polygons);
  size_t neg_used = 0;
  size_t pos_used = 0;

  // Polygons go through the following states:
  // 1. Start point not processed yet
  //    polygon:                  in parent
  //    event_points.start:       points to parent end event
  //    event_points.end:         original polygon index
  //    child_event_points.start: not created yet
  //    child_event_points.end:   not created yet
  //    polygon_index_map:        not initialized
  //
  // 2a.Start point processed, but not end point, polygon in the neg child
  //    polygon:                  in neg_child
  //    event_points.start:       points to parent end event
  //    event_points.end:         index of start event in neg_event_points
  //    neg_event_points.start:   neg_child index
  //    neg_event_points.end:     not created yet
  //    polygon_index_map:        neg_child index
  //
  // 2b.Start point processed, but not end point, polygon in pos child
  //    polygon:                  in pos_child
  //    event_points.start:       possibly overwritten by pos_event_points
  //    event_points.end:         index of start event in pos_event_points +
  //                              neg_poly_count*2
  //    pos_event_points.start:   index of polygon in `pos_child`
  //    pos_event_points.end:     not created yet
  //    polygon_index_map:        pos_child index + neg_poly_count*2
  //
  // 2c.Start point processed, but not end point, polygon in both children
  //    polygon:                  in pos_child and neg_child at the same index
  //    event_points.start:       possibly overwritten by pos_event_points
  //    event_points.end:         index of start event in pos_event_points
  //    neg_event_points.start:   index of end event in neg_event_points
  //    neg_event_points.end:     child index of polygon
  //    pos_event_points.start:   index of polygon in `pos_child`
  //    pos_event_points.end:     not created yet
  //    polygon_index_map:        neg_child and pos_child index
  //
  // 3. Start and end point processed
  //    polygon:                  in appropriate child/children
  //    event_points.start:       possibly overwritten by neg_event_points
  //    event_points.end:         possibly overwritten by neg_event_points
  //    child_event_point.start:  index of child end event point
  //    child_event_point.end:    child index of polygon
  //    polygon_index_map:        child index of polygon

  // These will be initialized when the first event point is processed.
  bool neg_new_location;
  bool pos_new_location;
  size_t i = 0;
  for (; i < polygons.size()*2; ++i) {
    const PolygonEventPoint& event = event_points[i];
    if (event.new_location) {
      neg_new_location = true;
      pos_new_location = true;
    }
    if (event.start) {
      const size_t poly_index =
        event_points[event.index.partner].index.content;
      PolygonRep& polygon = polygons[poly_index];
      // True if the start of the polygon is on the positive side or on the
      // split plane.
      bool start_pos_side = i >= split_location_begin;
      // True if the end of the polygon is on the negative side or on the split
      // plane.
      bool end_neg_side = event.index.partner < split_location_end;
      if (start_pos_side) {
        if (end_neg_side) {
          // It's coincident with the split plane. Use the polygon normal to
          // determine which side the border polygon goes to.
          bool pos_child =
            polygon.plane().normal().components()[dimension].IsNegative();
          if (pos_child) {
            pos_border_polygons.push_back(std::move(polygon));
          } else {
            neg_border_polygons.push_back(std::move(polygon));
          }
          polygon_index_map[poly_index] = -1;
          event_points[event.index.partner].index.partner = -1;
        } else {
          event_points[event.index.partner].index.partner =
            pos_used + neg_polygons.size()*2;
          polygon_index_map[poly_index] = pos_polygons.size() + neg_poly_count;
          PolygonEventPoint& child_event = event_points[pos_used];
          ++pos_used;
          child_event.index.content = pos_polygons.size();
          pos_polygons.push_back(std::move(polygon));
          child_event.new_location = pos_new_location;
          pos_new_location = false;
          child_event.start = true;
        }
      } else {
        if (!end_neg_side) {
          // Goes to both children
          auto child_polygons =
            std::move(polygon).CreateSplitChildren(
                polygon.GetSplitInfo(split_plane));
          neg_polygons[used_extra] = std::move(child_polygons.first);
          pos_polygons[used_extra] = std::move(child_polygons.second);
          polygon_index_map[poly_index] = used_extra;

          // Add the negative end event.
          const size_t neg_end_event_index = neg_poly_count*2 - used_extra - 1;
          PolygonEventPoint& neg_end_event =
            neg_event_points[neg_end_event_index];
          neg_end_event.start = false;
          neg_end_event.new_location = false;
          neg_end_event.index.content = used_extra;

          // Add the negative start event.
          PolygonEventPoint& neg_start_event = neg_event_points[neg_used];
          ++neg_used;
          neg_start_event.start = true;
          neg_start_event.index.partner = neg_end_event_index;
          neg_start_event.new_location = neg_new_location;
          neg_new_location = false;

          // Add the positive start event.
          event_points[event.index.partner].index.partner =
            pos_used + neg_polygons.size()*2;
          PolygonEventPoint& pos_start_event = event_points[pos_used];
          ++pos_used;
          pos_start_event.start = true;
          pos_start_event.index.content = used_extra;
          pos_start_event.new_location = pos_new_location;
          pos_new_location = false;

          ++used_extra;
        } else {
          // Goes to just the negative child
          event_points[event.index.partner].index.partner = neg_used;
          polygon_index_map[poly_index] = neg_polygons.size();
          PolygonEventPoint& child_event = neg_event_points[neg_used];
          ++neg_used;
          child_event.index.content = neg_polygons.size();
          neg_polygons.push_back(std::move(polygon));
          child_event.new_location = neg_new_location;
          neg_new_location = false;
          child_event.start = true;
        }
      }
    } else {
      size_t start_event_index = event.index.partner;
      if (start_event_index == static_cast<size_t>(-1)) {
        // The start event was from a border polygon.
      } else if (start_event_index < neg_polygons.size()*2) {
        // The start event belongs to a negative polygon.
        assert(start_event_index < neg_used);
        PolygonEventPoint& child_event = neg_event_points[neg_used];

        child_event.start = false;
        child_event.new_location = neg_new_location;
        neg_new_location = false;
        child_event.index.content =
          neg_event_points[start_event_index].index.content;
        neg_event_points[start_event_index].index.partner = neg_used;
        ++neg_used;
      } else {
        start_event_index -= neg_polygons.size()*2;
        // This is an end event for the positive child.
        assert(start_event_index < pos_used);
        PolygonEventPoint& child_event = event_points[pos_used];

        child_event.start = false;
        child_event.new_location = pos_new_location;
        pos_new_location = false;
        child_event.index.content =
          event_points[start_event_index].index.content;
        event_points[start_event_index].index.partner = pos_used;
        ++pos_used;
      }
    }
  }
  polygons.clear();
}

template <typename ParentPolygon>
int PolygonEventPointPartition::CompareHeapLocationToMain(
    int dimension, const HomoPoint3& heap_location,
    const PolygonEventPoint& main_event, size_t extra_poly_count,
    const PolygonEventPoint* event_points,
    const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
      neg_polygons,
    const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
      pos_polygons,
    const size_t* polygon_index_map,
    const PolygonEventPoint* const neg_event_points,
    const PolygonEventPoint* const pos_event_points) {
  const HomoPoint3* main_location;
  if (main_event.start) {
    const size_t original_poly_index =
      event_points[main_event.index.partner].index.content;
    size_t mapped_index = polygon_index_map[original_poly_index];
    if (mapped_index < extra_poly_count) {
      // The parent polygon was split in two. Pick the child polygon with the
      // first event point.
      const HomoPoint3& neg_location = neg_polygons[mapped_index].min_vertex(dimension);
      const HomoPoint3& pos_location = pos_polygons[mapped_index].min_vertex(dimension);
      if (neg_location.IsLessThanComponent(dimension, pos_location)) {
        main_location = &neg_location;
      } else {
        main_location = &pos_location;
      }
    } else if (mapped_index < neg_polygons.size()) {
      main_location = &neg_polygons[mapped_index].min_vertex(dimension);
    } else {
      main_location = &pos_polygons[mapped_index].min_vertex(dimension);
    }
  } else {
    size_t child_start_event_index = main_event.index.partner;
    if (child_start_event_index == static_cast<size_t>(-1)) {
      // This parent end event is from a split polygon. The corresponding
      // child start event hasn't been written to the child event array yet
      // and is still in the heap. Because start events must be processed
      // before end events, and the heap contains the start event, the heap
      // must be processed first.
      return -1;
    }
    if (child_start_event_index < neg_polygons.size()*2) {
      const size_t poly_index =
        neg_event_points[child_start_event_index].index.content;
      main_location = &neg_polygons[poly_index].max_vertex(dimension);
    } else {
      child_start_event_index -= neg_polygons.size()*2;
      const size_t poly_index =
        pos_event_points[child_start_event_index].index.content;
      main_location = &pos_polygons[poly_index].max_vertex(dimension);
    }
  }
  return heap_location.CompareComponent(dimension, *main_location);
}

template <typename ParentPolygon>
void PolygonEventPointPartition::ApplySecondary(
    int dimension, size_t parent_poly_count,
    const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
      neg_polygons,
    const std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
      pos_polygons,
    const size_t* polygon_index_map,
    PolygonEventPoint* const event_points,
    PolygonEventPoint* const neg_event_points,
    PolygonEventPoint* const pos_event_points,
    std::vector<PolygonMergeEvent>& merge_heap) const {
  PolygonEventPoint* event_points_pos = event_points;
  PolygonEventPoint* const event_points_end = event_points + parent_poly_count*2;

  const size_t extra_poly_count = GetExtraPolygonCount(parent_poly_count);
  merge_heap.resize(2*extra_poly_count);
  PolygonMergeEvent* heap_start = merge_heap.data();
  PolygonMergeEvent* heap_end = heap_start;
  PolygonMergeEventCompare<ParentPolygon> heap_compare(dimension,
                                                       neg_polygons,
                                                       pos_polygons);
  size_t neg_used = 0;
  size_t pos_used = 0;
  while (event_points_pos < event_points_end) {
    assert(event_points_pos->new_location);
    // Skip main events for border polygons
    while (event_points_pos < event_points_end) {
      if (event_points_pos->start) {
        const size_t end_event_index = event_points_pos->index.partner;
        const size_t mapped_index =
          polygon_index_map[event_points[end_event_index].index.content];
        if (mapped_index == static_cast<size_t>(-1)) {
          event_points[end_event_index].index.partner = -2;
          ++event_points_pos;
        } else {
          break;
        }
      } else {
        size_t child_start_event_index = event_points_pos->index.partner;
        if (child_start_event_index == static_cast<size_t>(-2)) {
          ++event_points_pos;
        } else {
          break;
        }
      }
    }
    if (event_points_pos == event_points_end) break;
    bool neg_new_location = true;
    bool pos_new_location = true;
    const HomoPoint3* heap_location;
    bool process_heap_events;
    bool process_main_events;
    if (heap_start != heap_end) {
      heap_location = &heap_start->GetLocation(dimension, neg_polygons,
                                               pos_polygons);
      int merge_main_comparison = CompareHeapLocationToMain(dimension,
                                                            *heap_location,
                                                            *event_points_pos,
                                                            extra_poly_count,
                                                            event_points,
                                                            neg_polygons,
                                                            pos_polygons,
                                                            polygon_index_map,
                                                            neg_event_points,
                                                            pos_event_points);
      process_heap_events = merge_main_comparison <= 0;
      process_main_events = merge_main_comparison >= 0;
    } else {
      process_heap_events = false;
      process_main_events = true;
    }
    // Process all points events at the location.
    while (process_heap_events || process_main_events) {
      // Process the merge end events up to and at the main event location.
      while (process_heap_events && !heap_start->start) {
        if (heap_start->pos_child) {
          assert(pos_used < pos_polygons.size()*2);
          pos_event_points[heap_start->partner_index].index.partner = pos_used;
          PolygonEventPoint& child_event = pos_event_points[pos_used];
          child_event.start = false;
          child_event.index.content = heap_start->polygon_index;
          child_event.new_location = pos_new_location;
          pos_new_location = false;
          ++pos_used;
        } else {
          neg_event_points[heap_start->partner_index].index.partner = neg_used;
          PolygonEventPoint& child_event = neg_event_points[neg_used];
          child_event.start = false;
          child_event.index.content = heap_start->polygon_index;
          child_event.new_location = neg_new_location;
          neg_new_location = false;
          ++neg_used;
        }
        std::pop_heap(heap_start, heap_end, heap_compare);
        --heap_end;
        if (heap_start == heap_end ||
            !heap_location->IsEquivalentComponent(dimension,
              heap_start->GetLocation(dimension, neg_polygons,
                                      pos_polygons))) {
          process_heap_events = false;
        }
      }
      // Process the main end events at the location.
      while (process_main_events && !event_points_pos->start) {
        size_t child_start_event_index = event_points_pos->index.partner;
        if (child_start_event_index == static_cast<size_t>(-2)) {
          // Skip end events from border polygons
        } else if (child_start_event_index < neg_polygons.size()*2) {
          assert(child_start_event_index < neg_used);
          PolygonEventPoint& child_event = neg_event_points[neg_used];
          const size_t poly_index =
            neg_event_points[child_start_event_index].index.content;
          neg_event_points[child_start_event_index].index.partner = neg_used;
          ++neg_used;
          child_event.start = false;
          child_event.index.content = poly_index;
          child_event.new_location = neg_new_location;
          neg_new_location = false;
        } else {
          assert(pos_used < pos_polygons.size()*2);
          child_start_event_index -= neg_polygons.size()*2;
          assert(child_start_event_index < pos_used);
          PolygonEventPoint& child_event = pos_event_points[pos_used];
          const size_t poly_index =
            pos_event_points[child_start_event_index].index.content;
          pos_event_points[child_start_event_index].index.partner = pos_used;
          ++pos_used;
          child_event.start = false;
          child_event.index.content = poly_index;
          child_event.new_location = pos_new_location;
          pos_new_location = false;
        }
        ++event_points_pos;
        if (event_points_pos == event_points_end ||
            event_points_pos->new_location) {
          process_main_events = false;
        }
      }
      // Process the merge start events up to and at the main event location.
      while (process_heap_events && heap_start->start) {
        if (heap_start->pos_child) {
          assert(pos_used < pos_polygons.size()*2);
          if (heap_start->partner_index != static_cast<size_t>(-1)) {
            assert(!event_points[heap_start->partner_index].start);
            event_points[heap_start->partner_index].index.partner =
              neg_polygons.size()*2 + pos_used;
          } else {
            heap_start->start = false;
            heap_start->partner_index = pos_used;
          }
          PolygonEventPoint& child_event = pos_event_points[pos_used];
          child_event.start = true;
          child_event.index.content = heap_start->polygon_index;
          child_event.new_location = pos_new_location;
          pos_new_location = false;
          ++pos_used;
        } else {
          if (heap_start->partner_index != static_cast<size_t>(-1)) {
            event_points[heap_start->partner_index].index.partner = neg_used;
          } else {
            heap_start->start = false;
            heap_start->partner_index = neg_used;
          }
          PolygonEventPoint& child_event = neg_event_points[neg_used];
          child_event.start = true;
          child_event.index.content = heap_start->polygon_index;
          child_event.new_location = neg_new_location;
          neg_new_location = false;
          ++neg_used;
        }
        std::pop_heap(heap_start, heap_end, heap_compare);
        --heap_end;
        if (!heap_end->start) {
          // Re-add the merge event for the end event.
          ++heap_end;
          std::push_heap(heap_start, heap_end, heap_compare);
        }
        if (heap_start == heap_end ||
            !heap_location->IsEquivalentComponent(dimension,
              heap_start->GetLocation(dimension, neg_polygons,
                                      pos_polygons))) {
          process_heap_events = false;
        }
      }
      // Process the main start events at the location.
      while (process_main_events && event_points_pos->start) {
        const size_t end_event_index = event_points_pos->index.partner;
        const size_t mapped_index =
          polygon_index_map[event_points[end_event_index].index.content];
        if (mapped_index == static_cast<size_t>(-1)) {
          // Skip start events from border polygons
          event_points[end_event_index].index.partner = -2;
        } else if (mapped_index < extra_poly_count) {
          // This polygon was split into both children.
          const BSPPolygon<AABBConvexPolygon<ParentPolygon>>& neg_polygon =
            neg_polygons[mapped_index];
          const BSPPolygon<AABBConvexPolygon<ParentPolygon>>& pos_polygon =
            pos_polygons[mapped_index];
          const int end_comparison =
            neg_polygon.max_vertex(dimension).CompareComponent(
                dimension, pos_polygon.max_vertex(dimension));
          // Add the negative child start event to the heap.
          heap_end->start = true;
          heap_end->pos_child = false;
          heap_end->polygon_index = mapped_index;
          if (end_comparison <= 0) {
            heap_end->partner_index = -1;
          } else {
            heap_end->partner_index = end_event_index;
          }
          ++heap_end;
          std::push_heap(heap_start, heap_end, heap_compare);
          // Add the positive child start event to the heap.
          heap_end->start = true;
          heap_end->pos_child = true;
          heap_end->polygon_index = mapped_index;
          if (end_comparison > 0) {
            heap_end->partner_index = -1;
          } else {
            heap_end->partner_index = end_event_index;
          }
          ++heap_end;
          std::push_heap(heap_start, heap_end, heap_compare);
          event_points[end_event_index].index.partner = -1;
          assert(static_cast<size_t>(heap_end - heap_start) <=
                 2*extra_poly_count);
          process_heap_events = true;
          heap_location = &heap_start->GetLocation(dimension, neg_polygons,
                                                   pos_polygons);
        } else if (mapped_index < neg_polygons.size()) {
          // This polygon went into only the negative child. Add the negative
          // child start event.
          PolygonEventPoint& neg_event = neg_event_points[neg_used];
          neg_event.start = true;
          neg_event.index.content = mapped_index;
          neg_event.new_location = neg_new_location;
          neg_new_location = false;
          // Tell the parent end event to update the negative start event.
          event_points[end_event_index].index.partner = neg_used;
          ++neg_used;
        } else {
          // This polygon went into only the positive child. Add the positive
          // child start event.
          assert(pos_used < pos_polygons.size()*2);
          PolygonEventPoint& pos_event = pos_event_points[pos_used];
          pos_event.start = true;
          pos_event.index.content = mapped_index - neg_polygons.size();
          pos_event.new_location = pos_new_location;
          pos_new_location = false;
          // Tell the parent end event to update the positive start event.
          event_points[end_event_index].index.partner =
            pos_used + neg_polygons.size()*2;
          ++pos_used;
        }
        ++event_points_pos;
        if (event_points_pos == event_points_end ||
            event_points_pos->new_location) {
          process_main_events = false;
        }
      }
    }
  }
  assert(heap_start == heap_end);
}

}  // walnut

#endif // WALNUT_POLYGON_EVENT_POINT_H__
