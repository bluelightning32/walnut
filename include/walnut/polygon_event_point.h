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

struct PolygonEventPointPartition {
  // The index of the event point that defines the split plane.
  //
  // This is also the number of events from the beginning of the event point
  // array should go to the negative child only.
  size_t split_index;

  size_t cost;

  // The number of polygons that go to the negative child (including polygons
  // that are split in two).
  size_t neg_poly_count;

  // The number of polygons that go to the positive child (including polygons
  // that are split in two).
  size_t pos_poly_count;

  size_t GetExtraPolygonCount(size_t parent_poly_count) const {
    return neg_poly_count + pos_poly_count - parent_poly_count;
  }

  size_t GetNegEventPointCount() const {
    return neg_poly_count * 2;
  }

  size_t GetPosEventPointCount() const {
    return pos_poly_count * 2;
  }

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
      std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
        pos_polygons) const;
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
  best.cost = -1;
  best.split_index = -1;
  MLogNEstimator neg_estimator, pos_estimator;
  bool last_event_was_end = false;
  size_t i;
  for (i = 0; i < polygons.size()*2; ++i) {
    const PolygonEventPoint& event_point = event_points[i];
    if (event_point.start) {
      if (last_event_was_end) {
        const size_t cost = neg_estimator.Estimate(neg_total, neg_log_count) +
                            pos_estimator.Estimate(pos_total, pos_log_count);
        if (cost < best.cost) {
          best.cost = cost;
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
    const size_t cost = neg_estimator.Estimate(neg_total, neg_log_count) +
                        pos_estimator.Estimate(pos_total, pos_log_count);
    if (cost < best.cost) {
      best.cost = cost;
      best.split_index = i;
      best.neg_poly_count = neg_total;
      best.pos_poly_count = pos_total;
    }
  }
  return best;
}

template <typename ParentPolygon>
void PolygonEventPointPartition::ApplyPrimary(
    int dimension,
    PolygonEventPoint* event_points,
    std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>& polygons,
    size_t* polygon_index_map,
    PolygonEventPoint* neg_event_points,
    std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>& neg_polygons,
    std::vector<BSPPolygon<AABBConvexPolygon<ParentPolygon>>>&
      pos_polygons) const {
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
  //    event_points.end:         index of start event in pos_event_points
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

  // This will be initialized when the first event point is processed.
  bool neg_new_location;
  bool pos_new_location = true;
  // Go through all of the events before the split. `split_index` itself points
  // to the last end event in the neg child.
  size_t i = 0;
  for (; i <= split_index; ++i) {
    const PolygonEventPoint& event = event_points[i];
    if (event.new_location) {
      neg_new_location = true;
    }
    if (event.start) {
      const size_t poly_index =
        event_points[event.index.partner].index.content;
      if (event.index.partner <= split_index) {
        // Goes to just the negative child
        event_points[event.index.partner].index.partner = neg_used;
        polygon_index_map[poly_index] = neg_polygons.size();
        PolygonEventPoint& child_event = neg_event_points[neg_used];
        ++neg_used;
        child_event.index.content = neg_polygons.size();
        neg_polygons.push_back(std::move(polygons[poly_index]));
        child_event.new_location = neg_new_location;
        neg_new_location = false;
        child_event.start = true;
      } else {
        // Goes to both children
        auto child_polygons =
          std::move(polygons[poly_index]).CreateSplitChildren(
              polygons[poly_index].GetSplitInfo(split_plane));
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
        event_points[event.index.partner].index.partner = pos_used;
        PolygonEventPoint& pos_start_event = event_points[pos_used];
        ++pos_used;
        pos_start_event.start = true;
        pos_start_event.index.content = used_extra;
        pos_start_event.new_location = pos_new_location;
        pos_new_location = false;

        ++used_extra;
      }
    } else {
      // In this loop, all end events are for polygons that are just in the
      // neg_child.
      const size_t start_event_index = event.index.partner;
      assert(start_event_index < neg_used);
      PolygonEventPoint& child_event = neg_event_points[neg_used];

      child_event.start = false;
      child_event.new_location = neg_new_location;
      neg_new_location = false;
      child_event.index.content =
        neg_event_points[start_event_index].index.content;
      neg_event_points[start_event_index].index.partner = neg_used;
      ++neg_used;
    }
  }
  for (; i < polygons.size()*2; ++i) {
    const PolygonEventPoint& event = event_points[i];
    if (event.new_location) {
      neg_new_location = true;
      pos_new_location = true;
    }
    if (event.start) {
      const size_t poly_index =
        event_points[event.index.partner].index.content;
      // Start events after the split index go to just the positive child
      event_points[event.index.partner].index.partner = pos_used;
      polygon_index_map[poly_index] = pos_polygons.size();
      PolygonEventPoint& child_event = event_points[pos_used];
      ++pos_used;
      child_event.index.content = pos_polygons.size();
      pos_polygons.push_back(std::move(polygons[poly_index]));
      child_event.new_location = pos_new_location;
      pos_new_location = false;
      child_event.start = true;
    } else {
      size_t start_event_index = event.index.partner;
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
  polygons.clear();
}

}  // walnut

#endif // WALNUT_POLYGON_EVENT_POINT_H__
