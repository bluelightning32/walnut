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
      return polygons[event_points[index.partner].index.content].min_vertex(dimension);
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
// split plane, along with the estimated cost.
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
  for (size_t i = 0; i < polygons.size()*2; ++i) {
    const PolygonEventPoint& event_point = event_points[i];
    if (event_point.start) {
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
    }
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

}  // walnut

#endif // WALNUT_POLYGON_EVENT_POINT_H__
