#include "walnut/mesh_plane_repairer.h"

#include "walnut/nudging_plane_builder.h"
#include "walnut/plucker_line.h"

namespace walnut {

void MeshPlaneRepairer::VertexInfo::TryAddAdjacent(
    const MeshPlaneRepairer::Polygon* polygon) {
  if (plane_count < 3) {
    // Don't allow duplicate planes
    for (size_t i = 0; i < plane_count; ++i) {
      if (adjacent[i]->plane.IsSameOrOpposite(polygon->plane)) return;
    }
    adjacent[plane_count] = polygon;
    ++plane_count;
  }
}

void MeshPlaneRepairer::Clear() {
  vertex_info_storage_.clear();
  vertex_info_map_.clear();

  polygon_vertices_.clear();
  polygons_.clear();
  active_polygon_vertices_.clear();
}

void MeshPlaneRepairer::AddVertex(const HomoPoint3& vertex) {
  active_polygon_vertices_.push_back(AddVertexInfo(vertex));
}

void MeshPlaneRepairer::AddVertex(HomoPoint3&& vertex) {
  active_polygon_vertices_.push_back(AddVertexInfo(std::move(vertex)));
}

MeshPlaneRepairer::VertexInfo* MeshPlaneRepairer::AddVertexInfo(
    const HomoPoint3& vertex) {
  auto it = vertex_info_map_.find(vertex);
  if (it != vertex_info_map_.end()) {
    return it->second;
  }
  HomoPoint3 reduced(vertex);
  reduced.Reduce();
  it = vertex_info_map_.find(vertex);
  if (it != vertex_info_map_.end()) {
    vertex_info_map_.emplace(vertex, it->second);
    return it->second;
  }
  vertex_info_storage_.emplace_back(reduced);
  vertex_info_map_.emplace(std::move(reduced), &vertex_info_storage_.back());
  vertex_info_map_.emplace(vertex, &vertex_info_storage_.back());
  return &vertex_info_storage_.back();
}

MeshPlaneRepairer::VertexInfo* MeshPlaneRepairer::AddVertexInfo(
    HomoPoint3&& vertex) {
  auto it = vertex_info_map_.find(vertex);
  if (it != vertex_info_map_.end()) {
    return it->second;
  }
  HomoPoint3 reduced(vertex);
  reduced.Reduce();
  it = vertex_info_map_.find(vertex);
  if (it != vertex_info_map_.end()) {
    vertex_info_map_.emplace(std::move(vertex), it->second);
    return it->second;
  }
  vertex_info_storage_.emplace_back(reduced);
  vertex_info_map_.emplace(std::move(reduced), &vertex_info_storage_.back());
  vertex_info_map_.emplace(std::move(vertex), &vertex_info_storage_.back());
  return &vertex_info_storage_.back();
}

void MeshPlaneRepairer::FinishFacet() {
  using ActiveVertexIterator = std::vector<VertexInfo*>::iterator;
  ActiveVertexIterator remaining_begin = active_polygon_vertices_.begin();
  ActiveVertexIterator remaining_end = active_polygon_vertices_.end();

  assert (remaining_begin != remaining_end);

  while (true) {
    ActiveVertexIterator accepted_from_begin_begin = remaining_begin;
    ActiveVertexIterator accepted_from_end_end = remaining_end;

    polygons_.emplace_back();
    Polygon& polygon = polygons_.back();

    // Start a new range.
    NudgingPlaneBuilder plane_builder;
    // Add one vertex from the end of the range first.
    {
      VertexInfo& vertex = **(remaining_end - 1);
      if (vertex.CanAddAdjacent()) {
        plane_builder.AddUnconstrained(&vertex.point);
      } else {
        plane_builder.TryAddConstrained(&vertex.point);
      }
      --remaining_end;
    }

    // The outer loop first tries to expand the polygon from the beginning of
    // the range, then the inner loop tries to expand it from the end of the
    // range again.
    for (; remaining_begin != remaining_end; ++remaining_begin) {
      VertexInfo& vertex = **remaining_begin;
      if (vertex.CanAddAdjacent()) {
        plane_builder.AddUnconstrained(&vertex.point);
      } else if (!plane_builder.TryAddConstrained(&vertex.point)) {
        // This vertex was rejected, which means the plane is set. Try adding as
        // many vertices from the end of the range as possible.
        for (; ; --remaining_end) {
          VertexInfo& vertex = **(remaining_end - 1);
          if (vertex.CanAddAdjacent()) {
            plane_builder.AddUnconstrained(&vertex.point);
          } else if (!plane_builder.TryAddConstrained(&vertex.point)) {
            break;
          }
        }
        break;
      }
    }

    assert(accepted_from_begin_begin != remaining_begin);

    polygon.plane = std::move(plane_builder).Build();
    // Copy the beginning portion of the accepted range.
    for (ActiveVertexIterator it = accepted_from_begin_begin;
         it != remaining_begin; ++it) {
      (*it)->TryAddAdjacent(&polygon);
      polygon_vertices_.push_back(*it);
    }

    // Copy the end portion of the accepted range in reverse order.
    for (ActiveVertexIterator it = accepted_from_end_end;
         it != remaining_end;) {
      --it;
      (*it)->TryAddAdjacent(&polygon);
      polygon_vertices_.push_back(*it);
    }

    polygon.vertex_end = polygon_vertices_.size();

    if (remaining_begin == remaining_end) break;
    // Include one vertex from the end and beginning of the range in the next
    // polygon.
    --remaining_begin;
    ++remaining_end;
  }

  active_polygon_vertices_.clear();
}

void MeshPlaneRepairer::FinalizeVertices() {
  for (VertexInfo& vertex : vertex_info_storage_) {
    vertex.Finalize();
  }
}

void MeshPlaneRepairer::VertexInfo::Finalize() {
  // If the point is already coincident with all adjacent planes, leave the
  // point as is.
  //
  // This optimization is important because `point` is already reduced.
  for (size_t i = 0; ; ++i) {
    if (i == plane_count) return;
    if (!adjacent[i]->plane.IsCoincident(point)) break;
  }

  assert(plane_count >= 2);
  PluckerLine line(adjacent[0]->plane, adjacent[1]->plane);
  if (plane_count == 3) {
    point = line.Intersect(adjacent[2]->plane);
  } else {
    point = line.Intersect(HalfSpace3(line.d(), point));
  }
  point.Reduce();
}

}  // walnut
