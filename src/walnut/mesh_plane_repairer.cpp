#include "walnut/mesh_plane_repairer.h"

#include "walnut/nudging_plane_builder.h"
#include "walnut/plucker_line.h"

namespace walnut {

void MeshPlaneRepairerBase::VertexInfo::TryAddAdjacent(
    const HalfSpace3* polygon) {
  if (plane_count < 3) {
    // Don't allow duplicate planes
    for (size_t i = 0; i < plane_count; ++i) {
      if (adjacent[i]->IsSameOrOpposite(*polygon)) {
        return;
      }
    }
    adjacent[plane_count] = polygon;
    ++plane_count;

    if (plane_count == 3) {
      Finalize();
    }
  }
}

void MeshPlaneRepairerBase::Clear() {
  vertex_info_storage_.clear();
  vertex_info_map_.clear();
}

MeshPlaneRepairerBase::VertexInfo* MeshPlaneRepairerBase::AddVertexInfo(
    const HomoPoint3& vertex) {
  auto it = vertex_info_map_.find(vertex);
  if (it != vertex_info_map_.end()) {
    return it->second;
  }
  HomoPoint3 reduced(vertex);
  reduced.Reduce();
  it = vertex_info_map_.find(reduced);
  if (it != vertex_info_map_.end()) {
    vertex_info_map_.emplace(vertex, it->second);
    return it->second;
  }
  vertex_info_storage_.emplace_back(reduced);
  vertex_info_map_.emplace(std::move(reduced), &vertex_info_storage_.back());
  vertex_info_map_.emplace(vertex, &vertex_info_storage_.back());
  return &vertex_info_storage_.back();
}

MeshPlaneRepairerBase::VertexInfo* MeshPlaneRepairerBase::AddVertexInfo(
    HomoPoint3&& vertex) {
  auto it = vertex_info_map_.find(vertex);
  if (it != vertex_info_map_.end()) {
    return it->second;
  }
  HomoPoint3 reduced(vertex);
  reduced.Reduce();
  it = vertex_info_map_.find(reduced);
  if (it != vertex_info_map_.end()) {
    vertex_info_map_.emplace(std::move(vertex), it->second);
    return it->second;
  }
  vertex_info_storage_.emplace_back(reduced);
  vertex_info_map_.emplace(std::move(reduced), &vertex_info_storage_.back());
  vertex_info_map_.emplace(std::move(vertex), &vertex_info_storage_.back());
  return &vertex_info_storage_.back();
}

HalfSpace3 MeshPlaneRepairerBase::GetNextPlanarRange(
    ActiveVertexIterator& first, ActiveVertexIterator& last) {
  // Start a new range.
  NudgingPlaneBuilder plane_builder;
  // Add one vertex from the end of the range first.
  {
    VertexInfo& vertex = **(last - 1);
    if (vertex.CanAddAdjacent()) {
      plane_builder.AddUnconstrained(&vertex.point);
    } else {
      plane_builder.TryAddConstrained(&vertex.point);
    }
    --last;
  }

  // The outer loop first tries to expand the polygon from the beginning of
  // the range, then the inner loop tries to expand it from the end of the
  // range again.
  for (; first != last; ++first) {
    VertexInfo& vertex = **first;
    if (vertex.CanAddAdjacent()) {
      plane_builder.AddUnconstrained(&vertex.point);
    } else if (!plane_builder.TryAddConstrained(&vertex.point)) {
      // This vertex was rejected, which means the plane is set. Try adding as
      // many vertices from the end of the range as possible.
      for (; ; --last) {
        VertexInfo& vertex = **(last - 1);
        if (vertex.CanAddAdjacent()) {
          plane_builder.AddUnconstrained(&vertex.point);
        } else if (!plane_builder.TryAddConstrained(&vertex.point)) {
          break;
        }
      }
      break;
    }
  }
  return std::move(plane_builder).Build();
}

void MeshPlaneRepairerBase::FinalizeVertices() {
  for (VertexInfo& vertex : vertex_info_storage_) {
    if (vertex.plane_count != 3) vertex.Finalize();
  }
}

void MeshPlaneRepairerBase::VertexInfo::Finalize() {
  // If the point is already coincident with all adjacent planes, leave the
  // point as is.
  //
  // This optimization is important because `point` is already reduced.
  for (size_t i = 0; ; ++i) {
    if (i == plane_count) return;
    if (!adjacent[i]->IsCoincident(point)) break;
  }

  assert(plane_count >= 1);
  if (plane_count == 1) {
    PluckerLine line(point, point.AddOffset(adjacent[0]->normal(), BigInt(1)));
    point = line.Intersect(*adjacent[0]);
  } else {
    PluckerLine line(*adjacent[0], *adjacent[1]);
    if (plane_count == 3) {
      point = line.Intersect(*adjacent[2]);
    } else {
      point = line.Intersect(HalfSpace3(line.d(), point));
    }
  }
  point.Reduce();
}

}  // walnut
