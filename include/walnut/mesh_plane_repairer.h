#ifndef WALNUT_MESH_PLANE_REPAIRER_H__
#define WALNUT_MESH_PLANE_REPAIRER_H__

#include <array>
#include <deque>
#include <vector>
#include <unordered_map>
// For std::pair
#include <utility>

#include "walnut/half_space3.h"
#include "walnut/homo_point3.h"
#include "walnut/transform_iterator.h"

namespace walnut {

class MeshPlaneRepairerProducer;

// This class takes a mesh as input and fixes each facet so that it is planar.
// It tries to fix the mesh by adjusting vertices. However, if the mesh is too
// constrained because too many broken facets meet at the same point, then it
// will resort to splitting the facets.
class MeshPlaneRepairer {
 public:
  struct Polygon {
    HalfSpace3 plane;
    size_t vertex_end;
  };

  struct VertexInfo {
    VertexInfo(const HomoPoint3& point) : point(point) { }

    // Adjust `point` to be coincident with the adjacent planes.
    void Finalize();

    bool CanAddAdjacent() {
      return plane_count < 3;
    }

    void TryAddAdjacent(const Polygon* polygon);

    std::array<const Polygon*, 3> adjacent;
    // The number of used entries in `adjacent`.
    size_t plane_count = 0;
    // This may be changed until `plane_count` equals 3, or the mesh has been
    // finalized.
    HomoPoint3 point;
  };

  // Adds a new vertex to the active facet
  //
  // After the last vertex is added for the facet, `FinishFacet` must be
  // called.
  void AddVertex(const HomoPoint3& vertex);

  // Adds a new vertex to the active facet
  //
  // After the last vertex is added for the facet, `FinishFacet` must be
  // called.
  void AddVertex(HomoPoint3&& vertex);

  // Finishes the active facet
  void FinishFacet();

  MeshPlaneRepairerProducer FinalizeMesh() &&;

  void Clear();

 private:
  void FinalizeVertices();

  // Returns the VertexInfo for `vertex` from `vertex_info_map_`.
  //
  // Looks up the existing entry, if it already exists, otherwise adds it.
  VertexInfo* AddVertexInfo(const HomoPoint3& vertex);

  // Returns the VertexInfo for `vertex` from `vertex_info_map_`.
  //
  // Looks up the existing entry, if it already exists, otherwise adds it.
  VertexInfo* AddVertexInfo(HomoPoint3&& vertex);

  // `std::deque` is used because entries in `vertex_info_map_` point to elements
  // in here.
  std::deque<VertexInfo> vertex_info_storage_;
  std::unordered_map<HomoPoint3, VertexInfo*, ReducedHomoPoint3Hasher>
    vertex_info_map_;

  std::vector<const VertexInfo*> polygon_vertices_;
  // `std::deque` is used because entries in `vertex_info_map_` point to elements
  // in here.
  std::deque<Polygon> polygons_;

  // These are the vertices of the polygon that's currently being processed.
  // `FinishFacet` will break it into 1 or more planar polygons and add them to
  // `polygon_vertices_` and `polygons_`.
  std::vector<VertexInfo*> active_polygon_vertices_;
};

class MeshPlaneRepairerProducer {
 public:
  using Polygon = MeshPlaneRepairer::Polygon;
  using VertexInfo = MeshPlaneRepairer::VertexInfo;
  struct VertexIteratorTransformer {
    const HomoPoint3& operator()(
        const VertexInfo* entry) const {
      return entry->point;
    }
  };
  using VertexInfoIteratorPtr =
    std::vector<const VertexInfo*>::const_iterator;
  using VertexIterator = TransformIterator<VertexInfoIteratorPtr,
                                           VertexIteratorTransformer>;

  MeshPlaneRepairerProducer(
      const std::vector<const VertexInfo*>& polygon_vertices,
      std::deque<Polygon>::iterator polygon_pos,
      std::deque<Polygon>::iterator polygon_end)
    : polygon_vertices_(polygon_vertices),
      polygon_pos_(polygon_pos),
      polygon_end_(polygon_end)
  {
  }

  bool HasMorePolygons() const {
    return polygon_pos_ != polygon_end_;
  }

  // Returns the plane of the next polygon, along with pointers to the
  // vertices.
  //
  // This may only be called if `HasMorePolygons` returns true. Calling this
  // advances the internal iterator position.
  //
  // `first_vertex` and `last_vertex` are output variables.
  HalfSpace3&& GetNextPolygon(VertexIterator& first_vertex,
                              VertexIterator& last_vertex) {
    HalfSpace3& result = polygon_pos_->plane;
    first_vertex = VertexIterator(polygon_vertices_.begin() + vertex_begin_,
                                  VertexIteratorTransformer());
    last_vertex = VertexIterator(polygon_vertices_.begin() +
                                 polygon_pos_->vertex_end,
                                 VertexIteratorTransformer());
    vertex_begin_ = polygon_pos_->vertex_end;
    ++polygon_pos_;
    return std::move(result);
  }

 private:
  const std::vector<const VertexInfo*>& polygon_vertices_;
  size_t vertex_begin_ = 0;
  std::deque<Polygon>::iterator polygon_pos_;
  std::deque<Polygon>::iterator polygon_end_;
};

inline MeshPlaneRepairerProducer MeshPlaneRepairer::FinalizeMesh() && {
  FinalizeVertices();
  return MeshPlaneRepairerProducer(polygon_vertices_, polygons_.begin(),
                                   polygons_.end());
}

}  // walnut

#endif // WALNUT_MESH_PLANE_REPAIRER_H__
