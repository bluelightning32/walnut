#ifndef WALNUT_MESH_PLANE_REPAIRER_H__
#define WALNUT_MESH_PLANE_REPAIRER_H__

#include <array>
#include <deque>
#include <tuple>
#include <unordered_map>
// For std::pair
#include <utility>
#include <vector>

#include "walnut/half_space3.h"
#include "walnut/homo_point3.h"
#include "walnut/transform_iterator.h"

namespace walnut {

// This is base for MeshPlaneRepairer to move as much non-templated functions
// as possible out and into the cpp file.
class MeshPlaneRepairerBase {
 public:
  struct VertexInfo {
    VertexInfo(const HomoPoint3& point) : point(point) { }

    // Adjust `point` to be coincident with the adjacent planes.
    void Finalize();

    bool CanAddAdjacent() {
      return plane_count < 3;
    }

    void TryAddAdjacent(const HalfSpace3* polygon);

    std::array<const HalfSpace3*, 3> adjacent;
    // The number of used entries in `adjacent`.
    size_t plane_count = 0;
    // This may be changed until `plane_count` equals 3, or the mesh has been
    // finalized.
    HomoPoint3 point;
  };

  size_t GetUniqueVertexCount() const {
    return vertex_info_storage_.size();
  }

  bool HasVertex(const HomoPoint3& vertex) const;

  bool IsValidState() const;

 protected:
  using ActiveVertexIterator = std::vector<VertexInfo*>::iterator;

  // Get the next range of active vertices that can be nudged onto the same
  // plane.
  //
  // On input, `first` and `last` represent the range of remaining active
  // vertices. On output, they are the vertices that were not accepted by the
  // planar range. The planar range is guaranteed to accept at least one vertex
  // from the end of the range and one from the beginning.
  static HalfSpace3 GetNextPlanarRange(ActiveVertexIterator& first, 
                                       ActiveVertexIterator& last);

  void FinalizeVertices();

  void Clear();

  // Returns the VertexInfo for `vertex` from `vertex_info_map_`.
  //
  // Looks up the existing entry, if it already exists, otherwise adds it.
  VertexInfo* AddVertexInfo(const HomoPoint3& vertex);

  // Returns the VertexInfo for `vertex` from `vertex_info_map_`.
  //
  // Looks up the existing entry, if it already exists, otherwise adds it.
  VertexInfo* AddVertexInfo(HomoPoint3&& vertex);

 private:
  // `std::deque` is used because entries in `vertex_info_map_` point to elements
  // in here.
  std::deque<VertexInfo> vertex_info_storage_;
  std::unordered_map<HomoPoint3, VertexInfo*, ReducedHomoPoint3Hasher,
                     HomoPoint3StrictEq> vertex_info_map_;
};

template <typename PolygonData = std::tuple<>>
class MeshPlaneRepairerProducer;

// This class takes a mesh as input and fixes each facet so that it is planar.
// It tries to fix the mesh by adjusting vertices. However, if the mesh is too
// constrained because too many broken facets meet at the same point, then it
// will resort to splitting the facets.
template <typename PolygonData = std::tuple<>>
class MeshPlaneRepairer : public MeshPlaneRepairerBase {
 public:
  struct Polygon {
    Polygon(HalfSpace3&& plane, const PolygonData& data)
      : plane(std::move(plane)), data(data) { }

    HalfSpace3 plane;
    size_t vertex_end;
    PolygonData data;
  };

  // Adds a new vertex to the active facet
  //
  // After the last vertex is added for the facet, `FinishFacet` must be
  // called.
  void AddVertex(const HomoPoint3& vertex) {
    active_polygon_vertices_.push_back(AddVertexInfo(vertex));
  }

  // Adds a new vertex to the active facet
  //
  // After the last vertex is added for the facet, `FinishFacet` must be
  // called.
  void AddVertex(HomoPoint3&& vertex) {
    active_polygon_vertices_.push_back(AddVertexInfo(std::move(vertex)));
  }

  // Finishes the active facet
  void FinishFacet(PolygonData data = PolygonData());

  MeshPlaneRepairerProducer<PolygonData> FinalizeMesh() &&;

  void Clear() {
    MeshPlaneRepairerBase::Clear();
    polygon_vertices_.clear();
    polygons_.clear();
    active_polygon_vertices_.clear();
  }

 private:
  std::vector<const VertexInfo*> polygon_vertices_;
  // `std::deque` is used because entries in `vertex_info_map_` point to elements
  // in here.
  std::deque<Polygon> polygons_;

  // These are the vertices of the polygon that's currently being processed.
  // `FinishFacet` will break it into 1 or more planar polygons and add them to
  // `polygon_vertices_` and `polygons_`.
  std::vector<VertexInfo*> active_polygon_vertices_;
};

template <typename PolygonData>
class MeshPlaneRepairerProducer {
 public:
  using Polygon = typename MeshPlaneRepairer<PolygonData>::Polygon;
  using VertexInfo = MeshPlaneRepairerBase::VertexInfo;
  struct VertexIteratorTransformer {
    const HomoPoint3& operator()(const VertexInfo* entry) const {
      return entry->point;
    }
  };
  using VertexInfoIteratorPtr =
    std::vector<const VertexInfo*>::const_iterator;
  using VertexIterator = TransformIterator<VertexInfoIteratorPtr,
                                           VertexIteratorTransformer>;

  MeshPlaneRepairerProducer(
      const std::vector<const VertexInfo*>& polygon_vertices,
      typename std::deque<Polygon>::iterator polygon_pos,
      typename std::deque<Polygon>::iterator polygon_end)
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
  typename std::deque<Polygon>::iterator polygon_pos_;
  typename std::deque<Polygon>::iterator polygon_end_;
};

template <typename PolygonData>
inline MeshPlaneRepairerProducer<PolygonData>
MeshPlaneRepairer<PolygonData>::FinalizeMesh() && {
  FinalizeVertices();
  return MeshPlaneRepairerProducer<PolygonData>(polygon_vertices_,
                                                polygons_.begin(),
                                                polygons_.end());
}

template <typename PolygonData>
inline void MeshPlaneRepairer<PolygonData>::FinishFacet(PolygonData data) {
  using ActiveVertexIterator = std::vector<VertexInfo*>::iterator;
  ActiveVertexIterator remaining_begin = active_polygon_vertices_.begin();
  ActiveVertexIterator remaining_end = active_polygon_vertices_.end();

  assert (remaining_begin != remaining_end);

  while (true) {
    ActiveVertexIterator accepted_from_begin_begin = remaining_begin;
    ActiveVertexIterator accepted_from_end_end = remaining_end;

    polygons_.emplace_back(GetNextPlanarRange(remaining_begin, remaining_end),
                           std::move(data));
    Polygon& polygon = polygons_.back();

    assert(accepted_from_begin_begin != remaining_begin);
    // Copy the beginning portion of the accepted range.
    for (ActiveVertexIterator it = accepted_from_begin_begin;
         it != remaining_begin; ++it) {
      (*it)->TryAddAdjacent(&polygon.plane);
      polygon_vertices_.push_back(*it);
    }

    // Copy the end portion of the accepted range in reverse order.
    for (ActiveVertexIterator it = accepted_from_end_end;
         it != remaining_end;) {
      --it;
      (*it)->TryAddAdjacent(&polygon.plane);
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

}  // walnut

#endif // WALNUT_MESH_PLANE_REPAIRER_H__
