#ifndef WALNUT_VTK_WALNUT_TO_VTK_MESH_H__
#define WALNUT_VTK_WALNUT_TO_VTK_MESH_H__

#include <unordered_map>

#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolygon.h>

#include "walnut/connected_polygon.h"
#include "walnut/redirectable_value.h"
#include "walnut/vertex_double_point3_mapper.h"

namespace walnut {

// Converts a vector of walnut polygons into a VTK polydata output
//
// `Polygon` must inherit from walnut::ConvexPolygon.
template<typename Polygon>
vtkSmartPointer<vtkPolyData> WalnutToVTKMesh(
    const std::vector<Polygon>& mesh) {
  auto poly_data = vtkSmartPointer<vtkPolyData>::New();
  SaveWalnutMeshToVTK(mesh, poly_data);
  return poly_data;
}

template<typename Polygon>
void SaveWalnutMeshToVTK(const std::vector<Polygon>& mesh,
                         vtkPolyData* save_to) {
  static_assert(
      std::is_base_of<typename Polygon::ConnectedPolygonRep,
                      Polygon>::value,
      "Polygon must inherit from ConnectedPolygon");
  static_assert(
      IsConnectedPolygon<typename Polygon::ConnectedPolygonRep>::value,
      "Polygon must inherit from ConnectedPolygon");

  auto value_factory = [](const DoublePoint3& p) -> vtkIdType {
    return 0;
  };
  auto value_merger = [](RedirectableValue<vtkIdType>& a,
                         RedirectableValue<vtkIdType>& b) {
    a.Redirect(b);
  };
  using Mapper =
    VertexDoublePoint3Mapper<decltype(value_factory), decltype(value_merger)>;
  Mapper mapper(value_factory, value_merger);
  // Add all of the edges to the map and merge any equivalent `HomoPoint3`s
  // that have different `DoublePoint3` representations.
  for (const Polygon& polygon : mesh) {
    for (const auto& edge : polygon.edges()) {
      mapper.Map(edge);
    }
  }

  std::unordered_map<DoublePoint3, RedirectableValue<vtkIdType>>& map =
    mapper.map;
  // Now that all points have been inserted and merged into `map`, they can be
  // added to `points`.
  auto points = vtkSmartPointer<vtkPoints>::New();
  for (std::pair<const DoublePoint3, RedirectableValue<vtkIdType>>& p : map) {
    if (p.second.IsPrimary()) {
      *p.second = points->InsertNextPoint(p.first.x, p.first.y, p.first.z);
    }
  }

  auto cell_array = vtkSmartPointer<vtkCellArray>::New();
  for (const Polygon& polygon : mesh) {
    auto converted_polygon = vtkSmartPointer<vtkPolygon>::New();
    converted_polygon->GetPointIds()->SetNumberOfIds(polygon.vertex_count());
    for (size_t i = 0; i < polygon.vertex_count(); ++i) {
      converted_polygon->GetPointIds()->SetId(
          i, *map[polygon.vertex(i).GetDoublePoint3()]);
    }
    cell_array->InsertNextCell(converted_polygon);
  }

  save_to->SetPoints(points);
  save_to->SetPolys(cell_array);
}

} // walnut

#endif // WALNUT_VTK_WALNUT_TO_VTK_MESH_H__
