#ifndef WALNUT_VISUALIZATIONS_MESH_ADAPTER_H__
#define WALNUT_VISUALIZATIONS_MESH_ADAPTER_H__

#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolygon.h>

#include "walnut/convex_polygon.h"

namespace walnut {

// Converts a vector of walnut polygons into a VTK polydata output
//
// `Polygon` must inherit from walnut::ConvexPolygon.
//
// The output will likely have duplicate points. It should be run through
// vtkCleanPolyData.
template<typename Polygon>
vtkSmartPointer<vtkPolyData> ConvertWalnutMesh(
    const std::vector<Polygon>& mesh) {
  static_assert(
      std::is_base_of<ConvexPolygon<Polygon::point3_bits,
                                    typename Polygon::VertexData>,
                      Polygon>::value,
      "Polygon must inherit from ConvexPolygon");
  using HomoPoint3Rep = typename Polygon::HomoPoint3Rep;

  auto points = vtkSmartPointer<vtkPoints>::New();
  auto cell_array = vtkSmartPointer<vtkCellArray>::New();
  for (const Polygon& polygon : mesh) {
    auto converted_polygon = vtkSmartPointer<vtkPolygon>::New();
    converted_polygon->GetPointIds()->SetNumberOfIds(polygon.vertex_count());
    for (size_t i = 0; i < polygon.vertex_count(); ++i) {
      HomoPoint3Rep point = polygon.vertex(i);
      double w = point.w();
      vtkIdType point_id = points->InsertNextPoint(double(point.x()) / w,
                                                   double(point.y()) / w,
                                                   double(point.z()) / w);
      converted_polygon->GetPointIds()->SetId(i, point_id);
    }
    cell_array->InsertNextCell(converted_polygon);
  }

  auto poly_data = vtkSmartPointer<vtkPolyData>::New();
  poly_data->SetPoints(points);
  poly_data->SetPolys(cell_array);
  return poly_data;
}

template<typename Polygon>
typename Polygon::HomoPoint3Rep GetTopPoint(const std::vector<Polygon>& mesh) {
  using HomoPoint3Rep = typename Polygon::HomoPoint3Rep;
  HomoPoint3Rep top(0, 0, 0, 0);

  for (const Polygon& polygon : mesh) {
    for (size_t i = 0; i < polygon.vertex_count(); ++i) {
      const HomoPoint3Rep& point = polygon.vertex(i);
      if (top.w().IsZero() || HomoPoint3Rep::TopnessLt(top, point)) {
        top = point;
      }
    }
  }
  return top;
}

} // walnut

#endif // WALNUT_VISUALIZATIONS_MESH_ADAPTER_H__
