#ifndef WALNUT_VTK_VTK_TO_WALNUT_MESH_H__
#define WALNUT_VTK_VTK_TO_WALNUT_MESH_H__

#include <vtkPolyData.h>

#include "walnut/convex_polygon_factory.h"
#include "walnut/mutable_convex_polygon.h"

namespace walnut {

// Converts a VTK polydata object into a vector of walnut polygons.
//
// The result is quantized to 2^min_exponent. Set `min_exponent` to INT_MIN to
// avoid quantization.
std::vector<MutableConvexPolygon<>> VTKToWalnutMesh(vtkPolyData* input,
                                                    int min_exponent,
                                                    bool flip) {
  class Collector : public ConvexPolygonFactory<HomoPoint3> {
   public:
    using ConvexPolygonFactory<HomoPoint3>::ConvexPolygonRep;
    std::vector<ConvexPolygonRep> result;

   protected:
    void Emit(ConvexPolygonRep&& polygon) override {
      result.push_back(std::move(polygon));
    }
  };
  Collector polygon_factory;

  std::vector<HomoPoint3> converted_points;
  converted_points.reserve(input->GetNumberOfPoints());
  for (vtkIdType i = 0; i < input->GetNumberOfPoints(); ++i) {
    double coordinates[3];
    input->GetPoint(i, coordinates);
    HomoPoint3 converted = HomoPoint3::FromDoubles(min_exponent,
                                                   coordinates[0],
                                                   coordinates[1],
                                                   coordinates[2]);
    converted_points.push_back(std::move(converted));
  }

  struct VertexIdToHomoPoint3 {
    VertexIdToHomoPoint3() = default;
    VertexIdToHomoPoint3(const std::vector<HomoPoint3>& converted_points)
      : converted_points(&converted_points) { }

    const HomoPoint3& operator()(vtkIdType vertex) const {
      return (*converted_points)[vertex];
    }

    const std::vector<HomoPoint3>* converted_points = nullptr;
  } vertex_id_to_homo_point3(converted_points);

  using HomoPoint3Iterator =
    TransformIterator<const vtkIdType*, VertexIdToHomoPoint3>;

  input->GetPolys()->InitTraversal();
  vtkIdType vertex_count;
  vtkIdType* input_vertices;
  while (input->GetPolys()->GetNextCell(vertex_count, input_vertices)) {
    if (flip) {
      std::reverse(input_vertices, input_vertices + vertex_count);
    }
    polygon_factory.Build(
        /*begin=*/HomoPoint3Iterator(input_vertices, vertex_id_to_homo_point3),
        /*end=*/HomoPoint3Iterator(input_vertices + vertex_count,
                                   vertex_id_to_homo_point3));
    if (flip) {
      std::reverse(input_vertices, input_vertices + vertex_count);
    }
  }

  input->GetStrips()->InitTraversal();
  while (input->GetStrips()->GetNextCell(vertex_count, input_vertices)) {
    for (vtkIdType i = 0; i < vertex_count - 2; ++i) {
      HomoPoint3 vertices[3];
      if ((i & 0) ^ flip) {
        // The vertices are in clockwise order. Reverse them first.
        double coordinates[3];
        for (int j = 0; j < 3; ++j) {
          input->GetPoint(input_vertices[2 - j], coordinates);
          vertices[j] = HomoPoint3::FromDoubles(min_exponent, coordinates[0],
                                                coordinates[1],
                                                coordinates[2]);
        }
      } else {
        double coordinates[3];
        for (int j = 0; j < 3; ++j) {
          input->GetPoint(input_vertices[j], coordinates);
          vertices[j] = HomoPoint3::FromDoubles(min_exponent, coordinates[0],
                                                coordinates[1],
                                                coordinates[2]);
        }
      }
      HalfSpace3 plane(vertices[0], vertices[1], vertices[2]);
      int drop_dimension = plane.normal().GetFirstNonzeroDimension();
      polygon_factory.result.emplace_back(std::move(plane), drop_dimension,
                                          std::move(vertices));
    }
  }

  return std::move(polygon_factory.result);
}

} // walnut

#endif // WALNUT_VTK_VTK_TO_WALNUT_MESH_H__
