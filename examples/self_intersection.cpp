#include "visualization_window.h"
#include "walnut/big_int.h"
#include "walnut/bsp_tree.h"
#include "walnut/connecting_visitor.h"
#include "walnut/extrude.h"
#include "walnut/homo_point3.h"
#include "walnut/mutable_convex_polygon.h"
#include "walnut/vector3.h"
#include "walnut/walnut_to_vtk_mesh.h"

#include <vtkCleanPolyData.h>

int main(int argc, char *argv[]) {
  std::vector<walnut::HomoPoint3> bottom_cap{
    walnut::HomoPoint3(0, 0, 0, 1),
    walnut::HomoPoint3(1, 2, 0, 1),
    walnut::HomoPoint3(2, -1, -1, 1),
    walnut::HomoPoint3(3, 2, 0, 1),
    walnut::HomoPoint3(4, 0, 0, 1),
  };

  std::vector<walnut::MutableConvexPolygon<>> walnut_mesh = 
    Extrude(bottom_cap, /*direction=*/walnut::Vector3(0, 0, 5),
            /*direction_denom=*/walnut::BigInt(1));

  walnut::BSPTree<> tree;
  walnut::BSPContentId id = tree.AllocateId();
  tree.AddContents(id, walnut_mesh);

  walnut::PolygonFilter filter(id);
  auto error_log = [](const std::string& error) {
    std::cerr << error << std::endl;
    assert(false);
  };
  walnut::ConnectingVisitor<walnut::PolygonFilter> visitor(filter, error_log);
  tree.Traverse(visitor);
  visitor.FilterEmptyPolygons();
  auto filtered_mesh = WalnutToVTKMesh(visitor.TakePolygons());

  auto cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
  auto converted_mesh = WalnutToVTKMesh(walnut_mesh);

  cleaner->SetInputData(converted_mesh);

  walnut::VisualizationWindow window;
  auto actor = window.AddShape(cleaner->GetOutputPort(), 1, 0.8, 0.8, 0.6);
  window.AddWireframe(cleaner->GetOutputPort());
  window.AddShapeNormals(cleaner->GetOutputPort(), /*scale=*/1);

  bool use_filtered = false;
  walnut::ObserverRegistration switch_mode = window.AddKeyPressObserver(
      [&use_filtered, &cleaner, &filtered_mesh, &converted_mesh](char key) {
      if (key == 'f') {
        use_filtered ^= true;
        if (use_filtered) {
          cleaner->SetInputData(filtered_mesh);
        } else {
          cleaner->SetInputData(converted_mesh);
        }
        return true;
      }
      return false;
    });

  double bounds[6];
  // xmin
  bounds[0] = 0;
  // xmax
  bounds[1] = 6;
  // ymin
  bounds[2] = 0;
  // ymax
  bounds[3] = 6;
  // zmin
  bounds[4] = 0;
  // zmax
  bounds[5] = 6;
  window.Axes(bounds, /*padding=*/0);
  window.Run();

  return 0;
}
