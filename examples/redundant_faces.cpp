#include "visualization_window.h"
#include "walnut/aabb.h"
#include "walnut/big_int.h"
#include "walnut/bsp_tree.h"
#include "walnut/connecting_visitor.h"
#include "walnut/homo_point3.h"
#include "walnut/mutable_convex_polygon.h"
#include "walnut/vector3.h"
#include "walnut/walnut_to_vtk_mesh.h"

#include <vtkCleanPolyData.h>

int main(int argc, char *argv[]) {
  std::vector<walnut::MutableConvexPolygon<>> cube1 = 
    walnut::AABB(-3, 0, 0, 0, 3, 3, 1).GetWalls();

  std::vector<walnut::MutableConvexPolygon<>> cube2 = 
    walnut::AABB(0, 0, 0, 3, 3, 3, 1).GetWalls();

  std::vector<walnut::MutableConvexPolygon<>> degenerate_cube = 
    walnut::AABB(0, 0, 3, 0, 3, 6, 1).GetWalls();

  std::vector<walnut::MutableConvexPolygon<>> appended_polyhedrons = cube1;
  appended_polyhedrons.insert(appended_polyhedrons.end(), cube2.begin(),
                              cube2.end());
  appended_polyhedrons.insert(appended_polyhedrons.end(),
                              degenerate_cube.begin(),
                              degenerate_cube.end());

  walnut::BSPTree<> tree;
  walnut::BSPContentId id = tree.AllocateId();
  tree.AddContents(id, appended_polyhedrons);

  walnut::PolygonFilter filter(id);
  auto error_log = [](const std::string& error) {
    std::cerr << error << std::endl;
    assert(false);
  };
  walnut::ConnectingVisitor<decltype(filter)> visitor(filter, error_log);
  tree.Traverse(visitor);
  visitor.FilterEmptyPolygons();
  auto filtered_mesh = WalnutToVTKMesh(visitor.TakePolygons());

  auto cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
  auto converted_mesh = WalnutToVTKMesh(appended_polyhedrons);

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
  bounds[0] = -3;
  // xmax
  bounds[1] = 3;
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
