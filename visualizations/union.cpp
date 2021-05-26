#include "mesh_adapter.h"
#include "visualization_window.h"

#include "walnut/bsp_tree.h"
#include "walnut/half_space3.h"

#include <vtkCleanPolyData.h>
#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>
#include <vtkQuantizePolyDataPoints.h>

int main(int argc, char *argv[]) {
  auto cu = vtkSmartPointer<vtkCubeSource>::New();
  cu->SetXLength(10);
  cu->SetYLength(5);
  cu->SetZLength(10);
  cu->SetCenter(0, -5, 0);

  auto cyl = vtkSmartPointer<vtkCylinderSource>::New();
  cyl->SetResolution(32);
  cyl->SetHeight(6);
  cyl->SetRadius(5);
  cyl->SetCenter(0, 0, 0);

  auto cu2 = vtkSmartPointer<vtkCubeSource>::New();
  cu2->SetXLength(10);
  cu2->SetYLength(5);
  cu2->SetZLength(10);
  cu2->SetCenter(0, 6, 0);

  walnut::BSPTree<> tree;
  std::vector<walnut::MutableConvexPolygon<>> polygons;

  cu->Update();
  polygons = walnut::VTKToWalnutMesh(vtkPolyData::SafeDownCast(
          cu->GetOutputDataObject(/*port=*/0)));
  walnut::BSPContentId id1 = tree.AllocateId();
  tree.AddContents(id1, std::move(polygons));

  cyl->Update();
  polygons = walnut::VTKToWalnutMesh(vtkPolyData::SafeDownCast(
          cyl->GetOutputDataObject(/*port=*/0)));
  walnut::BSPContentId id2 = tree.AllocateId();
  tree.AddContents(id2, std::move(polygons));

  cu2->Update();
  polygons = walnut::VTKToWalnutMesh(vtkPolyData::SafeDownCast(
          cu2->GetOutputDataObject(/*port=*/0)));
  walnut::BSPContentId id3 = tree.AllocateId();
  tree.AddContents(id3, std::move(polygons));

  auto cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
  cleaner->SetToleranceIsAbsolute(true);
  cleaner->SetAbsoluteTolerance(0.000001);

  auto error_log = [](const std::string& error) {
    std::cerr << error << std::endl;
    assert(false);
  };
  walnut::UnionIdsFilter filter({id1, id2, id3});
  walnut::ConnectingVisitor<decltype(filter)> visitor(filter, error_log);
  tree.Traverse(visitor);
  visitor.FilterEmptyPolygons();
  cleaner->SetInputData(walnut::WalnutToVTKMesh(visitor.TakePolygons()));

  walnut::VisualizationWindow window;
  auto actor = window.AddShape(cleaner->GetOutputPort(), 1, 0.8, 0.8, 0.6);
  window.AddWireframe(cleaner->GetOutputPort());
  window.AddShapeNormals(cleaner->GetOutputPort(), /*scale=*/1);

  double bounds[6];
  // xmin
  bounds[0] = -5;
  // xmax
  bounds[1] = 5;
  // ymin
  bounds[2] = -10;
  // ymax
  bounds[3] = 10;
  // zmin
  bounds[4] = -5;
  // zmax
  bounds[5] = 5;
  window.Axes(bounds, /*padding=*/0);
  window.Run();

  return 0;
}
