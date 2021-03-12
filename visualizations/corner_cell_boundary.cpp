#include "walnut/aabb.h"
#include "walnut/bsp_tree.h"
#include "walnut/half_space3.h"
#include "walnut/point3.h"

#include "mesh_adapter.h"
#include "visualization_window.h"

#include <vtkCleanPolyData.h>

int main(int argc, char *argv[]) {
  walnut::Point3<> p[] = {
    walnut::Point3<>{1, 0, 0},
    walnut::Point3<>{0, 1, 0},
    walnut::Point3<>{0, 0, 1},
  };

  walnut::BSPTree<> tree;
  walnut::HalfSpace3<> split(p[0], p[1], p[2]);
  tree.root.Split(split);
  walnut::AABB<> bounding_box(walnut::Point3<>(0, 0, 0),
                                          walnut::Point3<>(2, 2, 2));
  std::vector<bool> node_path = {false};
  walnut::BSPTree<>::MappedBSPNode node_border_root;
  walnut::BSPTree<>::MappedBSPNode* node_border_leaf = tree.GetNodeBorder(
      node_path.begin(), node_path.end(), bounding_box, node_border_root);
  std::vector<walnut::BSPTree<>::OutputPolygon> mesh = node_border_leaf->contents();
  mesh.insert(mesh.end(), node_border_leaf->border_contents().begin(),
              node_border_leaf->border_contents().end());

  auto converted_mesh = ConvertWalnutMesh(mesh);
  auto cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
  cleaner->SetToleranceIsAbsolute(true);
  cleaner->SetAbsoluteTolerance(0.000001);
  cleaner->SetInputData(converted_mesh);

  walnut::VisualizationWindow window;
  auto actor = window.AddShape(cleaner->GetOutputPort(), 1, 0.8, 0.8, 0.6);
  window.AddWireframe(cleaner->GetOutputPort());
  window.AddShapeNormals(cleaner->GetOutputPort(), /*scale=*/1);

  double bounds[6];
  // xmin
  bounds[0] = 0;
  // xmax
  bounds[1] = 1;
  // ymin
  bounds[2] = 0;
  // ymax
  bounds[3] = 1;
  // zmin
  bounds[4] = 0;
  // zmax
  bounds[5] = 1;
  window.Axes(bounds, /*padding=*/0);
  window.Run();

  return 0;
}
