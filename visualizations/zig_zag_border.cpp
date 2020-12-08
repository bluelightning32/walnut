#include "walnut/bsp_tree.h"
#include "walnut/half_space3.h"
#include "walnut/point3.h"
#include "walnut/rectangular_prism.h"

#include "mesh_adapter.h"
#include "visualization_window.h"

#include <vtkCleanPolyData.h>

constexpr const double pi = 3.14159265358979323846;

std::vector<walnut::BSPTree<>::OutputPolygon> CreateCellBorder() {
  walnut::BSPTree<> tree;
  using NodeRep = typename walnut::BSPTree<>::BSPNodeRep;
  std::vector<bool> node_path;

  // The first index in the array is which vector to use. For each vector,
  // the first element is a component that will later be distributed between
  // x and z. The second element is a y value.
  double y_rotation[4][2];
  double y_rotation_angles[4] = {-pi/3, -pi/128, pi/128, pi/3};
  for (int i = 0; i < 4; ++i) {
    y_rotation[i][0] = cos(y_rotation_angles[i]);
    y_rotation[i][1] = sin(y_rotation_angles[i]);
  }
  static constexpr double kDist = 4;
  static constexpr double kDenom = 10000;
  NodeRep* leaf = &tree.root;
  static constexpr int kZigZagCount = 8;
  for (int i = 0; i < kZigZagCount; ++i) {
    for (int j = 0; j < 4; ++j) {
      double angle = pi * (i*2 + (j >= 2)) / (kZigZagCount * 2);
      double x = y_rotation[j][0] * cos(angle) * kDenom;
      double y = y_rotation[j][1] * kDenom;
      double z = y_rotation[j][0] * sin(angle) * kDenom;

      walnut::HalfSpace3<> split(x, y, z, /*w=*/kDist * kDenom);
      leaf->Split(split);
      // Include the side of the cell away from split's normal. That is the
      // side closer to the origin.
      leaf = leaf->negative_child();
      node_path.push_back(false);
    }
  }
  walnut::RectangularPrism<> bounding_box(walnut::Point3<>(-8, -8, 0),
                                          walnut::Point3<>(8, 8, 10));
  return tree.GetNodeBorder(node_path.begin(), node_path.end(), bounding_box);
}

int main(int argc, char *argv[]) {
  auto converted_mesh = ConvertWalnutMesh(CreateCellBorder());
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
  bounds[0] = -5;
  // xmax
  bounds[1] = 5;
  // ymin
  bounds[2] = -5;
  // ymax
  bounds[3] = 5;
  // zmin
  bounds[4] = 0;
  // zmax
  bounds[5] = 5;
  vtkSmartPointer<vtkCubeAxesActor> axes = window.Axes(bounds, /*padding=*/0);
  window.UseTopDownView();
  // The top down view ensures that nothing is going to overlap the axis
  // labels. So the label size can be reduced without hurting readability.
  axes->SetScreenSize(10);
  window.Run();

  return 0;
}
