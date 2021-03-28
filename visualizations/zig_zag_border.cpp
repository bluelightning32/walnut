#include <vtkCleanPolyData.h>
#include <vtkMapper.h>
#include <vtkPolyLineSource.h>
#include <vtkProperty.h>
#include <vtkProperty2D.h>
#include <vtkTextProperty.h>

#include "function_command.h"
#include "mesh_adapter.h"
#include "normals_actor.h"
#include "points_actor.h"
#include "visualization_window.h"
#include "walnut/aabb.h"
#include "walnut/bsp_tree.h"
#include "walnut/half_space3.h"
#include "walnut/point3.h"

constexpr const double pi = 3.14159265358979323846;

std::vector<walnut::BSPTree<>::OutputPolygon> CreateCellBorder(
    double top, double vert_angle) {
  walnut::BSPTree<> tree;
  using NodeRep = typename walnut::BSPTree<>::BSPNodeRep;
  std::vector<bool> node_path;

  // The first index in the array is which vector to use. For each vector,
  // the first element is a component that will later be distributed between
  // x and z. The second element is a y value.
  double y_rotation[4][2];
  double y_rotation_angles[4] = {-pi/3, -pi/32, pi/32, pi/3};
  for (int i = 0; i < 4; ++i) {
    y_rotation[i][0] = cos(y_rotation_angles[i]);
    y_rotation[i][1] = sin(y_rotation_angles[i]);
  }
  static constexpr double kDist = 4;
  static constexpr double kDenom = 10000;
  NodeRep* leaf = &tree.root;
  static constexpr int kZigZagCount = 4;
  for (int i = 0; i < kZigZagCount; ++i) {
    for (int j = 0; j < 4; ++j) {
      double angle = pi * (i*2 + (j >= 2)) / (kZigZagCount * 2);
      double x = y_rotation[j][0] * cos(angle) * kDenom;
      double y = y_rotation[j][1] * kDenom;
      double z = y_rotation[j][0] * sin(angle) * kDenom;

      walnut::HalfSpace3 split(x, y, z, /*w=*/kDist * kDenom);
      leaf->Split(split);
      // Include the side of the cell away from split's normal. That is the
      // side closer to the origin.
      leaf = leaf->negative_child();
      node_path.push_back(false);
    }
  }

  // Cut the top off of the cell.
  walnut::HalfSpace3 split(kDenom * 1 * sin(vert_angle * pi/2), 0,
                           kDenom / 1 * cos(vert_angle * pi/2),
                           /*w=*/top * kDenom);
  leaf->Split(split);
  // Include the side of the cell away from split's normal. That is the
  // side closer to the origin.
  leaf = leaf->negative_child();
  node_path.push_back(false);

  walnut::AABB<> bounding_box(walnut::Point3(-8, -8, 0),
                              walnut::Point3(8, 8, 10));
  walnut::BSPTree<>::MappedBSPNode node_border_root;
  walnut::BSPTree<>::MappedBSPNode* node_border_leaf = tree.GetNodeBorder(
      node_path.begin(), node_path.end(), bounding_box, node_border_root);
  std::vector<walnut::BSPTree<>::OutputPolygon> mesh = node_border_leaf->contents();
  mesh.insert(mesh.end(), node_border_leaf->border_contents().begin(),
              node_border_leaf->border_contents().end());
  return mesh;
}

int main(int argc, char *argv[]) {
  auto cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
  static constexpr double kInitialTop = 3;
  std::vector<walnut::BSPTree<>::OutputPolygon> mesh =
    CreateCellBorder(kInitialTop, 0);
  auto converted_mesh = ConvertWalnutMesh(mesh);
  cleaner->SetInputData(converted_mesh);
  cleaner->SetToleranceIsAbsolute(true);
  cleaner->SetAbsoluteTolerance(0.000001);

  walnut::VisualizationWindow window;
  auto actor = window.AddShape(cleaner->GetOutputPort(), 1, 0.8, 0.8, 0.6);
  window.AddWireframe(cleaner->GetOutputPort());
  walnut::NormalsActor normals(window, cleaner->GetOutputPort(), /*scale=*/1);

  walnut::PointsActor top_point(window, 1, 0, 0, 1);
  top_point.AddPoint(GetTopPoint(mesh));

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

  vtkSmartPointer<vtkSliderWidget> height_widget;
  vtkSmartPointer<vtkSliderRepresentation2D> height_rep;
  {
    auto pair = window.Create2DSliderWidget();
    height_widget = std::move(pair.first);
    height_rep = std::move(pair.second);
  }

  height_rep->SetMinimumValue(0);
  height_rep->SetMaximumValue(4.1);
  height_rep->SetValue(kInitialTop);

  height_rep->GetPoint1Coordinate()->SetValue(75, 40);
  height_rep->GetPoint2Coordinate()->SetValue(75, 600);

  vtkSmartPointer<vtkSliderWidget> angle_widget;
  vtkSmartPointer<vtkSliderRepresentation2D> angle_rep;
  {
    auto pair = window.Create2DSliderWidget();
    angle_widget = std::move(pair.first);
    angle_rep = std::move(pair.second);
  }

  angle_rep->SetMinimumValue(-1);
  angle_rep->SetMaximumValue(1);
  angle_rep->SetValue(0);

  angle_rep->GetPoint1Coordinate()->SetValue(125, 40);
  angle_rep->GetPoint2Coordinate()->SetValue(125, 600);

  auto update_mesh = [height_rep, angle_rep, cleaner, &top_point]() {
    std::vector<walnut::BSPTree<>::OutputPolygon> mesh =
      CreateCellBorder(height_rep->GetValue(), angle_rep->GetValue());
    auto converted_mesh = ConvertWalnutMesh(mesh);
    cleaner->SetInputData(converted_mesh);
    top_point.SetPoint(0, GetTopPoint(mesh));
  };
  vtkSmartPointer<walnut::FunctionCommand> callback =
    walnut::MakeFunctionCommand([update_mesh](
          vtkObject* caller, unsigned long event_id, void* data) {
        update_mesh();
      });
  height_widget->AddObserver(vtkCommand::InteractionEvent, callback);
  angle_widget->AddObserver(vtkCommand::InteractionEvent, callback);

  window.Run();

  return 0;
}
