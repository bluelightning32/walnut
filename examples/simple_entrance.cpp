#include "bsp_visualization.h"
#include "visualization_window.h"

#include "walnut/aabb.h"
#include "walnut/vtkWalnutBooleanFilter.h"

#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>

namespace {

const walnut::Point3 cube_top(0, 0, 11);
const walnut::Point3 cube_bottom(0, 0, -11);
const walnut::Point3 cube_north(0, 10, 3);
const walnut::Point3 cube_north_west(-9, 5, -4);
const walnut::Point3 cube_south_west(-9, -5, 4);
const walnut::Point3 cube_south(0, -10, -3);
const walnut::Point3 cube_south_east(9, -5, 4);
const walnut::Point3 cube_north_east(9, 5, -4);

const walnut::Point3* cube_peripheral_points[6] = {
  &cube_north,
  &cube_north_west,
  &cube_south_west,
  &cube_south,
  &cube_south_east,
  &cube_north_east,
};

std::vector<walnut::HalfSpace3> GetTiltedCubeSplitPlanes() {
  std::vector<walnut::HalfSpace3> split_planes;
  for (int i = 0; i < 6; i += 2) {
    const walnut::Point3* this_point = cube_peripheral_points[i];
    const walnut::Point3* next_point = cube_peripheral_points[(i + 2)%6];

    split_planes.emplace_back(*this_point, *next_point, cube_top);
  }
  for (int i = 1; i < 6; i += 2) {
    const walnut::Point3* this_point = cube_peripheral_points[i];
    const walnut::Point3* next_point = cube_peripheral_points[(i + 2)%6];

    split_planes.emplace_back(*next_point, *this_point, cube_bottom);
  }

  return split_planes;
}

} // namespace

int main(int argc, char *argv[]) {
  auto cu = vtkSmartPointer<vtkCubeSource>::New();
  cu->SetXLength(10);
  cu->SetYLength(5);
  cu->SetZLength(10);
  cu->SetCenter(0, -5, 0);

  cu->Update();
  cu->GetOutputPort();

  std::vector<walnut::MutableConvexPolygon<>> walnut_mesh =
    walnut::AABB(-12, -12, -12, 12, 12, 12, /*denom=*/1).GetWalls();
  walnut::BSPTree<> tree;
  walnut::BSPContentId id = tree.AllocateId();
  tree.AddContents(id, walnut_mesh);

  std::vector<walnut::HalfSpace3> cube_splits = GetTiltedCubeSplitPlanes();
  walnut::BSPNode<>* pos = &tree.root;
  for (int i = 0; i < 3; ++i) {
    pos->Split(cube_splits[i]);
    pos = pos->negative_child();
  }

  walnut::PolygonFilter filter(id);
  auto error_log = [](const std::string& error) {
    std::cerr << error << std::endl;
    assert(false);
  };
  walnut::ConnectingVisitor<decltype(filter)> visitor(filter, error_log);
  tree.Traverse(visitor);

  walnut::VisualizationWindow window("simple_entrance");
  walnut::BSPVisualization tree_visualization(
      window,
      /*bounding_box=*/walnut::AABB(-28, -28, -70, 28, 28, 70, 1),
      /*labelling_box=*/walnut::AABB(-13, -13, -20, 13, 13, 20, 1),
      tree);
  tree_visualization.AddContent(id, walnut_mesh);
  tree_visualization.UseTopDownView();

  window.Run();

  return 0;
}
