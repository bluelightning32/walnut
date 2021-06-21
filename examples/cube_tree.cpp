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
    walnut::AABB(0, 0, 0, 10, 10, 10, /*denom=*/1).GetWalls();
  walnut::BSPTree<> tree;
  walnut::BSPContentId id = tree.AllocateId();
  tree.AddContents(id, walnut_mesh);

  std::vector<walnut::HalfSpace3> cube_splits = GetTiltedCubeSplitPlanes();
  walnut::BSPNode<>* pos = &tree.root;
  for (walnut::HalfSpace3& split_plane : cube_splits) {
    pos->Split(split_plane);
    pos = pos->negative_child();
  }

  walnut::VisualizationWindow window;
  walnut::BSPVisualization tree_visualization(window,
                                              walnut::AABB(-29, -29, -70, 29,
                                                           29, 70, 1),
                                              tree);
  tree_visualization.AddContent(id, walnut_mesh);

  double bounds[6];
  // xmin
  bounds[0] = -12;
  // xmax
  bounds[1] = 12;
  // ymin
  bounds[2] = -12;
  // ymax
  bounds[3] = 12;
  // zmin
  bounds[4] = -12;
  // zmax
  bounds[5] = 12;
  window.Axes(bounds, /*padding=*/0);

  window.UseTopDownView(bounds);
  window.Run();

  return 0;
}
