#include "bsp_visualization.h"
#include "visualization_window.h"

#include "walnut/aabb.h"

using namespace walnut;

int main(int argc, char *argv[]) {
  VisualizationWindow window("kerf_effect1");
  BSPVisualization tree_visualization(
      window,
      /*bounding_box=*/AABB(-22, -22, -70, 22, 22, 70, 1),
      /*labelling_box=*/AABB(-13, -13, -20, 13, 13, 20, 1));
  tree_visualization.AddContent(
      tree_visualization.AllocateId(),
      AABB(/*min_x=*/BSPVisualization::tilted_cube_top.x(),
           /*min_y=*/BSPVisualization::tilted_cube_top.y() + 2,
           /*min_z=*/BSPVisualization::tilted_cube_bottom.z(),
           /*max_x=*/BSPVisualization::tilted_cube_north_east.x(),
           /*max_y=*/BSPVisualization::tilted_cube_north.y(),
           /*max_z=*/BSPVisualization::tilted_cube_top.z() + 1));
  tree_visualization.AddContent(
      tree_visualization.AllocateId(),
      AABB(/*min_x=*/BSPVisualization::tilted_cube_south_west.x(),
           /*min_y=*/BSPVisualization::tilted_cube_top.y() + 2,
           /*min_z=*/BSPVisualization::tilted_cube_bottom.z(),
           /*max_x=*/BSPVisualization::tilted_cube_top.x(),
           /*max_y=*/BSPVisualization::tilted_cube_north.y(),
           /*max_z=*/BSPVisualization::tilted_cube_top.z() + 1));

  BSPNode<>* new_split_pos = tree_visualization.SplitToTiltedCubeTopPlanes();
  new_split_pos->Split(HalfSpace3(/*normal=*/Vector3(0, -1, 0), BigInt(-6)));

  tree_visualization.UpdateShapes();
  tree_visualization.UseTopDownView();

  window.Run();

  return 0;
}
