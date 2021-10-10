#include "bsp_visualization.h"
#include "visualization_window.h"

#include "walnut/aabb.h"

using namespace walnut;

int main(int argc, char *argv[]) {
  VisualizationWindow window("kerf_effect2");
  BSPVisualization tree_visualization(
      window,
      /*bounding_box=*/AABB(-22, -22, -70, 22, 22, 70, 1),
      /*labelling_box=*/AABB(-13, -13, -20, 13, 13, 20, 1));
  tree_visualization.AddContent(
      tree_visualization.AllocateId(),
      AABB(/*min_x=*/BSPVisualization::tilted_cube_top.x(),
           /*min_y=*/BSPVisualization::tilted_cube_top.y(),
           /*min_z=*/BSPVisualization::tilted_cube_bottom.z(),
           /*max_x=*/BSPVisualization::tilted_cube_north_east.x(),
           /*max_y=*/BSPVisualization::tilted_cube_north.y(),
           /*max_z=*/BSPVisualization::tilted_cube_top.z() + 1));
  tree_visualization.AddContent(
      tree_visualization.AllocateId(),
      AABB(/*min_x=*/BSPVisualization::tilted_cube_south_west.x(),
           /*min_y=*/BSPVisualization::tilted_cube_top.y(),
           /*min_z=*/BSPVisualization::tilted_cube_bottom.z(),
           /*max_x=*/BSPVisualization::tilted_cube_top.x(),
           /*max_y=*/BSPVisualization::tilted_cube_north.y(),
           /*max_z=*/BSPVisualization::tilted_cube_top.z() + 1));
  tree_visualization.AddContent(
      tree_visualization.AllocateId(),
      AABB(/*min_x=*/BSPVisualization::tilted_cube_south_west.x(),
           /*min_y=*/BSPVisualization::tilted_cube_south.y(),
           /*min_z=*/BSPVisualization::tilted_cube_bottom.z(),
           /*max_x=*/BSPVisualization::tilted_cube_top.x(),
           /*max_y=*/BSPVisualization::tilted_cube_top.y(),
           /*max_z=*/BSPVisualization::tilted_cube_top.z() + 1));
  tree_visualization.AddContent(
      tree_visualization.AllocateId(),
      AABB(/*min_x=*/BSPVisualization::tilted_cube_top.x(),
           /*min_y=*/BSPVisualization::tilted_cube_south.y(),
           /*min_z=*/BSPVisualization::tilted_cube_bottom.z(),
           /*max_x=*/BSPVisualization::tilted_cube_south_east.x(),
           /*max_y=*/BSPVisualization::tilted_cube_top.y(),
           /*max_z=*/BSPVisualization::tilted_cube_top.z() + 1));

  BSPNode<>* new_split_pos = tree_visualization.SplitToTiltedCubeTopPlanes();
  BSPNode<>* old_split_pos = &tree_visualization.full_tree().root;
  for (int i = 0; i < 3; ++i) {
    Vector3 normal = old_split_pos->split().normal();
    // Lower the value of the z component of the normal by 2/3rds (if
    // normalized).
    normal.x() *= 3;
    normal.y() *= 3;
    normal.z() *= 2;
    new_split_pos->Split(HalfSpace3(
          normal, /*coincident=*/BSPVisualization::tilted_cube_top));
    new_split_pos = new_split_pos->negative_child();
    old_split_pos = old_split_pos->negative_child();
  }

  tree_visualization.UpdateShapes();
  tree_visualization.UseTopDownView();

  window.Run();

  return 0;
}
