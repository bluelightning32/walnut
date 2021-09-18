#include "bsp_visualization.h"
#include "visualization_window.h"

#include "walnut/aabb.h"

using namespace walnut;

int main(int argc, char *argv[]) {
  VisualizationWindow window("split_before_after_crossing");
  BSPVisualization tree_visualization(
      window,
      /*bounding_box=*/AABB(-28, -28, -70, 28, 28, 70, 1),
      /*labelling_box=*/AABB(-13, -13, -20, 13, 13, 20, 1));
  tree_visualization.AddContent(
      tree_visualization.AllocateId(),
      AABB(/*min_x=*/BSPVisualization::tilted_cube_north_west.x(),
           /*min_y=*/BSPVisualization::tilted_cube_north.y()*3/4,
           /*min_z=*/BSPVisualization::tilted_cube_bottom.z(),
           /*max_x=*/BSPVisualization::tilted_cube_north_east.x(),
           /*max_y=*/BSPVisualization::tilted_cube_north.y() + 10,
           /*max_z=*/BSPVisualization::tilted_cube_top.z()));

  // After all of these splits, `pos` will have only 1 facet left from the
  // rectangular prism, and that facet will be vertical.
  BSPNode<>* pos = tree_visualization.SplitToTiltedCube();
  // This split will not create a crossing entrance.
  pos = tree_visualization.SplitNorthWest(pos,
                                          BSPVisualization::tilted_cube_top,
                                          BSPVisualization::tilted_cube_north, 0.5);
  // This split creates an entrance crossing.
  pos = tree_visualization.SplitNorthWest(pos,
                                          BSPVisualization::tilted_cube_top,
                                          BSPVisualization::tilted_cube_north, 7/8.0);
  // This split creates an exit crossing.
  pos = tree_visualization.SplitNorthWest(pos,
                                          BSPVisualization::tilted_cube_north_west,
                                          BSPVisualization::tilted_cube_north_west, 0);

  tree_visualization.UseTopDownView();

  window.Run();

  return 0;
}
