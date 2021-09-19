#include "bsp_visualization.h"
#include "visualization_window.h"

#include "walnut/aabb.h"

using namespace walnut;

int main(int argc, char *argv[]) {
  VisualizationWindow window("min_max_check");
  BSPVisualization tree_visualization(
      window,
      /*bounding_box=*/AABB(-28, -28, -20, 28, 28, 20, 1),
      /*labelling_box=*/AABB(-13, -13, -20, 13, 13, 20, 1));
  tree_visualization.AddContent(
      tree_visualization.AllocateId(),
      AABB(/*min_x=*/-10,
           /*min_y=*/-10,
           /*min_z=*/0,
           /*max_x=*/0,
           /*max_y=*/10,
           /*max_z=*/10,
           /*denom=*/1));
  tree_visualization.AddContent(
      tree_visualization.AllocateId(),
      AABB(/*min_x=*/0,
           /*min_y=*/-10,
           /*min_z=*/0,
           /*max_x=*/10,
           /*max_y=*/10,
           /*max_z=*/10,
           /*denom=*/1));

  tree_visualization.SetMinAxesBounds(AABB(/*min_x=*/-10,
                                           /*min_y=*/-10,
                                           /*min_z=*/0,
                                           /*max_x=*/10,
                                           /*max_y=*/10,
                                           /*max_z=*/18,
                                           /*denom=*/1));

  BSPNode<>* pos = &tree_visualization.full_tree().root;

  pos->Split(HalfSpace3(/*x=*/1, /*y=*/0, /*z=*/0, /*dist=*/10));
  pos = pos->negative_child();

  pos->Split(HalfSpace3(/*x=*/0, /*y=*/1, /*z=*/0, /*dist=*/10));
  pos = pos->negative_child();

  pos->Split(HalfSpace3(/*x=*/-1, /*y=*/0, /*z=*/0, /*dist=*/10));
  pos = pos->negative_child();

  pos->Split(HalfSpace3(/*x=*/0, /*y=*/-1, /*z=*/0, /*dist=*/10));
  pos = pos->negative_child();

  // Cut the top off
  pos->Split(HalfSpace3(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/8));
  pos = pos->negative_child();

  // Leave a crease running from the top-left to the bottom-right.
  pos->Split(HalfSpace3(/*x=*/1, /*y=*/1, /*z=*/1, /*dist=*/5));
  pos = pos->negative_child();
  pos->Split(HalfSpace3(/*x=*/-1, /*y=*/-1, /*z=*/1, /*dist=*/5));
  pos = pos->negative_child();

  tree_visualization.UpdateShapes();
  tree_visualization.UseTopDownView();

  window.Run();

  return 0;
}
