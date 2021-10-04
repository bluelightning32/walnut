#include "bsp_visualization.h"
#include "visualization_window.h"

#include "walnut/aabb.h"

using namespace walnut;

int main(int argc, char *argv[]) {
  VisualizationWindow window("bottom_top_check");
  BSPVisualization tree_visualization(
      window,
      /*bounding_box=*/AABB(-28, -28, -70, 28, 28, 70, 1),
      /*labelling_box=*/AABB(-13, -13, -20, 13, 13, 20, 1));
  tree_visualization.AddContent(
      tree_visualization.AllocateId(),
      AABB(/*min_x=*/-11,
           /*min_y=*/-11,
           /*min_z=*/-2,
           /*max_x=*/11,
           /*max_y=*/8,
           /*max_z=*/21,
           /*denom=*/1));
  tree_visualization.AddContent(
      tree_visualization.AllocateId(),
      AABB(/*min_x=*/-10,
           /*min_y=*/-10,
           /*min_z=*/-1,
           /*max_x=*/10,
           /*max_y=*/-6,
           /*max_z=*/20,
           /*denom=*/1));

  tree_visualization.SetMinAxesBounds(AABB(/*min_x=*/-10,
                                           /*min_y=*/-8,
                                           /*min_z=*/0,
                                           /*max_x=*/10,
                                           /*max_y=*/10,
                                           /*max_z=*/18,
                                           /*denom=*/1));

  // Back of the pyramid
  tree_visualization.full_tree().root.Split(
      HalfSpace3(/*x=*/0, /*y=*/1, /*z=*/0, /*dist=*/10));
  BSPNode<>* pos = tree_visualization.full_tree().root.negative_child();

  // Bottom of the pyramid
  pos->Split(HalfSpace3(/*x=*/0, /*y=*/0, /*z=*/-1, /*dist=*/0));
  pos = pos->negative_child();

  // Side of the pyramid
  pos->Split(HalfSpace3(Point3(0, -50, 0), Point3(0, 10, 18),
                        Point3(-10, 10, 0)));
  pos = pos->negative_child();

  // Side of the pyramid
  pos->Split(HalfSpace3(Point3(0, -50, 0), Point3(10, 10, 0),
                        Point3(0, 10, 18)));
  pos = pos->negative_child();

  // Cut the side of the pyramid deeper
  pos->Split(HalfSpace3(Point3(0, -8, 0), Point3(0, 10, 30),
                        Point3(-10, 10, 0)));
  pos = pos->negative_child();

  // Cut the side of the pyramid deeper
  pos->Split(HalfSpace3(Point3(0, -8, 0), Point3(10, 10, 0),
                        Point3(0, 10, 30)));
  pos = pos->negative_child();
  
  // Cut off the point left from the deeper cuts
  pos->Split(HalfSpace3(/*normal=*/Vector3(0, -1, 1),
                        /*coincident=*/HomoPoint3(0, 6, 17, 1)));
  pos = pos->negative_child();

  tree_visualization.UpdateShapes();
  tree_visualization.UseTopDownView();

  window.Run();

  return 0;
}
