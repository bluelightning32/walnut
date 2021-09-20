#include "bsp_visualization.h"
#include "visualization_window.h"

#include "walnut/aabb.h"

int main(int argc, char *argv[]) {
  walnut::VisualizationWindow window("simple_entrance");
  walnut::BSPVisualization tree_visualization(
      window,
      /*bounding_box=*/walnut::AABB(-16, -16, -70, 16, 16, 70, 1),
      /*labelling_box=*/walnut::AABB(-13, -13, -20, 13, 13, 20, 1));
  walnut::BSPContentId id = tree_visualization.AllocateId();
  tree_visualization.AddContent(id, walnut::AABB(-12, -12, -12, 12, 12, 12, /*denom=*/1));

  tree_visualization.SplitToTiltedCubeTopPlanes();
  tree_visualization.FinishSplitting(walnut::PolygonFilter(id));

  tree_visualization.UseTopDownView();

  window.Run();

  return 0;
}
