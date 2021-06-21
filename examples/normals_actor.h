#ifndef WALNUT_VISUALIZATIONS_NORMALS_ACTOR_H__
#define WALNUT_VISUALIZATIONS_NORMALS_ACTOR_H__

#include "visualization_window.h"

namespace walnut {

struct NormalsActor {
 public:
  NormalsActor() = default;

  NormalsActor(NormalsActor&& other) {
    *this = std::move(other);
  }

  NormalsActor(VisualizationWindow& window, vtkAlgorithmOutput* shape,
               double scale=3, bool start3d = false);

  NormalsActor(VisualizationWindow& window, vtkPolyData* arrow_data,
               double scale=3, bool start3d = false);

  NormalsActor& operator=(NormalsActor&& other);

  void SetVisibility(bool visible);
  void SetColor(double r, double g, double b);

  bool use_3d = true;
  bool visible = true;
  vtkSmartPointer<vtkActor> actor_2d;
  vtkSmartPointer<vtkActor> actor_3d;

  VisualizationWindow* window = nullptr;
  ObserverRegistration switch_mode;
};

} // walnut

#endif // WALNUT_VISUALIZATIONS_NORMALS_ACTOR_H__
