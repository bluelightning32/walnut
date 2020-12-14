#ifndef WALNUT_VISUALIZATIONS_NORMALS_ACTOR_H__
#define WALNUT_VISUALIZATIONS_NORMALS_ACTOR_H__

#include "visualization_window.h"

namespace walnut {

struct NormalsActor {
 public:
  NormalsActor(walnut::VisualizationWindow& window,
               vtkAlgorithmOutput* shape,
               double scale=3, bool start3d = false);

  vtkSmartPointer<vtkActor> actor_2d;
  vtkSmartPointer<vtkActor> actor_3d;

  walnut::ObserverRegistration switch_mode;
};

} // walnut

#endif // WALNUT_VISUALIZATIONS_NORMALS_ACTOR_H__
