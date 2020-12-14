#include "normals_actor.h"

namespace walnut {

NormalsActor::NormalsActor(walnut::VisualizationWindow& window,
                           vtkAlgorithmOutput* shape,
                           double scale, bool start3d) {
  actor_2d = window.AddShapeNormals(shape, /*scale=*/1, /*normals3d=*/false);
  actor_3d = window.AddShapeNormals(shape, /*scale=*/1, /*normals3d=*/true);

  if (start3d) {
    actor_2d->VisibilityOff();
  } else {
    actor_3d->VisibilityOff();
  }
  switch_mode = window.AddKeyPressObserver(
      [actor_2d = actor_2d, actor_3d = actor_3d](char key) {
      if (key == 'n') {
        actor_3d->SetVisibility(!actor_3d->GetVisibility());
        actor_2d->SetVisibility(!actor_3d->GetVisibility());
        return true;
      }
      return false;
    });
}

} // walnut
