#include "normals_actor.h"

#include <vtkProperty.h>

namespace walnut {

NormalsActor::NormalsActor(VisualizationWindow& window,
                           vtkAlgorithmOutput* shape,
                           double scale, bool start3d)
    : use_3d(start3d), window(&window) {
  actor_2d = window.AddShapeNormals(shape, /*scale=*/scale,
                                    /*normals3d=*/false);
  actor_3d = window.AddShapeNormals(shape, /*scale=*/scale,
                                    /*normals3d=*/true);

  actor_2d->SetVisibility(visible && !use_3d);
  actor_3d->SetVisibility(visible && use_3d);
  switch_mode = window.AddKeyPressObserver(
      [this](char key) {
      if (key == 'n') {
        use_3d ^= true;
        actor_2d->SetVisibility(visible && !use_3d);
        actor_3d->SetVisibility(visible && use_3d);
        return true;
      }
      return false;
    });
}

NormalsActor& NormalsActor::operator=(NormalsActor&& other) {
  use_3d = other.use_3d;
  visible = other.visible;
  actor_2d = std::move(other.actor_2d);
  actor_3d = std::move(other.actor_3d);
  window = other.window;
  other.window = nullptr;

  other.switch_mode.Clear();
  if (window) {
    switch_mode = window->AddKeyPressObserver(
        [this](char key) {
        if (key == 'n') {
          use_3d ^= true;
          actor_2d->SetVisibility(visible && !use_3d);
          actor_3d->SetVisibility(visible && use_3d);
          return true;
        }
        return false;
      });
  }
  return *this;
}

void NormalsActor::SetVisibility(bool visible) {
  this->visible = visible;
  actor_2d->SetVisibility(visible && !use_3d);
  actor_3d->SetVisibility(visible && use_3d);
}

void NormalsActor::SetColor(double r, double g, double b) {
  actor_2d->GetProperty()->SetColor(r, g, b);
  actor_3d->GetProperty()->SetColor(r, g, b);
}

} // walnut
