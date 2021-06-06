#ifndef WALNUT_VISUALIZATIONS_VISUALIZATION_WINDOW_H__
#define WALNUT_VISUALIZATIONS_VISUALIZATION_WINDOW_H__

#include <functional>

#include <vtkActor.h>
#include <vtkAlgorithmOutput.h>
#include <vtkCubeAxesActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSliderRepresentation2D.h>
#include <vtkSliderWidget.h>
#include <vtkSmartPointer.h>

#include "observer_registration.h"

namespace walnut {

class VisualizationWindow {
 public:
  VisualizationWindow();

  vtkSmartPointer<vtkActor> AddShape(vtkPolyData* shape,
                                     double r, double g, double b, double a);

  vtkSmartPointer<vtkActor> AddShape(vtkAlgorithmOutput* shape,
                                     double r, double g, double b, double a);

  // Call `GetProperty()->SetColor` on the result to change the color.
  vtkSmartPointer<vtkActor> AddWireframe(vtkAlgorithmOutput* shape);

  // Call `GetProperty()->SetColor` on the result to change the color.
  vtkSmartPointer<vtkActor> AddShapeNormals(vtkAlgorithmOutput* shape,
                                            double scale=3,
                                            bool normals3d=true);

  // Call `GetBounds` on a previously added actor to get the bounds.
  vtkSmartPointer<vtkCubeAxesActor> Axes(double content_bounds[6],
                                         double padding = 5);

  // Creates a new 2D slider and adds it to the window.
  std::pair<vtkSmartPointer<vtkSliderWidget>,
            vtkSmartPointer<vtkSliderRepresentation2D>> Create2DSliderWidget();

  // Moves the camera so that it looks down on the origin.
  //
  // Also switches the camera to use an orthogonal projection instead of a
  // perspective view.
  void UseTopDownView();

  void Zoom(double factor);

  void Run();

  // Adds an observer that is notified of key presses.
  //
  // The function should return true if the window needs to be re-rendered, or
  // false if the keypress was ignored.
  ObserverRegistration AddKeyPressObserver(std::function<bool(char)> observer);

  // Adds an observer that is notified of key presses.
  //
  // The function should return true if the window needs to be re-rendered, or
  // false if the keypress was ignored.
  ObserverRegistration AddKeyPressObserver(
      std::function<bool(const char*)> observer);

  void Redraw();

 private:
  vtkSmartPointer<vtkRenderer> renderer_;
  vtkSmartPointer<vtkRenderWindow> render_window_;
  vtkSmartPointer<vtkRenderWindowInteractor> interactor_;
};

} // walnut

#endif // WALNUT_VISUALIZATIONS_VISUALIZATION_WINDOW_H__
