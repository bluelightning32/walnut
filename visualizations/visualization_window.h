#ifndef WALNUT_VISUALIZATIONS_VISUALIZATION_WINDOW_H__
#define WALNUT_VISUALIZATIONS_VISUALIZATION_WINDOW_H__

#include <vtkActor.h>
#include <vtkAlgorithmOutput.h>
#include <vtkCubeAxesActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>

namespace walnut {

class VisualizationWindow {
 public:
  VisualizationWindow();

  vtkSmartPointer<vtkActor> AddShape(vtkSmartPointer<vtkAlgorithmOutput> shape,
                                     double r, double g, double b, double a);

  // Call `GetProperty()->SetColor` on the result to change the color.
  vtkSmartPointer<vtkActor> AddWireframe(
      vtkSmartPointer<vtkAlgorithmOutput> shape);

  // Call `GetProperty()->SetColor` on the result to change the color.
  vtkSmartPointer<vtkActor> AddShapeNormals(
      vtkSmartPointer<vtkAlgorithmOutput> shape, double scale=3);

  // Call `GetBounds` on a previously added actor to get the bounds.
  vtkSmartPointer<vtkCubeAxesActor> Axes(double content_bounds[6],
                                         double padding = 5);

  // Moves the camera so that it looks down on the origin.
  //
  // Also switches the camera to use an orthogonal projection instead of a
  // perspective view.
  void UseTopDownView();

  void Zoom(double factor);

  void Run();

 private:
  vtkSmartPointer<vtkRenderer> renderer_;
  vtkSmartPointer<vtkRenderWindow> render_window_;
  vtkSmartPointer<vtkRenderWindowInteractor> interactor_;
};

} // walnut

#endif // WALNUT_VISUALIZATIONS_VISUALIZATION_WINDOW_H__
