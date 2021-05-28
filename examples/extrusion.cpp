// This program shows how to use the VisualizationWindow class. It does not use
// the walnut library.

#include "visualization_window.h"

#include <vtkLinearExtrusionFilter.h>
#include <vtkRegularPolygonSource.h>

vtkSmartPointer<vtkPolyDataAlgorithm> GenerateShape() {
  auto pentagon = vtkSmartPointer<vtkRegularPolygonSource>::New();
  pentagon->SetNumberOfSides(5);
  pentagon->SetCenter(0, 0, 0);
  pentagon->SetNormal(0, 1, 0);
  pentagon->SetRadius(10);

  auto extrusion = vtkSmartPointer<vtkLinearExtrusionFilter>::New();
  extrusion->SetVector(0, 1, 0);
  extrusion->SetScaleFactor(20);
  extrusion->SetInputConnection(pentagon->GetOutputPort());
  return extrusion;
}

int main(int argc, char *argv[]) {
  auto shape = GenerateShape();

  walnut::VisualizationWindow window;
  auto actor = window.AddShape(shape->GetOutputPort(), 1, 0.8, 0.8, 0.6);
  window.AddWireframe(shape->GetOutputPort());
  window.AddShapeNormals(shape->GetOutputPort());
  window.Axes(actor->GetBounds());
  window.Run();

  return 0;
}
