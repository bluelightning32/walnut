#include "render.h"

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
  Render(GenerateShape(), "extrusion");
  return 0;
}
