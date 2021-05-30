#include "visualization_window.h"
#include "walnut/vtkWalnutBooleanFilter.h"

#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>

int main(int argc, char *argv[]) {
  auto cu = vtkSmartPointer<vtkCubeSource>::New();
  cu->SetXLength(10);
  cu->SetYLength(5);
  cu->SetZLength(10);
  cu->SetCenter(0, -5, 0);

  auto cyl = vtkSmartPointer<vtkCylinderSource>::New();
  cyl->SetResolution(32);
  cyl->SetHeight(6);
  cyl->SetRadius(5);
  cyl->SetCenter(0, 0, 0);

  auto cu2 = vtkSmartPointer<vtkCubeSource>::New();
  cu2->SetXLength(10);
  cu2->SetYLength(5);
  cu2->SetZLength(10);
  cu2->SetCenter(0, 6, 0);

  auto bool_filter = vtkSmartPointer<walnut::vtkWalnutBooleanFilter>::New();
  bool_filter->AddInputConnection(cu->GetOutputPort());
  bool_filter->AddInputConnection(cyl->GetOutputPort());
  bool_filter->AddInputConnection(cu2->GetOutputPort());
  bool_filter->SetMinExponent(-8);

  walnut::VisualizationWindow window;
  auto actor = window.AddShape(bool_filter->GetOutputPort(), 1, 0.8, 0.8, 0.6);
  window.AddWireframe(bool_filter->GetOutputPort());
  window.AddShapeNormals(bool_filter->GetOutputPort(), /*scale=*/1);

  int mode = 0;
  walnut::ObserverRegistration switch_mode = window.AddKeyPressObserver(
      [&bool_filter, &cu2, &mode](char key) {
      if (key == 'm') {
        ++mode;
        mode %= 3;
        switch(mode) {
        case 0:
          bool_filter->SetOperationToUnion();
          break;
        case 1:
          // There is no intersection between all 3 objects. So remove the last
          // one and compute the intersection between the first two.
          bool_filter->RemoveInputConnection(0, 2);
          bool_filter->SetOperationToIntersection();
          break;
        case 2:
          // Add back the object that was removed in mode 1.
          bool_filter->AddInputConnection(cu2->GetOutputPort());
          bool_filter->SetOperationToDifference();
          break;
        }
        return true;
      }
      return false;
    });

  double bounds[6];
  // xmin
  bounds[0] = -5;
  // xmax
  bounds[1] = 5;
  // ymin
  bounds[2] = -10;
  // ymax
  bounds[3] = 10;
  // zmin
  bounds[4] = -5;
  // zmax
  bounds[5] = 5;
  window.Axes(bounds, /*padding=*/0);
  window.Run();

  return 0;
}
