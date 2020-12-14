#include "points_actor.h"

#include <vtkProperty.h>

namespace walnut {

PointsActor::PointsActor(VisualizationWindow& window, double r, double g,
                         double b, double a) {
  poly_data_->SetPoints(points_);
  poly_data_->SetVerts(verticies_);

  actor_ = window.AddShape(poly_data_, r, g, b, a);
  actor_->GetProperty()->SetPointSize(20);
}

void PointsActor::SetNumberOfPoints(vtkIdType count) {
  points_->SetNumberOfPoints(count);
  for (vtkIdType i = verticies_->GetNumberOfCells(); i < count; ++i) {
    vtkIdType ids[1] = { i };
    verticies_->InsertNextCell(1, ids);
  }
}

vtkIdType PointsActor::AddPoint(double x, double y, double z) {
  vtkIdType id = points_->InsertNextPoint(x, y, z);
  vtkIdType ids[1] = { id };
  verticies_->InsertNextCell(1, ids);
  points_->Modified();
  return id;
}

void PointsActor::SetPoint(vtkIdType index, double x, double y, double z) {
  points_->SetPoint(index, x, y, z);
  points_->Modified();
}

} // walnut
