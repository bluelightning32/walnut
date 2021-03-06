#ifndef WALNUT_VISUALIZATIONS_POINTS_ACTOR_H__
#define WALNUT_VISUALIZATIONS_POINTS_ACTOR_H__

#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkActor.h>
#include <vtkSmartPointer.h>

#include "visualization_window.h"
#include "walnut/homo_point3.h"

namespace walnut {

class PointsActor {
 public:
  PointsActor(VisualizationWindow& window, double r, double g, double b,
              double a);

  void SetNumberOfPoints(vtkIdType count);

  vtkIdType AddPoint(double x, double y, double z);

  vtkIdType AddPoint(const HomoPoint3& p) {
    double w(p.w());
    return AddPoint(double(p.x()) / w, double(p.y()) / w, double(p.z()) / w);
  }

  void SetPoint(vtkIdType index, double x, double y, double z);

  void SetPoint(vtkIdType index, const HomoPoint3& p) {
    double w(p.w());
    SetPoint(index, double(p.x()) / w, double(p.y()) / w, double(p.z()) / w);
  }

 private:
  vtkSmartPointer<vtkPoints> points_ = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> verticies_ =
    vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkPolyData> poly_data_ =
    vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkActor> actor_;
};

} // walnut

#endif // WALNUT_VISUALIZATIONS_POINTS_ACTOR_H__
