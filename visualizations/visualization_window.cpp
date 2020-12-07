#include "visualization_window.h"

#include <vtkArrowSource.h>
#include <vtkApplyColors.h>
#include <vtkCamera.h>
#include <vtkCellCenters.h>
#include <vtkGlyph3D.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkProperty.h>
#include <vtkTextProperty.h>

namespace walnut {

// Apply a Color to a shape.
vtkSmartPointer<vtkAlgorithm> Color(vtkSmartPointer<vtkAlgorithmOutput> input,
                                    double r, double g, double b, double a) {
  auto filter = vtkSmartPointer<vtkApplyColors>::New();
  filter->SetInputConnection(0, input);
  filter->SetDefaultCellColor(r, g, b);
  filter->SetDefaultCellOpacity(a);
  return filter;
}

vtkSmartPointer<vtkGlyph3D> GetNormalsGlyph(
    vtkSmartPointer<vtkAlgorithmOutput> shape) {
  auto normals = vtkSmartPointer<vtkPolyDataNormals>::New();
  normals->SetInputConnection(shape);

  normals->ComputePointNormalsOff();
  normals->ComputeCellNormalsOn();

  auto centers = vtkSmartPointer<vtkCellCenters>::New();
  centers->SetInputConnection(normals->GetOutputPort());

  auto arrow = vtkSmartPointer<vtkArrowSource>::New();
  arrow->Update();

  auto glyph = vtkSmartPointer<vtkGlyph3D>::New();

  glyph->SetInputConnection(centers->GetOutputPort());
  glyph->SetSourceData(arrow->GetOutput());
  glyph->SetVectorModeToUseNormal();
  glyph->SetScaleModeToScaleByVector();
  glyph->SetScaleFactor(3);
  glyph->OrientOn();

  return glyph;
}

VisualizationWindow::VisualizationWindow() :
    renderer_(vtkSmartPointer<vtkRenderer>::New()),
    render_window_(vtkSmartPointer<vtkRenderWindow>::New()),
    interactor_(vtkSmartPointer<vtkRenderWindowInteractor>::New()) {

  renderer_->GetActiveCamera()->SetFocalPoint(0, 0, 0);
  renderer_->GetActiveCamera()->SetPosition(10, 10, 10);
  renderer_->GetActiveCamera()->SetViewUp(0, 0, 1);
  renderer_->ResetCamera();
  renderer_->SetBackground(1, 1, 1);

  render_window_->AddRenderer(renderer_);
  render_window_->SetSize(1024, 768);

  interactor_->SetRenderWindow(render_window_);

  auto interactor_style = vtkSmartPointer<vtkInteractorStyleSwitch>::New();
  interactor_style->SetCurrentStyleToTrackballCamera();
  interactor_->SetInteractorStyle(interactor_style);
}

vtkSmartPointer<vtkActor> VisualizationWindow::AddShape(
    vtkSmartPointer<vtkAlgorithmOutput> shape, double r, double g, double b,
    double a) {
  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(Color(shape, r, g, b, a)->GetOutputPort());
  mapper->SetScalarModeToUseCellFieldData();
  mapper->SelectColorArray("vtkApplyColors color");
  mapper->SetScalarVisibility(true);

  auto actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  renderer_->AddActor(actor);
  renderer_->ResetCamera();
  return actor;
}

vtkSmartPointer<vtkActor> VisualizationWindow::AddWireframe(
    vtkSmartPointer<vtkAlgorithmOutput> shape) {
  auto wireframe_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  wireframe_mapper->SetInputConnection(shape);

  auto wireframe_actor = vtkSmartPointer<vtkActor>::New();
  wireframe_actor->SetMapper(wireframe_mapper);
  wireframe_actor->GetProperty()->SetColor(0, 0, 0.25);
  wireframe_actor->GetProperty()->SetRepresentationToWireframe();
  wireframe_actor->GetProperty()->SetLineWidth(3);

  renderer_->AddActor(wireframe_actor);
  renderer_->ResetCamera();
  return wireframe_actor;
}

vtkSmartPointer<vtkActor> VisualizationWindow::AddShapeNormals(
    vtkSmartPointer<vtkAlgorithmOutput> shape) {
  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(GetNormalsGlyph(shape)->GetOutputPort());

  auto actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(0,0.5,1);

  renderer_->AddActor(actor);
  renderer_->ResetCamera();
  return actor;
}

vtkSmartPointer<vtkCubeAxesActor> VisualizationWindow::Axes(
    double content_bounds[6], double padding) {
  vtkCamera* camera = renderer_->GetActiveCamera();

  auto actor = vtkSmartPointer<vtkCubeAxesActor>::New();
  //actor->SetUseTextActor3D(1);
  double bounds[6];
  for (int i = 0; i < 6; i+=2) {
    bounds[i] = content_bounds[i] - padding;
  }
  for (int i = 1; i < 6; i+=2) {
    bounds[i] = content_bounds[i] + padding;
  }
  actor->SetBounds(bounds);
  actor->SetCamera(camera);
  actor->GetTitleTextProperty(0)->SetColor(0, 0, 0);
  actor->GetTitleTextProperty(0)->SetFontSize(48);
  actor->GetLabelTextProperty(0)->SetColor(0, 0, 0);

  actor->GetTitleTextProperty(1)->SetColor(0, 0, 0);
  actor->GetLabelTextProperty(1)->SetColor(0, 0, 0);

  actor->GetTitleTextProperty(2)->SetColor(0, 0, 0);
  actor->GetLabelTextProperty(2)->SetColor(0, 0, 0);

  actor->GetXAxesLinesProperty()->SetColor(0, 0, 0);
  actor->GetYAxesLinesProperty()->SetColor(0, 0, 0);
  actor->GetZAxesLinesProperty()->SetColor(0, 0, 0);
  actor->GetXAxesLinesProperty()->SetLineWidth(2);
  actor->GetYAxesLinesProperty()->SetLineWidth(2);
  actor->GetZAxesLinesProperty()->SetLineWidth(2);

  actor->GetXAxesGridlinesProperty()->SetColor(0, 0, 0);
  actor->GetYAxesGridlinesProperty()->SetColor(0, 0, 0);
  actor->GetZAxesGridlinesProperty()->SetColor(0, 0, 0);

  actor->DrawXGridlinesOn();
  actor->DrawYGridlinesOn();
  actor->DrawZGridlinesOn();
  actor->SetGridLineLocation(
    actor->VTK_GRID_LINES_FURTHEST);

  actor->XAxisMinorTickVisibilityOff();
  actor->YAxisMinorTickVisibilityOff();
  actor->ZAxisMinorTickVisibilityOff();

  actor->SetFlyModeToStaticEdges();
  actor->SetFlyModeToFurthestTriad();
  //actor->SetCornerOffset(0.0);
  actor->SetTitleOffset(0);
  actor->SetLabelOffset(10);

  renderer_->AddActor(actor);
  return actor;
}

void VisualizationWindow::Run() {
  render_window_->Render();
  interactor_->Start();
}

} // walnut
