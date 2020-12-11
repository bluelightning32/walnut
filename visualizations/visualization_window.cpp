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
    vtkSmartPointer<vtkAlgorithmOutput> shape, double scale) {
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
  glyph->SetScaleFactor(scale);
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

void VisualizationWindow::UseTopDownView() {
  renderer_->GetActiveCamera()->ParallelProjectionOn();
  renderer_->GetActiveCamera()->SetFocalPoint(0, 0, 0);
  renderer_->GetActiveCamera()->SetPosition(0, 0, 10);
  renderer_->GetActiveCamera()->SetViewUp(0, 1, 0);
  renderer_->ResetCamera();
  Zoom(1.2);
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
  Zoom(1.5);
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
  Zoom(1.5);
  return wireframe_actor;
}

vtkSmartPointer<vtkActor> VisualizationWindow::AddShapeNormals(
    vtkSmartPointer<vtkAlgorithmOutput> shape, double scale) {
  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(GetNormalsGlyph(shape, scale)->GetOutputPort());

  auto actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(0,0.5,1);

  renderer_->AddActor(actor);
  renderer_->ResetCamera();
  Zoom(1.5);
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
  actor->SetScreenSize(15);
  actor->GetTitleTextProperty(0)->SetColor(0, 0, 0);
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
  renderer_->ResetCamera();
  Zoom(1.5);
  return actor;
}

vtkSmartPointer<vtkSliderWidget> VisualizationWindow::CreateSliderWidget() {
  auto slider = vtkSmartPointer<vtkSliderWidget>::New();
  slider->SetInteractor(interactor_);
  slider->SetAnimationModeToAnimate();
  slider->EnabledOn();
  return slider;
}

void VisualizationWindow::Zoom(double factor) {
  renderer_->GetActiveCamera()->Zoom(factor);
}

void VisualizationWindow::Run() {
  render_window_->Render();
  interactor_->Start();
}

} // walnut
