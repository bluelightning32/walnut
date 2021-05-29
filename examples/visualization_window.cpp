#include "visualization_window.h"

#include <vtkArrowSource.h>
#include <vtkApplyColors.h>
#include <vtkCamera.h>
#include <vtkCellCenters.h>
#include <vtkGlyph2D.h>
#include <vtkGlyph3D.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkProperty.h>
#include <vtkProperty2D.h>
#include <vtkTextProperty.h>

#include "function_command.h"

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

// Apply a Color to a shape.
vtkSmartPointer<vtkAlgorithm> Color(vtkSmartPointer<vtkPolyData> input,
                                    double r, double g, double b, double a) {
  auto filter = vtkSmartPointer<vtkApplyColors>::New();
  filter->SetInputData(0, input);
  filter->SetDefaultCellColor(r, g, b);
  filter->SetDefaultCellOpacity(a);
  return filter;
}

vtkSmartPointer<vtkGlyph3D> GetNormalsGlyph(
    vtkSmartPointer<vtkAlgorithmOutput> shape, double scale, bool normals3d) {
  auto normals = vtkSmartPointer<vtkPolyDataNormals>::New();
  normals->SetInputConnection(shape);

  normals->SplittingOff();
  normals->ConsistencyOff();
  normals->ComputePointNormalsOff();
  normals->ComputeCellNormalsOn();

  auto centers = vtkSmartPointer<vtkCellCenters>::New();
  centers->SetInputConnection(normals->GetOutputPort());

  auto arrow = vtkSmartPointer<vtkArrowSource>::New();
  arrow->Update();

  vtkSmartPointer<vtkGlyph3D> glyph;
  if (normals3d) {
    glyph = vtkSmartPointer<vtkGlyph3D>::New();
  } else {
    glyph = vtkSmartPointer<vtkGlyph2D>::New();
  }

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
    vtkSmartPointer<vtkPolyData> shape, double r, double g, double b,
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
    vtkSmartPointer<vtkAlgorithmOutput> shape, double scale, bool normals3d) {
  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(
      GetNormalsGlyph(shape, scale, normals3d)->GetOutputPort());

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

std::pair<vtkSmartPointer<vtkSliderWidget>,
          vtkSmartPointer<vtkSliderRepresentation2D>>
VisualizationWindow::Create2DSliderWidget() {
  auto slider_rep = vtkSmartPointer<vtkSliderRepresentation2D>::New();
  slider_rep->GetTubeProperty()->SetColor(.8, .8, .8);
  slider_rep->GetSliderProperty()->SetColor(.5, .5, .5);
  slider_rep->GetLabelProperty()->SetColor(0, 0, 0);
  slider_rep->GetCapProperty()->SetColor(.5 , .5, .5);
  slider_rep->SetEndCapWidth(0.025);
  slider_rep->SetSliderLength(0.04);
  slider_rep->GetPoint1Coordinate()->SetCoordinateSystemToDisplay();
  slider_rep->GetPoint2Coordinate()->SetCoordinateSystemToDisplay();

  auto slider = vtkSmartPointer<vtkSliderWidget>::New();
  slider->SetRepresentation(slider_rep);
  slider->SetInteractor(interactor_);
  slider->SetAnimationModeToAnimate();
  slider->EnabledOn();

  return std::make_pair(slider, slider_rep);
}

void VisualizationWindow::Zoom(double factor) {
  renderer_->GetActiveCamera()->Zoom(factor);
}

void VisualizationWindow::Run() {
  render_window_->Render();
  interactor_->Start();
}

ObserverRegistration VisualizationWindow::AddKeyPressObserver(
    std::function<bool(char)> observer) {
  vtkSmartPointer<walnut::FunctionCommand> command =
    walnut::MakeFunctionCommand([interactor = interactor_, observer](
          vtkObject* caller, unsigned long event_id, void* data) {
        if (observer(interactor->GetKeyCode())) {
          interactor->Render();
        }
      });

  return ObserverRegistration(
      interactor_,
      interactor_->AddObserver(vtkCommand::KeyPressEvent, command));
}

void VisualizationWindow::Redraw() {
  render_window_->Render();
}

} // walnut
