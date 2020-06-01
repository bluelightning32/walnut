#include <iostream>

#include <vtkActor.h>
#include <vtkArrowSource.h>
#include <vtkApplyColors.h>
#include <vtkCamera.h>
#include <vtkCellCenters.h>
#include <vtkCubeAxesActor.h>
#include <vtkGlyph3D.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkNamedColors.h>
#include <vtkPNGWriter.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSphereSource.h>
#include <vtkSTLWriter.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkWindowToImageFilter.h>

vtkSmartPointer<vtkAlgorithm> Color(vtkSmartPointer<vtkAlgorithm> input,
                                    double r, double g, double b, double a) {
  auto filter = vtkSmartPointer<vtkApplyColors>::New();
  filter->SetInputConnection(0, input->GetOutputPort());
  filter->SetDefaultCellColor(r, g, b);
  filter->SetDefaultCellOpacity(a);
  return filter;
}

vtkSmartPointer<vtkCubeAxesActor> GetAxisActor(double content_bounds[6], double padding, vtkCamera* camera) {
  vtkSmartPointer<vtkNamedColors> colors =
    vtkSmartPointer<vtkNamedColors>::New();
  vtkColor3d axis1Color = colors->GetColor3d("Black");
  vtkColor3d axis2Color = colors->GetColor3d("Black");
  vtkColor3d axis3Color = colors->GetColor3d("Black");

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
  actor->GetTitleTextProperty(0)->SetColor(axis1Color.GetData());
  actor->GetTitleTextProperty(0)->SetFontSize(48);
  actor->GetLabelTextProperty(0)->SetColor(axis1Color.GetData());

  actor->GetTitleTextProperty(1)->SetColor(axis2Color.GetData());
  actor->GetLabelTextProperty(1)->SetColor(axis2Color.GetData());

  actor->GetTitleTextProperty(2)->SetColor(axis3Color.GetData());
  actor->GetLabelTextProperty(2)->SetColor(axis3Color.GetData());

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
  return actor;
}

vtkSmartPointer<vtkGlyph3D> GetNormalsGlyph(vtkSmartPointer<vtkAlgorithm> shape) {
  auto normals = vtkSmartPointer<vtkPolyDataNormals>::New();
  normals->SetInputConnection(shape->GetOutputPort());

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

vtkSmartPointer<vtkActor> GetNormalsActor(vtkSmartPointer<vtkAlgorithm> shape) {
  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(GetNormalsGlyph(shape)->GetOutputPort());

  auto actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(0,0.5,1);

  return actor;
}

inline void Render(vtkSmartPointer<vtkAlgorithm> shape) {
  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(Color(shape, 1, 0.8, 0.8, 0.5)->GetOutputPort());
  mapper->SetScalarModeToUseCellFieldData();
  mapper->SelectColorArray("vtkApplyColors color");
  mapper->SetScalarVisibility(true);

  auto actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  auto wireframe_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  wireframe_mapper->SetInputConnection(shape->GetOutputPort());

  auto wireframe_actor = vtkSmartPointer<vtkActor>::New();
  wireframe_actor->SetMapper(wireframe_mapper);
  wireframe_actor->GetProperty()->SetColor(0, 0, 0);
  wireframe_actor->GetProperty()->SetRepresentationToWireframe();
  wireframe_actor->GetProperty()->SetLineWidth(3);

  auto renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(actor);
  renderer->AddActor(wireframe_actor);
  renderer->AddActor(GetNormalsActor(shape));
  renderer->ResetCamera();
  renderer->AddActor(GetAxisActor(actor->GetBounds(), 5, renderer->GetActiveCamera()));
  renderer->ResetCamera();
  renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);
  renderer->GetActiveCamera()->SetPosition(10, 10, 10);
  renderer->GetActiveCamera()->SetViewUp(0, 0, 1);
  renderer->ResetCamera();

  vtkSmartPointer<vtkNamedColors> colors =
    vtkSmartPointer<vtkNamedColors>::New();
  vtkColor3d backgroundColor = colors->GetColor3d("White");
  renderer->SetBackground(backgroundColor.GetData());

  auto render_window = vtkSmartPointer<vtkRenderWindow>::New();
  render_window->AddRenderer(renderer);
  render_window->SetSize(640, 480);

  auto interator = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interator->SetRenderWindow(render_window);

  auto interactor_style = vtkSmartPointer<vtkInteractorStyleSwitch>::New();
  interactor_style->SetCurrentStyleToTrackballCamera();
  interator->SetInteractorStyle(interactor_style);

  render_window->Render();

  interator->Start();
}
