#include "bsp_visualization.h"

// For std::strcmp
#include <cstring>
#include <vtkPointData.h>
#include <vtkProperty.h>
#include <vtkProperty2D.h>

#include "walnut/mesh.h"
#include "walnut/walnut_to_vtk_mesh.h"

namespace walnut {

BSPVisualization::BSPVisualization(VisualizationWindow& window,
                                   const AABB& bounding_box,
                                   const AABB& labelling_box,
                                   const BSPTreeRep& tree)
  : window_(window), bounding_box_(bounding_box),
    labelling_box_(labelling_box),
    crossing_label_offset_(labelling_box.GetDiagonalLength() / 200),
    original_tree_(tree),
    original_pos_(&tree.root), pos_(&tree_.root),
    key_press_listener(window.AddKeyPressObserver(
          [this](const char* key) {
            return KeyPressed(key);
          })) {
  UpdateShapes();
  border_actor_ = window.AddShape(border_filter->GetOutputPort(), /*r=*/1.0,
                                  /*g=*/1.0, /*b=*/0.0, /*a=*/0.2);
  split_actor_ = window.AddShape(
      split_filter_->GetOutputPort(), /*r=*/0.5, /*g=*/1.0,
      /*b=*/0.0, /*a=*/0.4);

  border_wireframe_ = window.AddWireframe(border_filter->GetOutputPort());
  border_wireframe_->GetProperty()->SetColor(/*r=*/1.0, /*g=*/1.0,
                                             /*b=*/0.0);
  split_wireframe_ = window.AddWireframe(split_filter_->GetOutputPort());
  split_wireframe_->GetProperty()->SetColor(/*r=*/0.8, /*g=*/1.0, /*b=*/0.0);

  border_normals_ = NormalsActor(window, border_filter->GetOutputPort(),
                                 /*scale=*/3, /*start3d=*/true);
  border_normals_.SetColor(/*r=*/1.0, /*g=*/1.0, /*b=*/0.0);
  split_normals_ = NormalsActor(
      window, split_filter_->GetOutputPort(), /*scale=*/3,
      /*start3d=*/true);
  split_normals_.SetColor(/*r=*/0.8, /*g=*/1.0, /*b=*/0.0);

  split_intersect_line_filter_->SetInputDataObject(WalnutToVTKMesh(
        std::vector<MutableConvexPolygon<>>{}));
  split_intersect_line_actor_ = window.AddShape(
      split_intersect_line_filter_->GetOutputPort(), /*r=*/0.8, /*g=*/1.0,
      /*b=*/0.0, /*a=*/1.0);
  split_intersect_line_actor_->GetProperty()->SetLineWidth(7);

  labels_actor_ = window.AddPointLabels(labelled_points_data_);
}

bool BSPVisualization::Up() {
  if (chosen_branches_.empty()) return false;

  chosen_branches_.pop_back();
  tree_.Reset();
  for (const std::pair<const BSPContentId,
                       ContentInfo>& content_pair : contents_) {
    for (const ConvexPolygon<>* polygon : content_pair.second.polygons) {
      tree_.AddContent(content_pair.first, *polygon);
    }
  }

  original_pos_ = &original_tree_.root;
  pos_ = &tree_.root;
  for (bool branch : chosen_branches_) {
    pos_->Split(original_pos_->split());
    if (branch) {
      original_pos_ = original_pos_->positive_child();
      pos_ = pos_->positive_child();
    } else {
      original_pos_ = original_pos_->negative_child();
      pos_ = pos_->negative_child();
    }
  }

  UpdateShapes();
  return true;
}

bool BSPVisualization::Down(bool branch) {
  if (original_pos_->IsLeaf()) return false;

  chosen_branches_.push_back(branch);
  pos_->Split(original_pos_->split());
  if (branch) {
    original_pos_ = original_pos_->positive_child();
    pos_ = pos_->positive_child();
  } else {
    original_pos_ = original_pos_->negative_child();
    pos_ = pos_->negative_child();
  }
  UpdateShapes();
  return true;
}

BSPVisualization::ContentInfo::ContentInfo(VisualizationWindow& window,
                                           bool start3d, double r, double g,
                                           double b, double a)
    : coincident_arrows(window, coincident_data, /*scale=*/3, start3d) {
  filter->SetInputDataObject(WalnutToVTKMesh(
        std::vector<MutableConvexPolygon<>>{}));
  actor = window.AddShape(filter->GetOutputPort(), r, g, b, a);

  line_filter->SetInputDataObject(WalnutToVTKMesh(
        std::vector<MutableConvexPolygon<>>{}));
  line_actor = window.AddShape(line_filter->GetOutputPort(),
                               r / 2, g / 2, b / 2, 1.0);
  line_actor->GetProperty()->SetLineWidth(7);

  vtkNew<vtkPoints> coincident_points;
  vtkNew<vtkDoubleArray> coincident_normals;
  coincident_normals->SetName("coincident_normals");
  coincident_normals->SetNumberOfComponents(3);

  coincident_data->SetPoints(coincident_points);
  coincident_data->GetPointData()->SetVectors(coincident_normals);

  coincident_arrows.SetColor(r / 2, g / 2, b / 2);

  labels_actor = window.AddPointLabels(labelled_points_data);
  labels_actor->GetProperty()->SetColor(r / 2, g / 2, b / 2);
}

void BSPVisualization::BuildingContentInfo::AddCrossing(
    const HomoPoint3& vertex, const Vector3& label_offset_direction,
    double label_offset_amount, bool pos_child, int type) {
  DoublePoint3 location = vertex.GetDoublePoint3();
  if (!pos_child) label_offset_amount *= -1;
  label_offset_amount /= sqrt((double)label_offset_direction.GetScaleSquared());
  location.x += (double)label_offset_direction.x() * label_offset_amount;
  location.y += (double)label_offset_direction.y() * label_offset_amount;
  location.z += (double)label_offset_direction.z() * label_offset_amount;
  labelled_points->InsertNextPoint(location.x, location.y, location.z);
  labels->InsertNextValue(type > 0 ? "+" : "-");
}

vtkNew<vtkPolyData> BSPVisualization::BuildingContentInfo::MakeEdges(
    vtkPoints* points) {
  vtkNew<vtkPolyData> line_poly_data;
  auto lines = vtkSmartPointer<vtkCellArray>::New();
  line_poly_data->SetPoints(points);
  line_poly_data->SetLines(lines);
  return line_poly_data;
}

bool BSPVisualization::KeyPressed(const char* key) {
  if (!std::strcmp(key, "Up")) {
    Up();
    return true;
  } else if (!std::strcmp(key, "Left")) {
    Down(/*branch=*/false);
    return true;
  } else if (!std::strcmp(key, "Right")) {
    Down(/*branch=*/true);
    return true;
  }
  return false;
}

std::string BSPVisualization::GetPWNString(
    const std::vector<BSPContentInfo>& content_info_by_id) {
  std::ostringstream out;
  out << "(";
  bool first = true;
  for (const BSPContentInfo& content : content_info_by_id) {
    if (!first) {
      out << ", ";
    }
    first = false;
    out << content.pwn;
  }
  out << ")";
  return out.str();
}

void BSPVisualization::UpdateShapes() {
  std::vector<bool> child_path(chosen_branches_);
  std::vector<MutableConvexPolygon<>> border =
    original_tree_.GetNodeBorderNoBoundWalls(child_path.begin(),
                                             child_path.end(),
                                             bounding_box_);
  vtkSmartPointer<vtkPolyData> vtk_border = WalnutToVTKMesh(border);
  border_filter->SetInputDataObject(vtk_border);

  vtkNew<vtkPoints> labelled_points;
  vtkNew<vtkStringArray> labels;
  labels->SetName("labels");
  labelled_points_data_->SetPoints(labelled_points);
  labelled_points_data_->GetPointData()->RemoveArray("labels");
  labelled_points_data_->GetPointData()->AddArray(labels);

  DoublePoint3 top = GetTopPoint(border).GetDoublePoint3();
  labelled_points->InsertNextPoint(top.x, top.y, top.z);
  labels->InsertNextValue("Top");

  std::unordered_map<DoublePoint3, vtkIdType> point_map;
  vtkNew<vtkPoints> points;

  vtkNew<vtkPolyData> split_lines;
  split_lines->SetLines(vtkSmartPointer<vtkCellArray>::New());
  split_lines->SetPoints(points);
  split_intersect_line_filter_->SetInputDataObject(split_lines);

  if (original_pos_->IsLeaf()) {
    vtkNew<vtkPolyData> empty;
    // The split actor will not be shown, but set its input anyway so that it
    // doesn't print warnings about not having any inputs.
    split_filter_->SetInputDataObject(empty);
  } else {
    std::vector<bool> neg_child_path(chosen_branches_);
    neg_child_path.push_back(false);
    std::vector<bool> pos_child_path(chosen_branches_);
    pos_child_path.push_back(true);
    std::vector<MutableConvexPolygon<>> split_wall;
    std::vector<MutableConvexPolygon<>> negative_child_walls =
      original_tree_.GetNodeBorderNoBoundWalls(neg_child_path.begin(),
                                               neg_child_path.end(),
                                               bounding_box_);

    // Move the split wall out of negative_child_walls into split_wall.
    for (MutableConvexPolygon<>& wall : negative_child_walls) {
      if (wall.plane() == original_pos_->split()) {
        split_wall.push_back(std::move(wall));
      }
    }
    assert(split_wall.size() == 1);
    split_filter_->SetInputDataObject(WalnutToVTKMesh(split_wall));

    DoublePoint3 new_top = GetTopPoint(split_wall).GetDoublePoint3();
    labelled_points->InsertNextPoint(new_top.x, new_top.y, new_top.z);
    labels->InsertNextValue("Top'");

    HomoPoint3 neg_center =
      GetCentroid(original_tree_.GetNodeBorder(neg_child_path.begin(),
                                               neg_child_path.end(),
                                               labelling_box_));
    {
      DoublePoint3 p = neg_center.GetDoublePoint3();
      labelled_points->InsertNextPoint(p.x, p.y, p.z);
      labels->InsertNextValue("Neg child PWN: " +
          GetPWNString(
            original_pos_->negative_child()->content_info_by_id()));
    }
    HomoPoint3 pos_center =
      GetCentroid(original_tree_.GetNodeBorder(pos_child_path.begin(),
                                               pos_child_path.end(),
                                               labelling_box_));
    {
      DoublePoint3 p = pos_center.GetDoublePoint3();
      labelled_points->InsertNextPoint(p.x, p.y, p.z);
      labels->InsertNextValue("Pos child PWN: " +
          GetPWNString(
            original_pos_->positive_child()->content_info_by_id()));
    }

    for (const BSPNodeRep::PolygonRep& polygon : pos_->contents()) {
      AddSplitOutline(polygon, point_map, points, split_lines);
    }
    for (const BSPNodeRep::PolygonRep& polygon : pos_->border_contents()) {
      AddSplitOutline(polygon, point_map, points, split_lines);
    }
  }

  std::map<BSPContentId, BuildingContentInfo> content_map;
  for (const BSPNodeRep::PolygonRep& polygon : pos_->contents()) {
    AddCoincidentEdges(polygon, point_map, points, content_map);
    AddCrossingLabels(polygon, content_map);
  }
  for (const BSPNodeRep::PolygonRep& polygon : pos_->border_contents()) {
    AddCoincidentEdges(polygon, point_map, points, content_map);
  }
  for (std::pair<const BSPContentId,
                 ContentInfo>& content_pair : contents_) {
    const BuildingContentInfo& info = content_map[content_pair.first];
    content_pair.second.filter->SetInputDataObject(WalnutToVTKMesh(
          info.facets));
    content_pair.second.line_filter->SetInputDataObject(info.edges);

    content_pair.second.coincident_data->SetPoints(info.coincident_points);
    content_pair.second.coincident_data->GetPointData()->SetVectors(
        info.coincident_normals);

    content_pair.second.labelled_points_data->SetPoints(
        info.labelled_points);
    content_pair.second.labelled_points_data->GetPointData()->RemoveArray(
        "labels");
    content_pair.second.labelled_points_data->GetPointData()->AddArray(
        info.labels);
  }
}

BSPVisualization::ContentInfo& BSPVisualization::GetContentInfo(
    BSPContentId id) {
  auto it = contents_.find(id);
  if (it != contents_.end()) return it->second;

  double colors[2][4] = {
    {1, 0.8, 0.8, 0.6},
    {0.8, 1, 0.8, 0.6},
  };

  size_t color_id = std::min(static_cast<size_t>(id),
                             static_cast<size_t>(2));

  auto inserted = contents_.emplace(
      id,
      ContentInfo(window_, /*start3d=*/border_normals_.use_3d,
                  /*r=*/colors[color_id][0], /*g=*/colors[color_id][1],
                  /*b=*/colors[color_id][2], /*a=*/colors[color_id][3]));
  assert(inserted.second);
  return inserted.first->second;
}

vtkIdType BSPVisualization::MapPoint(
    const DoublePoint3& point,
    std::unordered_map<DoublePoint3, vtkIdType> point_map, vtkPoints* points) {
  auto found = point_map.find(point);
  if (found != point_map.end()) {
    return found->second;
  } else {
    vtkIdType point_id = points->InsertNextPoint(point.x, point.y, point.z);
    point_map.emplace(point, point_id);
    return point_id;
  }
}

void BSPVisualization::AddCoincidentEdges(
    const BSPNodeRep::PolygonRep& polygon,
    std::unordered_map<DoublePoint3, vtkIdType> point_map,
    vtkPoints* points,
    std::map<BSPContentId, BuildingContentInfo>& content_map) {
  auto content_it = content_map.find(polygon.id);
  if (content_it == content_map.end()) {
    auto inserted = content_map.emplace(polygon.id,
                                        BuildingContentInfo(points));
    assert(inserted.second);
    content_it = inserted.first;
  }

  BuildingContentInfo& info = content_it->second;
  info.facets.emplace_back(polygon);
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    const auto& edge = polygon.edge(i);
    if (edge.edge_last_coincident.split != nullptr) {
      const DoublePoint3 start = edge.vertex().GetDoublePoint3();
      const DoublePoint3 end =
        polygon.vertex((i + 1) % polygon.vertex_count()).GetDoublePoint3();
      vtkIdType endpoints[2] = {
        MapPoint(start, point_map, points),
        MapPoint(end, point_map, points)
      };
      info.edges->GetLines()->InsertNextCell(2, endpoints);

      info.coincident_points->InsertNextPoint((start.x + end.x) / 2,
                                              (start.y + end.y) / 2,
                                              (start.z + end.z) / 2);
      long double d = (long double)edge.edge_last_coincident.split->d() *
        (edge.edge_last_coincident.pos_side ? -1 : 1);
      double normal[3] = {
        double((long double)edge.edge_last_coincident.split->x() / d),
        double((long double)edge.edge_last_coincident.split->y() / d),
        double((long double)edge.edge_last_coincident.split->z() / d)
      };
      info.coincident_normals->InsertNextTuple(normal);
    }

    if (edge.vertex_last_coincident.split != nullptr) {
      const DoublePoint3 p = edge.vertex().GetDoublePoint3();
      info.coincident_points->InsertNextPoint(p.x, p.y, p.z);
      long double d = (long double)edge.vertex_last_coincident.split->d() *
        (edge.vertex_last_coincident.pos_side ? -1 : 1);
      double normal[3] = {
        double((long double)edge.vertex_last_coincident.split->x() / d),
        double((long double)edge.vertex_last_coincident.split->y() / d),
        double((long double)edge.vertex_last_coincident.split->z() / d)
      };
      info.coincident_normals->InsertNextTuple(normal);
    }
  }
}

void BSPVisualization::AddCrossingLabels(
    const BSPNodeRep::PolygonRep& polygon,
    std::map<BSPContentId, BuildingContentInfo>& content_map) {
  auto content_it = content_map.find(polygon.id);
  assert(content_it != content_map.end());

  BuildingContentInfo& info = content_it->second;
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    const auto& current_edge = polygon.edge(i);

    const SplitSide& edge_last_coincident =
      current_edge.edge_last_coincident;
    if (edge_last_coincident.split == nullptr) continue;
    int edge_comparison = RXYCompareBivector(original_pos_->split().normal(),
                                             edge_last_coincident);
    if (edge_comparison == 0) continue;

    std::pair<int, bool> push_info =
      original_pos_->GetPWNEffectAtVertex(edge_comparison,
                                          edge_last_coincident,
                                          current_edge);
    if (push_info.first != 0) {
      info.AddCrossing(current_edge.vertex(),
                       original_pos_->split().normal(),
                       crossing_label_offset_, push_info.second,
                       push_info.first);
    }

    const auto& next_edge =
      polygon.const_edge((i + 1)%polygon.vertex_count());
    push_info = original_pos_->GetPWNEffectAtVertex(edge_comparison,
                                                    edge_last_coincident,
                                                    next_edge);
    if (push_info.first != 0) {
      info.AddCrossing(next_edge.vertex(),
                       original_pos_->split().normal(),
                       crossing_label_offset_, push_info.second,
                       -push_info.first);
    }
  }
}

void BSPVisualization::AddSplitOutline(
    const BSPNodeRep::PolygonRep& polygon,
    std::unordered_map<DoublePoint3, vtkIdType> point_map,
    vtkPoints* points, vtkPolyData* split_lines) {
  assert(!original_pos_->IsLeaf());

  ConvexPolygonSplitInfo split_info =
    polygon.GetSplitInfo(original_pos_->split());
  if (split_info.ShouldEmitNegativeChild() &&
      split_info.ShouldEmitPositiveChild()) {
    const HomoPoint3& start = split_info.has_new_shared_point1 ?
      split_info.new_shared_point1 :
      polygon.vertex(split_info.pos_range().second % polygon.vertex_count());
    const HomoPoint3& end = split_info.has_new_shared_point2 ?
      split_info.new_shared_point2 :
      polygon.vertex(split_info.neg_range().second % polygon.vertex_count());

    vtkIdType endpoints[2] = {
      MapPoint(start.GetDoublePoint3(), point_map, points),
      MapPoint(end.GetDoublePoint3(), point_map, points)
    };
    split_lines->GetLines()->InsertNextCell(2, endpoints);
  }
}

} // walnut
