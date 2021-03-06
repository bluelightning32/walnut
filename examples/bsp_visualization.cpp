#include "bsp_visualization.h"

// For std::floor and std::ceil
#include <cmath>
// For std::strcmp
#include <cstring>
#include <vtkPointData.h>
#include <vtkProperty.h>
#include <vtkProperty2D.h>

#include "walnut/concat_range.h"
#include "walnut/mesh.h"
#include "walnut/walnut_to_vtk_mesh.h"

namespace walnut {

const Point3 BSPVisualization::tilted_cube_top(0, 0, 11);
const Point3 BSPVisualization::tilted_cube_bottom(0, 0, -11);
const Point3 BSPVisualization::tilted_cube_north(0, 10, 3);
const Point3 BSPVisualization::tilted_cube_north_west(-9, 5, -4);
const Point3 BSPVisualization::tilted_cube_south_west(-9, -5, 4);
const Point3 BSPVisualization::tilted_cube_south(0, -10, -3);
const Point3 BSPVisualization::tilted_cube_south_east(9, -5, 4);
const Point3 BSPVisualization::tilted_cube_north_east(9, 5, -4);

const Point3* BSPVisualization::tilted_cube_peripheral_points[6] = {
  &tilted_cube_north,
  &tilted_cube_north_west,
  &tilted_cube_south_west,
  &tilted_cube_south,
  &tilted_cube_south_east,
  &tilted_cube_north_east,
};

BSPVisualization::BuildingContentInfo::BuildingContentInfo() {
  coincident_normals->SetName("coincident_normals");
  coincident_normals->SetNumberOfComponents(3);
  labels->SetName("labels");
}

BSPVisualization::BuildingContentInfo::BuildingContentInfo(vtkPoints* points)
    : edges(MakeEdges(points)) {
  coincident_normals->SetName("coincident_normals");
  coincident_normals->SetNumberOfComponents(3);
  labels->SetName("labels");
}

BSPVisualization::BSPVisualization(VisualizationWindow& window,
                                   const AABB& bounding_box,
                                   const AABB& labelling_box)
  : window_(window), bounding_box_(bounding_box),
    labelling_box_(labelling_box),
    crossing_label_offset_(labelling_box.GetDiagonalLength() / 200),
    full_tree_pos_(&full_tree_.root), view_pos_(&view_tree_.root),
    key_press_listener(window.AddKeyPressObserver(
          [this](const char* key) {
            return KeyPressed(key);
          })) {
  std::vector<HomoPoint3> cube_points{HomoPoint3(tilted_cube_top),
                                      HomoPoint3(tilted_cube_bottom)};
  for (const Point3* point : tilted_cube_peripheral_points) {
    cube_points.push_back(HomoPoint3(*point));
  }
  min_axes_bounds_ = ConvexVertexAABBTracker(cube_points.begin(),
                                             cube_points.end()).aabb();

  UpdateShapes();
  border_actor_ = window.AddShape(border_filter->GetOutputPort(), /*r=*/0.2,
                                  /*g=*/0.2, /*b=*/0.2, /*a=*/0.3);
  split_actor_ = window.AddShape(
      split_filter_->GetOutputPort(), /*r=*/0.1, /*g=*/0.5,
      /*b=*/0.1, /*a=*/0.4);

  border_wireframe_ = window.AddWireframe(border_filter->GetOutputPort());
  border_wireframe_->GetProperty()->SetColor(/*r=*/0.2, /*g=*/0.2,
                                             /*b=*/0.2);
  split_wireframe_ = window.AddWireframe(split_filter_->GetOutputPort());
  split_wireframe_->GetProperty()->SetColor(/*r=*/0.1, /*g=*/0.5, /*b=*/0.1);

  border_normals_ = NormalsActor(window, border_filter->GetOutputPort(),
                                 /*scale=*/3, /*start3d=*/true);
  border_normals_.SetColor(/*r=*/0.2, /*g=*/0.2, /*b=*/0.2);
  split_normals_ = NormalsActor(
      window, split_filter_->GetOutputPort(), /*scale=*/3,
      /*start3d=*/true);
  split_normals_.SetColor(/*r=*/0.1, /*g=*/0.5, /*b=*/0.1);

  split_intersect_line_filter_->SetInputDataObject(WalnutToVTKMesh(
        std::vector<MutableConvexPolygon<>>{}));
  split_intersect_line_actor_ = window.AddShape(
      split_intersect_line_filter_->GetOutputPort(), /*r=*/0.1, /*g=*/0.5,
      /*b=*/0.1, /*a=*/1.0);
  split_intersect_line_actor_->GetProperty()->SetLineWidth(7);

  labels_actor_ = window.AddPointLabels(labelled_points_data_);
}

BSPVisualization::~BSPVisualization() = default;

void BSPVisualization::AddContent(BSPContentId id, const AABB& aabb) {
  AddContent(id, aabb.GetWalls());
}

bool BSPVisualization::Up() {
  if (chosen_branches_.empty()) return false;

  chosen_branches_.pop_back();
  view_tree_.Reset();
  for (const std::pair<const BSPContentId,
                       ContentInfo>& content_pair : contents_) {
    for (const ConvexPolygon<>& polygon : content_pair.second.polygons) {
      view_tree_.AddContent(content_pair.first, polygon);
    }
  }

  full_tree_pos_ = &full_tree_.root;
  view_pos_ = &view_tree_.root;
  for (bool branch : chosen_branches_) {
    view_pos_->Split(full_tree_pos_->split());
    if (branch) {
      full_tree_pos_ = full_tree_pos_->positive_child();
      view_pos_ = view_pos_->positive_child();
    } else {
      full_tree_pos_ = full_tree_pos_->negative_child();
      view_pos_ = view_pos_->negative_child();
    }
  }

  UpdateShapes();
  return true;
}

bool BSPVisualization::Down(bool branch) {
  if (full_tree_pos_->IsLeaf()) return false;

  chosen_branches_.push_back(branch);
  view_pos_->Split(full_tree_pos_->split());
  if (branch) {
    full_tree_pos_ = full_tree_pos_->positive_child();
    view_pos_ = view_pos_->positive_child();
  } else {
    full_tree_pos_ = full_tree_pos_->negative_child();
    view_pos_ = view_pos_->negative_child();
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
    double label_offset_amount, bool pos_child, BSPContentId polygon_id,
    int type) {
  DoublePoint3 location = vertex.GetDoublePoint3();
  if (!pos_child) label_offset_amount *= -1;
  label_offset_amount /= sqrt((double)label_offset_direction.GetScaleSquared());
  location.x += (double)label_offset_direction.x() * label_offset_amount;
  location.y += (double)label_offset_direction.y() * label_offset_amount;
  location.z += (double)label_offset_direction.z() * label_offset_amount;
  labelled_points->InsertNextPoint(location.x, location.y, location.z);
  labels->InsertNextValue(
    (std::ostringstream() << (type > 0 ? "+" : "-")).str());
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
  } else if (!std::strcmp(key, "1")) {
    UseTopDownView();
    return true;
  } else if (!std::strcmp(key, "2")) {
    UseSecondView();
    return true;
  } else if (!std::strcmp(key, "4")) {
    UseFourthView();
    return true;
  } else if (!std::strcmp(key, "l")) {
    ShowLabels(!show_labels_);
    return true;
  } else if (!std::strcmp(key, "h")) {
    int next_focus = focus_content_ + 1;
    if (next_focus == static_cast<int>(contents_.size())) next_focus = -1;
    FocusContent(next_focus);
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

void BSPVisualization::AdjustNodePathToAvoidPoint(
    BSPTreeRep& tree_copy,
    std::vector<bool>& node_path,
    const HomoPoint3& avoid) const {
  std::vector<ConnectingVisitorOutputPolygon<>> border =
    tree_copy.GetNodeBorder(node_path.begin(), node_path.end(),
                            labelling_box_);
  if (border.empty()) {
    return;
  }

  HomoPoint3 center = GetCentroid(border);
  BigInt denom;
  Vector3 direction = center.Difference(avoid, denom);
  if (denom < 0) {
    direction = -direction;
    denom = -denom;
  }
  // Project the direction onto the XY plane, because we're trying to avoid
  // `avoid` in the top-down view.
  direction.z() = 0;
  HomoPoint3 farthest;
  BigInt farthest_denom;
  Vector3 farthest_dir;
  if (direction.IsZero()) {
    direction.x() = 1;
    farthest = GetFarthest(border, direction);
    farthest_dir = farthest.Difference(avoid, farthest_denom);
    if (farthest_denom < 0) {
      farthest_dir = -farthest_dir;
      farthest_denom = -farthest_denom;
    }
    denom = farthest_denom * 10;
  } else {
    farthest = GetFarthest(border, direction);
    farthest_dir = farthest.Difference(avoid, farthest_denom);
    if (farthest_denom < 0) {
      farthest_dir = -farthest_dir;
      farthest_denom = -farthest_denom;
    }
  }

  // If the difference between `avoid` and `center` is at least a fifth as far
  // as it can possibly go in that direction, then that is good enough.
  //
  // |direction/denom| > (direction/denom) * (farthest_dir/farthest_denom) / |direction/denom| / 5
  // (1/denom)^2 * |direction|^2 > direction * (farthest_dir/farthest_denom) / 5 / denom
  // (1/denom) * |direction|^2 > direction * (farthest_dir/farthest_denom) / 5
  // |direction|^2 > denom * direction * (farthest_dir/farthest_denom) / 5
  // farthest_denom * |direction|^2 > denom * direction * farthest_dir / 5
  // 5 * farthest_denom * |direction|^2 > denom * direction * farthest_dir
  if (5 * farthest_denom * direction.GetScaleSquared() >
      denom * direction.Dot(farthest_dir)) {
    return;
  }

  BSPNodeRep* copy_pos = &tree_copy.root;
  for (bool branch : node_path) {
    if (branch) {
      copy_pos = copy_pos->positive_child();
    } else {
      copy_pos = copy_pos->negative_child();
    }
  }

  HomoPoint3 new_split_coincident =
    avoid.AddOffset(farthest_dir, farthest_denom * 5);
  farthest_dir.z() = 0;
  copy_pos->Split(HalfSpace3(farthest_dir, new_split_coincident));
  node_path.push_back(true);
}

HomoPoint3 BSPVisualization::GetLabelLocation(
    const std::vector<bool>& node_path,
    const std::vector<HomoPoint3>& avoid) const {
  BSPTreeRep tree_copy;
  const BSPNodeRep* original_pos = &full_tree_.root;
  BSPNodeRep* copy_pos = &tree_copy.root;
  for (bool branch : node_path) {
    copy_pos->Split(original_pos->split());
    if (branch) {
      original_pos = original_pos->positive_child();
      copy_pos = copy_pos->positive_child();
    } else {
      original_pos = original_pos->negative_child();
      copy_pos = copy_pos->negative_child();
    }
  }

  std::vector<bool> path_copy = node_path;
  for (const HomoPoint3& avoid_point : avoid) {
    if (avoid_point.dist_denom().IsZero()) continue;

    AdjustNodePathToAvoidPoint(tree_copy, path_copy, avoid_point);
  }

  std::vector<ConnectingVisitorOutputPolygon<>> border =
    tree_copy.GetNodeBorder(path_copy.begin(), path_copy.end(),
                            labelling_box_);

  HomoPoint3 center = GetCentroid(border);
  if (center.w() == 0) {
    center = HomoPoint3(0, 0, 0, 1);
  }
  return center;
}

void BSPVisualization::AddPWNLabel(vtkStringArray* labels,
                                   const std::string& label,
                                   vtkPoints* labelled_points,
                                   const HomoPoint3& location) {
  DoublePoint3 p = location.GetDoublePoint3();

  labelled_points->InsertNextPoint(p.x, p.y, p.z);
  labels->InsertNextValue(label);
}

void BSPVisualization::UpdateShapes() {
  std::vector<bool> child_path(chosen_branches_);
  std::vector<MutableConvexPolygon<>> border =
    full_tree_.GetNodeBorderNoBoundWalls(child_path.begin(),
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

  HomoPoint3 top = GetTopPoint(border);
  DoublePoint3 top_double = top.GetDoublePoint3();
  labelled_points->InsertNextPoint(top_double.x, top_double.y, top_double.z);
  labels->InsertNextValue("Top");

  std::unordered_map<DoublePoint3, vtkIdType> point_map;
  vtkNew<vtkPoints> points;

  vtkNew<vtkPolyData> split_lines;
  split_lines->SetLines(vtkSmartPointer<vtkCellArray>::New());
  split_lines->SetPoints(points);
  split_intersect_line_filter_->SetInputDataObject(split_lines);

  if (full_tree_pos_->IsLeaf()) {
    vtkNew<vtkPolyData> empty;
    // The split actor will not be shown, but set its input anyway so that it
    // doesn't print warnings about not having any inputs.
    split_filter_->SetInputDataObject(empty);

    std::vector<HomoPoint3> avoid { top };
    HomoPoint3 label_location = GetLabelLocation(chosen_branches_, avoid);
    AddPWNLabel(labels, "Leaf PWN: " +
                GetPWNString(
                  full_tree_pos_->content_info_by_id()),
                labelled_points, label_location);
  } else {
    std::vector<bool> neg_child_path(chosen_branches_);
    neg_child_path.push_back(false);
    std::vector<bool> pos_child_path(chosen_branches_);
    pos_child_path.push_back(true);
    std::vector<MutableConvexPolygon<>> split_wall;
    std::vector<MutableConvexPolygon<>> negative_child_walls =
      full_tree_.GetNodeBorderNoBoundWalls(neg_child_path.begin(),
                                           neg_child_path.end(),
                                           bounding_box_);

    // Move the split wall out of negative_child_walls into split_wall.
    for (MutableConvexPolygon<>& wall : negative_child_walls) {
      if (wall.plane() == full_tree_pos_->split()) {
        split_wall.push_back(std::move(wall));
      }
    }
    assert(split_wall.size() == 1);
    split_filter_->SetInputDataObject(WalnutToVTKMesh(split_wall));

    HomoPoint3 new_top = GetTopPoint(split_wall);
    DoublePoint3 new_top_double = new_top.GetDoublePoint3();
    labelled_points->InsertNextPoint(new_top_double.x, new_top_double.y,
                                     new_top_double.z);
    labels->InsertNextValue("Top'");

    std::vector<HomoPoint3> avoid { top, new_top };
    HomoPoint3 neg_label_location = GetLabelLocation(neg_child_path, avoid);
    avoid.push_back(neg_label_location);
    HomoPoint3 pos_label_location = GetLabelLocation(pos_child_path, avoid);

    AddPWNLabel(labels, "Neg child PWN: " +
                GetPWNString(
                  full_tree_pos_->negative_child()->content_info_by_id()),
                labelled_points, neg_label_location);
    AddPWNLabel(labels, "Pos child PWN: " +
                GetPWNString(
                  full_tree_pos_->positive_child()->content_info_by_id()),
                labelled_points, pos_label_location);

    for (const BSPNodeRep::PolygonRep& polygon : view_pos_->contents()) {
      AddSplitOutline(polygon, point_map, points, split_lines);
    }
    for (const BSPNodeRep::PolygonRep& polygon :
         view_pos_->border_contents()) {
      AddSplitOutline(polygon, point_map, points, split_lines);
    }
  }

  std::map<BSPContentId, BuildingContentInfo> content_map;
  size_t crossing_num = 0;
  for (const BSPNodeRep::PolygonRep& polygon : view_pos_->contents()) {
    AddCoincidentEdges(polygon, point_map, points, content_map);
    AddCrossingLabels(polygon, content_map, crossing_num);
  }
  for (const BSPNodeRep::PolygonRep& polygon : view_pos_->border_contents()) {
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

  double colors[4][4] = {
    {1, 0.5, 0.2, 0.6},
    {0.2, 0.2, 1, 0.6},
    {0.6, 0.3, 0.6, 0.6},
    {0.75, 0.6, 0.0, 0.6},
  };

  size_t color_id = std::min(static_cast<size_t>(id),
                             static_cast<size_t>(3));

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
      int d = edge.edge_last_coincident.pos_side ? -1 : 1;
      double normal[3] = {
        double(edge.edge_last_coincident.split->x() * d),
        double(edge.edge_last_coincident.split->y() * d),
        double(edge.edge_last_coincident.split->z() * d)
      };
      info.coincident_normals->InsertNextTuple(normal);
    }

    if (edge.vertex_last_coincident.split != nullptr) {
      const DoublePoint3 p = edge.vertex().GetDoublePoint3();
      info.coincident_points->InsertNextPoint(p.x, p.y, p.z);
      int d = edge.vertex_last_coincident.pos_side ? -1 : 1;
      double normal[3] = {
        double(edge.vertex_last_coincident.split->x() * d),
        double(edge.vertex_last_coincident.split->y() * d),
        double(edge.vertex_last_coincident.split->z() * d)
      };
      info.coincident_normals->InsertNextTuple(normal);
    }
  }
}

void BSPVisualization::AddCrossingLabels(
    const BSPNodeRep::PolygonRep& polygon,
    std::map<BSPContentId, BuildingContentInfo>& content_map,
    size_t& crossing_num) {
  auto content_it = content_map.find(polygon.id);
  assert(content_it != content_map.end());

  BuildingContentInfo& info = content_it->second;
  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    const auto& current_edge = polygon.edge(i);

    const SplitSide& edge_last_coincident =
      current_edge.edge_last_coincident;
    if (edge_last_coincident.split == nullptr) continue;
    int edge_comparison = RXYCompareBivector(full_tree_pos_->split().normal(),
                                             edge_last_coincident);
    if (edge_comparison == 0) continue;

    std::pair<int, bool> push_info =
      full_tree_pos_->GetPWNEffectAtVertex(edge_comparison,
                                          edge_last_coincident,
                                          current_edge);
    if (push_info.first != 0) {
      ++crossing_num;
      info.AddCrossing(current_edge.vertex(),
                       full_tree_pos_->split().normal(),
                       crossing_label_offset_ * crossing_num,
                       push_info.second, polygon.id, push_info.first);
    }

    const auto& next_edge =
      polygon.const_edge((i + 1)%polygon.vertex_count());
    push_info = full_tree_pos_->GetPWNEffectAtVertex(edge_comparison,
                                                    edge_last_coincident,
                                                    next_edge);
    if (push_info.first != 0) {
      ++crossing_num;
      info.AddCrossing(next_edge.vertex(),
                       full_tree_pos_->split().normal(),
                       crossing_label_offset_ * crossing_num,
                       push_info.second, polygon.id, -push_info.first);
    }
  }
}

void BSPVisualization::AddSplitOutline(
    const BSPNodeRep::PolygonRep& polygon,
    std::unordered_map<DoublePoint3, vtkIdType> point_map,
    vtkPoints* points, vtkPolyData* split_lines) {
  assert(!full_tree_pos_->IsLeaf());

  ConvexPolygonSplitInfo split_info =
    polygon.GetSplitInfo(full_tree_pos_->split());
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

std::array<double, 6> BSPVisualization::GetContentBounds() const {
  DoublePoint3 content_min = min_axes_bounds_.min_point().GetDoublePoint3();
  DoublePoint3 content_max = min_axes_bounds_.max_point().GetDoublePoint3();

  if (ignore_contents_) {
    std::array<double, 6> bounds;
    bounds[0] = content_min.x;
    bounds[1] = content_max.x;
    bounds[2] = content_min.y;
    bounds[3] = content_max.y;
    bounds[4] = content_min.z;
    bounds[5] = content_max.z;
    return bounds;
  }

  using VertexIterator = ConvexPolygon<>::ConstVertexIterator;
  ConcatRange<VertexIterator> all_vertices;
  for (const std::pair<const BSPContentId, ContentInfo>& content : contents_) {
    std::vector<const ConvexPolygon<>*> polygons;
    for (const ConvexPolygon<>& polygon : content.second.polygons) {
      all_vertices.Append(polygon.vertices_begin(), polygon.vertices_end());
    }
  }
  AABB bounding_box =
    ConvexVertexAABBTracker(all_vertices.begin(), all_vertices.end()).aabb();

  DoublePoint3 double_min = bounding_box.min_point().GetDoublePoint3();
  DoublePoint3 double_max = bounding_box.max_point().GetDoublePoint3();

  std::array<double, 6> bounds;
  // xmin
  bounds[0] = std::floor(std::min(double_min.x, content_min.x));
  // xmax
  bounds[1] = std::ceil(std::max(double_max.x, content_max.x));
  // ymin
  bounds[2] = std::floor(std::min(double_min.y, content_min.y));
  // ymax
  bounds[3] = std::ceil(std::max(double_max.y, content_max.y));
  // zmin
  bounds[4] = std::floor(std::min(double_min.z, content_min.z));
  // zmax
  bounds[5] = std::ceil(std::max(double_max.z, content_max.z));
  return bounds;
}

void BSPVisualization::UseTopDownView() {
  if (axes_actor_) {
    window_.RemoveActor(axes_actor_);
    axes_actor_ = nullptr;
  }

  std::array<double, 6> bounds = GetContentBounds();
  axes_actor_ = window_.Axes(bounds.data(), /*padding=*/0);

  window_.UseTopDownView(bounds.data());
}

void BSPVisualization::UseSecondView() {
  if (axes_actor_) {
    window_.RemoveActor(axes_actor_);
    axes_actor_ = nullptr;
  }

  std::array<double, 6> bounds = GetContentBounds();
  axes_actor_ = window_.Axes(bounds.data(), /*padding=*/0);

  window_.UseSecondView(bounds);
}

void BSPVisualization::UseFourthView() {
  if (axes_actor_) {
    window_.RemoveActor(axes_actor_);
    axes_actor_ = nullptr;
  }

  std::array<double, 6> bounds = GetContentBounds();
  axes_actor_ = window_.Axes(bounds.data(), /*padding=*/0);

  window_.UseFourthView(bounds);
}

void BSPVisualization::ShowLabels(bool show) {
  show_labels_ = show;
  labels_actor_->SetVisibility(show_labels_);
  int i = 0;
  for (std::pair<const BSPContentId, ContentInfo>& content_pair : contents_) {
    bool show = focus_content_ == -1 || focus_content_ == i;
    content_pair.second.labels_actor->SetVisibility(show_labels_ && show);
    ++i;
  }
}

BSPNode<>* BSPVisualization::SplitToTiltedCube(BSPNode<>* start) {
  return SplitToTiltedCubeBottomPlanes(SplitToTiltedCubeTopPlanes(start));
}

BSPNode<>* BSPVisualization::SplitToTiltedCubeTopPlanes(BSPNode<>* start) {
  assert(start->IsLeaf());

  for (int i = 0; i < 6; i += 2) {
    const Point3* this_point = tilted_cube_peripheral_points[i];
    const Point3* next_point = tilted_cube_peripheral_points[(i + 2)%6];

    start->Split(HalfSpace3(*this_point, *next_point, tilted_cube_top));
    start = start->negative_child();
  }
  UpdateShapes();
  return start;
}

BSPNode<>* BSPVisualization::SplitToTiltedCubeBottomPlanes(BSPNode<>* start) {
  assert(start->IsLeaf());

  for (int i = 1; i < 6; i += 2) {
    const Point3* this_point = tilted_cube_peripheral_points[i];
    const Point3* next_point = tilted_cube_peripheral_points[(i + 2)%6];

    start->Split(HalfSpace3(*next_point, *this_point, tilted_cube_bottom));
    start = start->negative_child();
  }
  UpdateShapes();
  return start;
}

BSPNode<>* BSPVisualization::SplitNorthWest(BSPNode<>* start,
                                            const Point3& edge_start,
                                            const Point3& edge_dest,
                                            double edge_dist) {
  assert(start->IsLeaf());
  Vector3 normal = Vector3(1, -1, 10);

  auto edge_vector = edge_dest - edge_start;
  double dist = double(normal.x()) * (double(edge_start.x()) +
                                      double(edge_vector.x()) * edge_dist) +
                double(normal.y()) * (double(edge_start.y()) +
                                      double(edge_vector.y()) * edge_dist) +
                double(normal.z()) * (double(edge_start.z()) +
                                      double(edge_vector.z()) * edge_dist);
  start->Split(HalfSpace3(normal, BigInt(long(dist))));
  return start->negative_child();
}

void BSPVisualization::FocusContent(int new_focus) {
  focus_content_ = new_focus;
  int i = 0;
  for (std::pair<const BSPContentId, ContentInfo>& content_pair : contents_) {
    bool show = focus_content_ == -1 || focus_content_ == i;
    content_pair.second.labels_actor->SetVisibility(show_labels_ && show);
    content_pair.second.actor->SetVisibility(show);
    content_pair.second.line_actor->SetVisibility(show);
    content_pair.second.coincident_arrows.SetVisibility(show);
    ++i;
  }
}

} // walnut
