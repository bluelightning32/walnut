#ifndef WALNUT_EXAMPLES_BSP_VISUALIZATION_H__
#define WALNUT_EXAMPLES_BSP_VISUALIZATION_H__

#include <cstring>
#include <vtkDoubleArray.h>
#include <vtkPassThroughFilter.h>
#include <vtkPointData.h>
#include <vtkProperty.h>

#include "normals_actor.h"
#include "visualization_window.h"
#include "walnut/bsp_tree.h"
#include "walnut/walnut_to_vtk_mesh.h"

namespace walnut {

class BSPVisualization {
 public:
  using BSPTreeRep = BSPTree<>;
  using BSPNodeRep = BSPTreeRep::BSPNodeRep;

  // Constructs the visualization for the partitions chosen in `tree`.
  //
  // `tree` and `window` must remain valid during the lifetime of the
  // BSPVisualization.
  BSPVisualization(VisualizationWindow& window, const AABB& bounding_box,
                   const BSPTreeRep& tree)
    : window_(window), bounding_box_(bounding_box), original_tree_(tree),
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

    UpdateActorVisibility();
  }

  // Adds content polygons to the visualization.
  //
  // Content polygons should be added directly to the original tree, then they
  // also need to be added to the visualization in order for them to show up in
  // the visualization.
  template <typename InputConvexPolygon>
  void AddContent(BSPContentId id,
                  const std::vector<InputConvexPolygon>& polygons) {
    std::vector<const ConvexPolygon<>*>& contents_vector =
      GetContentInfo(id).polygons;
    for (const InputConvexPolygon& polygon : polygons) {
      contents_vector.push_back(&polygon);
      tree_.AddContent(id, polygon);
    }
    UpdateShapes();
  }

  // Adds a content polygons to the visualization.
  //
  // Content polygons should be added directly to the original tree, then they
  // also need to be added to the visualization in order for them to show up in
  // the visualization.
  template <typename InputConvexPolygon>
  void AddContent(BSPContentId id, const InputConvexPolygon& polygon) {
    GetContentInfo(id).polygons.push_back(&polygon);
    tree_.AddContent(id, polygon);
    UpdateShapes();
  }

  // Moves the current position back up the tree.
  //
  // Returns true if the move was performed, or false if the position was
  // already at the root of the tree and thus no move ws performed.
  bool Up() {
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
    UpdateActorVisibility();
    return true;
  }

  // Moves the current position to the specified child.
  //
  // If `branch` is true, a move to the positive child is attempted, otherwise
  // a move to the negative child is attempted.
  //
  // Returns true if the move was performed, or false if the current position
  // is a leaf (has no children).
  bool Down(bool branch) {
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
    UpdateActorVisibility();
    return true;
  }

 private:
  struct ContentInfo {
    ContentInfo(VisualizationWindow& window, double r, double g, double b,
                double a) {
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

      coincident_arrows = window.AddPointArrows(coincident_data);
      coincident_arrows->GetProperty()->SetColor(r / 2, g / 2, b / 2);
    }

    std::vector<const ConvexPolygon<>*> polygons;

    vtkNew<vtkPassThroughFilter> filter;
    vtkSmartPointer<vtkActor> actor;
    vtkNew<vtkPassThroughFilter> line_filter;
    vtkSmartPointer<vtkActor> line_actor;

    // Arrows for the normals of `vertex_last_coincident` and
    // `edge_last_coincident` of the polygons inside the tree.
    vtkNew<vtkPolyData> coincident_data;
    vtkSmartPointer<vtkActor> coincident_arrows;
  };

  struct BuildingContentInfo {
    BuildingContentInfo() = default;

    BuildingContentInfo(vtkPoints* points) : edges(MakeEdges(points)) {
      coincident_normals->SetName("coincident_normals");
      coincident_normals->SetNumberOfComponents(3);
    }

    std::vector<MutableConvexPolygon<>> facets;
    vtkNew<vtkPolyData> edges;
    vtkNew<vtkPoints> coincident_points;
    vtkNew<vtkDoubleArray> coincident_normals;

   private:
    vtkNew<vtkPolyData> MakeEdges(vtkPoints* points) {
      vtkNew<vtkPolyData> line_poly_data;
      auto lines = vtkSmartPointer<vtkCellArray>::New();
      line_poly_data->SetPoints(points);
      line_poly_data->SetLines(lines);
      return line_poly_data;
    }
  };

  bool KeyPressed(const char* key) {
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

  void UpdateShapes() {
    std::vector<bool> child_path(chosen_branches_);
    vtkSmartPointer<vtkPolyData> border =
      WalnutToVTKMesh(
          original_tree_.GetNodeBorderNoBoundWalls(child_path.begin(),
                                                   child_path.end(),
                                                   bounding_box_));
    border_filter->SetInputDataObject(border);

    std::unordered_map<DoublePoint3, vtkIdType> point_map;
    vtkNew<vtkPoints> points;

    vtkNew<vtkPolyData> split_lines;
    split_lines->SetLines(vtkSmartPointer<vtkCellArray>::New());
    split_lines->SetPoints(points);

    if (original_pos_->IsLeaf()) {
      // The split actor will not be shown, but set its input to the border
      // anyway so that it doesn't print warnings about not having any inputs.
      split_filter_->SetInputDataObject(border);
    } else {
      std::vector<bool> neg_child_path(chosen_branches_);
      neg_child_path.push_back(false);
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

      for (const BSPNodeRep::PolygonRep& polygon : pos_->contents()) {
        AddSplitOutline(polygon, point_map, points, split_lines);
      }
      for (const BSPNodeRep::PolygonRep& polygon : pos_->border_contents()) {
        AddSplitOutline(polygon, point_map, points, split_lines);
      }
      split_intersect_line_filter_->SetInputDataObject(split_lines);
    }

    std::map<BSPContentId, BuildingContentInfo> content_map;
    for (const BSPNodeRep::PolygonRep& polygon : pos_->contents()) {
      AddCoincidentEdges(polygon, point_map, points, content_map);
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
    }
  }

  void UpdateActorVisibility() {
    split_actor_->SetVisibility(!original_pos_->IsLeaf());
    split_wireframe_->SetVisibility(!original_pos_->IsLeaf());
    split_normals_.SetVisibility(!original_pos_->IsLeaf());
    split_intersect_line_actor_->SetVisibility(!original_pos_->IsLeaf());
  }

  ContentInfo& GetContentInfo(BSPContentId id) {
    auto it = contents_.find(id);
    if (it != contents_.end()) return it->second;

    double colors[2][4] = {
      {1, 0.8, 0.8, 0.6},
      {0.8, 1, 0.8, 0.6},
    };

    size_t color_id = std::min(static_cast<size_t>(id),
                               static_cast<size_t>(2));

    auto inserted = contents_.emplace(id,
                                      ContentInfo(window_,
                                                  /*r=*/colors[color_id][0],
                                                  /*g=*/colors[color_id][1],
                                                  /*b=*/colors[color_id][2],
                                                  /*a=*/colors[color_id][3]));
    assert(inserted.second);
    return inserted.first->second;
  }

  vtkIdType MapPoint(const DoublePoint3& point,
                     std::unordered_map<DoublePoint3, vtkIdType> point_map,
                     vtkPoints* points) {
    auto found = point_map.find(point);
    if (found != point_map.end()) {
      return found->second;
    } else {
      vtkIdType point_id = points->InsertNextPoint(point.x, point.y, point.z);
      point_map.emplace(point, point_id);
      return point_id;
    }
  }

  void AddCoincidentEdges(
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

  void AddSplitOutline(const BSPNodeRep::PolygonRep& polygon,
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

  VisualizationWindow& window_;
  AABB bounding_box_;

  const BSPTreeRep& original_tree_;
  const BSPNodeRep* original_pos_;
  // false means the negative child was chosen, and true means the positive
  // child was chosen.
  std::vector<bool> chosen_branches_;
  BSPTreeRep tree_;
  BSPNodeRep* pos_;

  std::map<BSPContentId, ContentInfo> contents_;

  vtkNew<vtkPassThroughFilter> border_filter;
  vtkNew<vtkPassThroughFilter> split_filter_;
  vtkSmartPointer<vtkActor> border_actor_;
  vtkSmartPointer<vtkActor> split_actor_;

  vtkSmartPointer<vtkActor> border_wireframe_;
  vtkSmartPointer<vtkActor> split_wireframe_;

  // Outline of where the split plane intersects with the content of the
  // current node.
  vtkNew<vtkPassThroughFilter> split_intersect_line_filter_;
  vtkSmartPointer<vtkActor> split_intersect_line_actor_;

  NormalsActor border_normals_;
  NormalsActor split_normals_;

  ObserverRegistration key_press_listener;
};

} // walnut

#endif // WALNUT_EXAMPLES_BSP_VISUALIZATION_H__
