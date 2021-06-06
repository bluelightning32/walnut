#ifndef WALNUT_EXAMPLES_BSP_VISUALIZATION_H__
#define WALNUT_EXAMPLES_BSP_VISUALIZATION_H__

#include <cstring>
#include <vtkPassThroughFilter.h>
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
    negative_child_actor_ = window.AddShape(
        negative_child_filter_->GetOutputPort(), /*r=*/1.0, /*g=*/1.0,
        /*b=*/0.0, /*a=*/0.2);
    split_actor_ = window.AddShape(
        split_filter_->GetOutputPort(), /*r=*/0.5, /*g=*/1.0,
        /*b=*/0.0, /*a=*/0.4);
    positive_child_actor_ = window.AddShape(
        positive_child_filter_->GetOutputPort(), /*r=*/1.0, /*g=*/1.0,
        /*b=*/0.0, /*a=*/0.2);

    negative_child_wireframe_ = window.AddWireframe(
        negative_child_filter_->GetOutputPort());
    negative_child_wireframe_->GetProperty()->SetColor(/*r=*/1.0, /*g=*/1.0,
                                                       /*b=*/0.0);
    split_wireframe_ = window.AddWireframe(split_filter_->GetOutputPort());
    split_wireframe_->GetProperty()->SetColor(/*r=*/0.8, /*g=*/1.0, /*b=*/0.0);
    positive_child_wireframe_ = window.AddWireframe(
        positive_child_filter_->GetOutputPort());
    positive_child_wireframe_->GetProperty()->SetColor(/*r=*/1.0, /*g=*/1.0,
                                                       /*b=*/0.0);

    negative_child_normals_ = NormalsActor(
        window, negative_child_filter_->GetOutputPort(), /*scale=*/3,
        /*start3d=*/true);
    negative_child_normals_.SetColor(/*r=*/1.0, /*g=*/1.0, /*b=*/0.0);
    split_normals_ = NormalsActor(
        window, split_filter_->GetOutputPort(), /*scale=*/3,
        /*start3d=*/true);
    split_normals_.SetColor(/*r=*/0.8, /*g=*/1.0, /*b=*/0.0);
    positive_child_normals_ = NormalsActor(
        window, positive_child_filter_->GetOutputPort(), /*scale=*/3,
        /*start3d=*/true);
    positive_child_normals_.SetColor(/*r=*/1.0, /*g=*/1.0, /*b=*/0.0);

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
      line_actor = window.AddShape(line_filter->GetOutputPort(), r / 2, g / 2, b / 2, 1.0);
      line_actor->GetProperty()->SetLineWidth(7);
    }

    std::vector<const ConvexPolygon<>*> polygons;

    vtkNew<vtkPassThroughFilter> filter;
    vtkSmartPointer<vtkActor> actor;
    vtkNew<vtkPassThroughFilter> line_filter;
    vtkSmartPointer<vtkActor> line_actor;
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
    if (original_pos_->IsLeaf()) {
      std::vector<bool> child_path(chosen_branches_);
      vtkSmartPointer<vtkPolyData> shape =
        WalnutToVTKMesh(
            original_tree_.GetNodeBorderNoBoundWalls(child_path.begin(),
                                                     child_path.end(),
                                                     bounding_box_));
      // Only the negative child will be shown, but apply the shape to the
      // other two filters too so that the actors do not print warnings.
      negative_child_filter_->SetInputDataObject(shape);
      split_filter_->SetInputDataObject(shape);
      positive_child_filter_->SetInputDataObject(shape);
    } else {
      std::vector<bool> child_path(chosen_branches_);
      child_path.push_back(false);
      std::vector<MutableConvexPolygon<>> split_wall;
      std::vector<MutableConvexPolygon<>> negative_child_walls =
        original_tree_.GetNodeBorderNoBoundWalls(child_path.begin(),
                                                 child_path.end(),
                                                 bounding_box_);
      child_path.pop_back();
      child_path.push_back(true);
      std::vector<MutableConvexPolygon<>> positive_child_walls =
        original_tree_.GetNodeBorderNoBoundWalls(child_path.begin(),
                                                 child_path.end(),
                                                 bounding_box_);

      // Move the split wall out of negative_child_walls into split_wall.
      // Remove the split wall from positive_child_walls.
      auto split_it = std::partition(
          negative_child_walls.begin(), negative_child_walls.end(),
          [this](MutableConvexPolygon<>& wall) {
            return wall.plane() != original_pos_->split();
          });
      assert(split_it + 1 == negative_child_walls.end());
      split_wall.push_back(std::move(*split_it));
      negative_child_walls.pop_back();
      HalfSpace3 neg_split = -original_pos_->split();
      split_it = std::partition(
          positive_child_walls.begin(), positive_child_walls.end(),
          [&neg_split](MutableConvexPolygon<>& wall) {
            return wall.plane() != neg_split;
          });
      assert(split_it + 1 == positive_child_walls.end());
      positive_child_walls.pop_back();

      negative_child_filter_->SetInputDataObject(
          WalnutToVTKMesh(negative_child_walls));
      split_filter_->SetInputDataObject(WalnutToVTKMesh(split_wall));
      positive_child_filter_->SetInputDataObject(
          WalnutToVTKMesh(positive_child_walls));
    }

    std::map<BSPContentId, std::vector<MutableConvexPolygon<>>> content_map;
    std::map<BSPContentId, vtkNew<vtkPolyData>> edge_map;
    std::unordered_map<DoublePoint3, vtkIdType> point_map;
    vtkNew<vtkPoints> points;
    for (const BSPNodeRep::PolygonRep& polygon : pos_->contents()) {
      content_map[polygon.id].emplace_back(polygon);
      AddCoincidentEdges(polygon, point_map, points, edge_map);
    }
    for (const BSPNodeRep::PolygonRep& polygon : pos_->border_contents()) {
      content_map[polygon.id].emplace_back(polygon);
      AddCoincidentEdges(polygon, point_map, points, edge_map);
    }
    for (std::pair<const BSPContentId,
                   ContentInfo>& content_pair : contents_) {
      content_pair.second.filter->SetInputDataObject(WalnutToVTKMesh(
            content_map[content_pair.first]));
      content_pair.second.line_filter->SetInputDataObject(
          edge_map[content_pair.first]);
    }
  }

  void UpdateActorVisibility() {
    split_actor_->SetVisibility(!original_pos_->IsLeaf());
    split_wireframe_->SetVisibility(!original_pos_->IsLeaf());
    split_normals_.SetVisibility(!original_pos_->IsLeaf());
    positive_child_actor_->SetVisibility(!original_pos_->IsLeaf());
    positive_child_wireframe_->SetVisibility(!original_pos_->IsLeaf());
    positive_child_normals_.SetVisibility(!original_pos_->IsLeaf());
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
      std::map<BSPContentId, vtkNew<vtkPolyData>>& edge_map) {
    auto edge_it = edge_map.find(polygon.id);
    if (edge_it == edge_map.end()) {
      vtkNew<vtkPolyData> line_poly_data;
      auto lines = vtkSmartPointer<vtkCellArray>::New();
      line_poly_data->SetPoints(points);
      line_poly_data->SetLines(lines);
      auto inserted = edge_map.emplace(polygon.id, std::move(line_poly_data));
      assert(inserted.second);
      edge_it = inserted.first;
    }
    for (size_t i = 0; i < polygon.vertex_count(); ++i) {
      const auto& edge = polygon.edge(i);
      if (edge.edge_last_coincident.split != nullptr) {
        vtkIdType endpoints[2] = {
          MapPoint(edge.vertex().GetDoublePoint3(), point_map, points),
          MapPoint(polygon.vertex((i + 1) %
                                  polygon.vertex_count()).GetDoublePoint3(),
                   point_map, points)
        };
        edge_it->second->GetLines()->InsertNextCell(2, endpoints);
      }
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

  vtkNew<vtkPassThroughFilter> negative_child_filter_;
  vtkNew<vtkPassThroughFilter> split_filter_;
  vtkNew<vtkPassThroughFilter> positive_child_filter_;
  vtkSmartPointer<vtkActor> negative_child_actor_;
  vtkSmartPointer<vtkActor> split_actor_;
  vtkSmartPointer<vtkActor> positive_child_actor_;

  vtkSmartPointer<vtkActor> negative_child_wireframe_;
  vtkSmartPointer<vtkActor> split_wireframe_;
  vtkSmartPointer<vtkActor> positive_child_wireframe_;

  NormalsActor negative_child_normals_;
  NormalsActor split_normals_;
  NormalsActor positive_child_normals_;

  ObserverRegistration key_press_listener;
};

} // walnut

#endif // WALNUT_EXAMPLES_BSP_VISUALIZATION_H__
