#ifndef WALNUT_EXAMPLES_BSP_VISUALIZATION_H__
#define WALNUT_EXAMPLES_BSP_VISUALIZATION_H__

#include <vtkDoubleArray.h>
#include <vtkPassThroughFilter.h>
#include <vtkStringArray.h>

#include "normals_actor.h"
#include "visualization_window.h"
#include "walnut/bsp_tree.h"

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
                   const AABB& labelling_box, const BSPTreeRep& tree);

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
  bool Up();

  // Moves the current position to the specified child.
  //
  // If `branch` is true, a move to the positive child is attempted, otherwise
  // a move to the negative child is attempted.
  //
  // Returns true if the move was performed, or false if the current position
  // is a leaf (has no children).
  bool Down(bool branch);

  void UseTopDownView();
  void UseSecondView();
  void UseFourthView();

  void ShowLabels(bool show);

 private:
  struct ContentInfo {
    ContentInfo(VisualizationWindow& window, bool start3d, double r, double g,
                double b, double a);

    std::vector<const ConvexPolygon<>*> polygons;

    vtkNew<vtkPassThroughFilter> filter;
    vtkSmartPointer<vtkActor> actor;
    vtkNew<vtkPassThroughFilter> line_filter;
    vtkSmartPointer<vtkActor> line_actor;

    // Arrows for the normals of `vertex_last_coincident` and
    // `edge_last_coincident` of the polygons inside the tree.
    vtkNew<vtkPolyData> coincident_data;
    NormalsActor coincident_arrows;

    vtkNew<vtkPolyData> labelled_points_data;
    // Actor that displays `labelled_points_data_`. The actor will show any
    // updates to `labelled_points_data_`.
    vtkSmartPointer<vtkActor2D> labels_actor;
  };

  struct BuildingContentInfo {
    BuildingContentInfo() {
      coincident_normals->SetName("coincident_normals");
      coincident_normals->SetNumberOfComponents(3);
      labels->SetName("labels");
    }

    BuildingContentInfo(vtkPoints* points) : edges(MakeEdges(points)) {
      coincident_normals->SetName("coincident_normals");
      coincident_normals->SetNumberOfComponents(3);
      labels->SetName("labels");
    }

    void AddCrossing(const HomoPoint3& vertex,
                     const Vector3& label_offset_direction,
                     double label_offset_amount, bool pos_child, int type);

    std::vector<MutableConvexPolygon<>> facets;
    // Edges of the content polygons that are coincident with a split.
    vtkNew<vtkPolyData> edges;
    // Origin points for coincident normal arrows
    //
    // This contains information about both content vertices that are
    // coincident with a split plane, and content edges that are coincident
    // with a split plane. For coincident vertices, this contains their
    // location. For coincident edges, this contains their midpoint.
    vtkNew<vtkPoints> coincident_points;
    // The corresponding coincident split plane normal for each entry in
    // `coincident_points`. Every entry in `coincident_points` has an entry in
    // `coincident_normals` with the same id.
    vtkNew<vtkDoubleArray> coincident_normals;

    vtkNew<vtkPoints> labelled_points;
    vtkNew<vtkStringArray> labels;

   private:
    vtkNew<vtkPolyData> MakeEdges(vtkPoints* points);
  };

  std::array<double, 6> GetContentBounds() const;

  void AdjustNodePathToAvoidPoint(BSPTreeRep& tree_copy,
                                  std::vector<bool>& node_path,
                                  const HomoPoint3& avoid);

  void AddPWNLabel(vtkStringArray* labels, const std::string& label,
                   vtkPoints* labelled_points,
                   const std::vector<bool>& node_path, const HomoPoint3& top,
                   const HomoPoint3& new_top);

  bool KeyPressed(const char* key);

  std::string GetPWNString(
      const std::vector<BSPContentInfo>& content_info_by_id);

  void UpdateShapes();

  ContentInfo& GetContentInfo(BSPContentId id);

  vtkIdType MapPoint(const DoublePoint3& point,
                     std::unordered_map<DoublePoint3, vtkIdType> point_map,
                     vtkPoints* points);

  void AddCoincidentEdges(
      const BSPNodeRep::PolygonRep& polygon,
      std::unordered_map<DoublePoint3, vtkIdType> point_map,
      vtkPoints* points,
      std::map<BSPContentId, BuildingContentInfo>& content_map);

  void AddCrossingLabels(
      const BSPNodeRep::PolygonRep& polygon,
      std::map<BSPContentId, BuildingContentInfo>& content_map);

  void AddSplitOutline(const BSPNodeRep::PolygonRep& polygon,
                       std::unordered_map<DoublePoint3, vtkIdType> point_map,
                       vtkPoints* points, vtkPolyData* split_lines);

  VisualizationWindow& window_;
  AABB bounding_box_;
  AABB labelling_box_;
  double crossing_label_offset_;

  const BSPTreeRep& original_tree_;
  const BSPNodeRep* original_pos_;
  // false means the negative child was chosen, and true means the positive
  // child was chosen.
  std::vector<bool> chosen_branches_;
  BSPTreeRep tree_;
  // Position in `tree_` that corresponds to `original_pos_` in
  // `original_tree_`. This is updated as the user requests to go up and down
  // the BSP tree.
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

  // labelled_points_data_.GetPoints() has the location of every point that
  // needs a label. labelled_points_data_.GetAbstractArray("labels") contains
  // the labels for the points.
  //
  // The points and labels are on each tree move by replacing the point and
  // label arrays inside of `labelled_points_data_`.
  vtkNew<vtkPolyData> labelled_points_data_;
  // Actor that displays `labelled_points_data_`. The actor will show any
  // updates to `labelled_points_data_`.
  vtkSmartPointer<vtkActor2D> labels_actor_;

  vtkSmartPointer<vtkCubeAxesActor> axes_actor_;

  bool show_labels_ = true;
};

} // walnut

#endif // WALNUT_EXAMPLES_BSP_VISUALIZATION_H__
