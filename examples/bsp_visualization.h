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
                   const AABB& labelling_box);

  ~BSPVisualization();

  BSPContentId AllocateId() {
    return full_tree_.AllocateId();
  }

  // Adds content polygons to the visualization.
  template <typename InputConvexPolygon>
  void AddContent(BSPContentId id,
                  const std::vector<InputConvexPolygon>& polygons) {
    std::vector<ConvexPolygon<>>& contents_vector =
      GetContentInfo(id).polygons;
    for (const InputConvexPolygon& polygon : polygons) {
      contents_vector.push_back(polygon);
      view_tree_.AddContent(id, polygon);
      full_tree_.AddContent(id, polygon);
    }
    UpdateShapes();
  }

  // Adds the walls of a `aabb` to the visualization.
  void AddContent(BSPContentId id, const AABB& aabb);

  // Adds a content polygons to the visualization.
  template <typename InputConvexPolygon>
  void AddContent(BSPContentId id, const InputConvexPolygon& polygon) {
    GetContentInfo(id).polygons.push_back(&polygon);
    view_tree_.AddContent(id, polygon);
    full_tree_.AddContent(id, polygon);
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
  void UseTopDownView(const std::array<double, 6>& bounds);
  void UseSecondView();
  void UseFourthView();

  void ShowLabels(bool show);

  void SetMinAxesBounds(const AABB& aabb, bool ignore_contents = true) {
    ignore_contents_ = ignore_contents;
    min_axes_bounds_ = aabb;
  }

  BSPTreeRep& full_tree() {
    return full_tree_;
  }

  template <typename Filter>
  void FinishSplitting(Filter&& filter) {
    auto error_log = [](const std::string& error) {
      std::cerr << error << std::endl;
      assert(false);
    };
    walnut::ConnectingVisitor<Filter> visitor(filter, error_log);
    full_tree_.Traverse(visitor);
    UpdateShapes();
  }

  void UpdateShapes();

  // Splits by the:
  // 1. north-west facet
  // 2. south facet
  // 3. north-east facet
  // 4. south-west facet
  // 5. south-east facet
  // 6. north facet
  //
  // Returns the newly created node.
  BSPNode<>* SplitToTiltedCube(BSPNode<>* start);

  BSPNode<>* SplitToTiltedCube() {
    return SplitToTiltedCube(&full_tree_.root);
  }

  // Splits by the:
  // 1. north-west facet
  // 2. south facet
  // 3. north-east facet
  //
  // Returns the newly created node.
  BSPNode<>* SplitToTiltedCubeTopPlanes(BSPNode<>* start);

  BSPNode<>* SplitToTiltedCubeTopPlanes() {
    return SplitToTiltedCubeTopPlanes(&full_tree_.root);
  }

  // Splits by the:
  // 1. south-west facet
  // 2. south-east facet
  // 3. north facet
  //
  // Returns the newly created node.
  BSPNode<>* SplitToTiltedCubeBottomPlanes(BSPNode<>* start);

  BSPNode<>* SplitToTiltedCubeBottomPlanes() {
    return SplitToTiltedCubeBottomPlanes(&full_tree_.root);
  }

  // Splits the node with a plane with a normal that is facing south east. The
  // distance of the split plane is adjusted so that the point ((edge_dest -
  // edge_start) * edge_dist) is on the split plane. When viewed from the top,
  // this often creates a split edge that runs through the northwest side.
  //
  //        < (edge_dest - edge_start) * edge_dist
  //  +----s---------+
  //  |   s          |
  //  |  s           |
  //  | s            |
  //  |s             |
  //  s              |
  //  +--------------+
  //
  // The child node from that split that is on the north-west side is returned.
  //
  // If the input node is the tilted cube, the top path will follow along the
  // east of the north edge, then to the south-east of the outer north-west
  // edge.
  //
  //         /|\                         |
  //      --- | ---                      |
  //     / << |    \                     |
  //  /-- / ^ |     --\                  |
  //  |T</  T |       |                  |
  //  |     _/ \_     |                  |
  //  |    /     \    |                  |
  //  | ---       --- |                  |
  //  |/             \|                  |
  //  \               /                  |
  //   ---         ---                   |
  //      \       /                      |
  //       --- ---                       |
  //          v                          |
  //
  BSPNode<>* SplitNorthWest(BSPNode<>* start, const Point3& edge_start,
                            const Point3& edge_dest, double edge_dist);

  // These points form an approximated cube that is turned on its point. The
  // cube is approximated, because the coordinates of an exact cube would
  // contain either sqrt(2) or sqrt(3).
  //
  //                  north                          |
  //                   /|\                           |
  //                --- | ---                        |
  //  north_west   /    |    \   north_east          |
  //            /--     |     --\                    |
  //            |      top &    |                    |
  //            |     bottom    |                    |
  //            |    /     \    |                    |
  //            | ---       --- |                    |
  //            |/             \|                    |
  // south_west \               / south_east         |
  //             ---         ---                     |
  //                \       /                        |
  //                 --- ---                         |
  //                    v                            |
  //                  south                          |
  //
  static const Point3 tilted_cube_top;
  static const Point3 tilted_cube_bottom;
  static const Point3 tilted_cube_north;
  static const Point3 tilted_cube_north_west;
  static const Point3 tilted_cube_south_west;
  static const Point3 tilted_cube_south;
  static const Point3 tilted_cube_south_east;
  static const Point3 tilted_cube_north_east;

  // Contains pointers to the tilted cube points, except for the top and
  // bottom. The first entry is `tilted_cube_north`, and the subsequent ones go
  // counter-clockwise around the tilted cube.
  static const Point3* tilted_cube_peripheral_points[6];

 private:
  struct ContentInfo {
    ContentInfo(VisualizationWindow& window, bool start3d, double r, double g,
                double b, double a);

    std::vector<ConvexPolygon<>> polygons;

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
    BuildingContentInfo();

    BuildingContentInfo(vtkPoints* points);

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
                                  const HomoPoint3& avoid) const;

  HomoPoint3 GetLabelLocation(const std::vector<bool>& node_path,
                              const std::vector<HomoPoint3>& avoid) const;

  void AddPWNLabel(vtkStringArray* labels, const std::string& label,
                   vtkPoints* labelled_points, const HomoPoint3& location);

  bool KeyPressed(const char* key);

  std::string GetPWNString(
      const std::vector<BSPContentInfo>& content_info_by_id);

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
  AABB min_axes_bounds_;
  double crossing_label_offset_;

  BSPTreeRep full_tree_;
  const BSPNodeRep* full_tree_pos_;
  // false means the negative child was chosen, and true means the positive
  // child was chosen.
  std::vector<bool> chosen_branches_;
  BSPTreeRep view_tree_;
  // Position in `view_tree_` that corresponds to `full_tree_pos_` in
  // `full_tree_`. This is updated as the user requests to go up and down
  // the BSP tree.
  BSPNodeRep* view_pos_;

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
  // Ignore the contents when setting the axes bounds.
  bool ignore_contents_ = false;
};

} // walnut

#endif // WALNUT_EXAMPLES_BSP_VISUALIZATION_H__
