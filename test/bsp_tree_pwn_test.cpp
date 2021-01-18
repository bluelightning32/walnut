#include "walnut/bsp_tree.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/convex_polygon_factory.h"

namespace walnut {

using testing::IsEmpty;
using testing::SizeIs;

template<typename Container>
auto
MakeConvexPolygon(const Container& vertices) ->
ConvexPolygon<std::iterator_traits<
    decltype(std::begin(vertices))>::value_type::component_bits> {
  using Iterator = decltype(std::begin(vertices));
  using Point3Rep = typename std::iterator_traits<Iterator>::value_type;
  using Factory = ConvexPolygonFactory<Point3Rep>;
  using ConvexPolygonRep = typename Factory::ConvexPolygonRep;

  class CollectOne : public Factory {
   public:
    CollectOne() = default;

    ConvexPolygonRep&& GetResult() {
      EXPECT_EQ(received_, 1);
      return std::move(result_);
    }

   protected:
    void Emit(ConvexPolygonRep&& polygon) override {
      EXPECT_EQ(received_, 0);
      received_++;
      result_ = std::move(polygon);
    }

   private:
    int received_ = 0;
    ConvexPolygonRep result_;
  };

  CollectOne collector;
  collector.Build(std::begin(vertices), std::end(vertices));
  return collector.GetResult();
}

// These points form an approximated cube that is turned on its point. The cube
// is approximated, because the coordinates of an exact cube would contain
// either sqrt(2) or sqrt(3).
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
const Point3<> cube_top(0, 0, 11);
const Point3<> cube_bottom(0, 0, -11);
const Point3<> cube_north(0, 10, 3);
const Point3<> cube_north_west(-9, 5, -4);
const Point3<> cube_south_west(-9, -5, 4);
const Point3<> cube_south(0, -10, -3);
const Point3<> cube_south_east(9, -5, 4);
const Point3<> cube_north_east(9, 5, -4);

const Point3<>* cube_peripheral_points[6] = {
  &cube_north,
  &cube_north_west,
  &cube_south_west,
  &cube_south,
  &cube_south_east,
  &cube_north_east,
};

TEST(TitltedCube, NorthEastSide) {
  HalfSpace3<> side(cube_top, cube_south_east, cube_north_east);
  EXPECT_TRUE(side.IsCoincident(cube_top));
  EXPECT_TRUE(side.IsCoincident(cube_south_east));
  EXPECT_TRUE(side.IsCoincident(cube_north_east));
  EXPECT_TRUE(side.IsCoincident(cube_north));
}

TEST(TitltedCube, NorthSide) {
  HalfSpace3<> side(cube_bottom, cube_north_west, cube_north);
  EXPECT_TRUE(side.IsCoincident(cube_bottom));
  EXPECT_TRUE(side.IsCoincident(cube_north_west));
  EXPECT_TRUE(side.IsCoincident(cube_north));
  EXPECT_TRUE(side.IsCoincident(cube_north_east));
}

TEST(TitltedCube, NorthWestSide) {
  HalfSpace3<> side(cube_top, cube_north, cube_north_west);
  EXPECT_TRUE(side.IsCoincident(cube_top));
  EXPECT_TRUE(side.IsCoincident(cube_north));
  EXPECT_TRUE(side.IsCoincident(cube_north_west));
  EXPECT_TRUE(side.IsCoincident(cube_south_west));
}

TEST(TitltedCube, SouthWestSide) {
  HalfSpace3<> side(cube_bottom, cube_south, cube_south_west);
  EXPECT_TRUE(side.IsCoincident(cube_bottom));
  EXPECT_TRUE(side.IsCoincident(cube_south));
  EXPECT_TRUE(side.IsCoincident(cube_south_west));
  EXPECT_TRUE(side.IsCoincident(cube_north_west));
}

TEST(TitltedCube, SouthSide) {
  HalfSpace3<> side(cube_top, cube_south_west, cube_south);
  EXPECT_TRUE(side.IsCoincident(cube_top));
  EXPECT_TRUE(side.IsCoincident(cube_south_west));
  EXPECT_TRUE(side.IsCoincident(cube_south));
  EXPECT_TRUE(side.IsCoincident(cube_south_east));
}

TEST(TitltedCube, SouthEastSide) {
  HalfSpace3<> side(cube_bottom, cube_north_east, cube_south_east);
  EXPECT_TRUE(side.IsCoincident(cube_bottom));
  EXPECT_TRUE(side.IsCoincident(cube_north_east));
  EXPECT_TRUE(side.IsCoincident(cube_south_east));
  EXPECT_TRUE(side.IsCoincident(cube_south));
}

// First parameter:
//   false - return the negative child from the directional splits like
//           `SplitNorthWest`. The tilted cube is made out of negative
//           children.
//   true  - return the positive child from the directional splits like
//           `SplitNorthWest`. The tilted cube is made out of negative
//           children.
//
// Second parameter:
//   false - `AddContent` adds the input polygon as is.
//   true  - `AddContent` flips the polygon before adding it to the tree.
class BSPTreePWN : public testing::TestWithParam<std::tuple<bool, bool>> {
 protected:
  // Splits `tree_` 6 times to form all sides of the tilted cube. The descendant
  // node for the inside of the cube is returned.
  BSPNode<>* SplitToCube() {
    EXPECT_TRUE(tree_.root.IsLeaf());

    std::vector<BSPNode<>::HalfSpace3Rep> split_planes;
    for (int i = 0; i < 6; i += 2) {
      const Point3<>* this_point = cube_peripheral_points[i];
      const Point3<>* next_point = cube_peripheral_points[(i + 2)%6];

      split_planes.emplace_back(*this_point, *next_point, cube_top);
    }
    for (int i = 1; i < 6; i += 2) {
      const Point3<>* this_point = cube_peripheral_points[i];
      const Point3<>* next_point = cube_peripheral_points[(i + 2)%6];

      split_planes.emplace_back(*next_point, *this_point, cube_bottom);
    }
    EXPECT_EQ(split_planes.size(), 6);

    BSPNode<>* pos = &tree_.root;
    // Splits by the:
    // 1. north-west facet
    // 2. south facet
    // 3. north-east facet
    // 4. south-west facet
    // 5. south-east facet
    // 6. north facet
    for (BSPNode<>::HalfSpace3Rep& split_plane : split_planes) {
      pos->Split(split_plane);
      pos = pos->negative_child();
    }
    return pos;
  }


  // Splits the node with a vertical plane with a normal that is facing north
  // west or south east (depending on the test's first parameter). The distance
  // of the split plane is adjusted so that the point
  // ((edge_dest - edge_start) * edge_dist) is on the split plane.
  //
  // The child on the north-west side is returned.
  //
  // If the input node is the cube, the M-int path will follow along the east
  // of the north edge, then to the south-east of the outer north-west edge.
  //
  //         /|\                         |
  //      --- | ---                      |
  //     / << |    \                     |
  //  /-- / ^ |     --\                  |
  //  |M</  M |       |                  |
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
  BSPNode<>* SplitNorthWest(BSPNode<>* parent, const Point3<>& edge_start,
                            const Point3<>& edge_dest, double edge_dist) {
    EXPECT_TRUE(parent->IsLeaf());
    Vector3<> normal = std::get<0>(GetParam()) ? Vector3<>(-1, 1, -10) :
                                                 Vector3<>(1, -1, 10);

    auto edge_vector = edge_dest - edge_start;
    double dist = double(normal.x()) * (double(edge_start.x()) +
                                        double(edge_vector.x()) * edge_dist) +
                  double(normal.y()) * (double(edge_start.y()) +
                                        double(edge_vector.y()) * edge_dist) +
                  double(normal.z()) * (double(edge_start.z()) +
                                        double(edge_vector.z()) * edge_dist);
    HalfSpace3<> split(normal, HalfSpace3<>::DistInt(long(dist)));
    parent->Split(split);
    BSPNode<>* return_child;
    BSPNode<>* other_child;
    if (std::get<0>(GetParam())) {
      return_child = parent->positive_child();
      other_child = parent->negative_child();
    } else {
      return_child = parent->negative_child();
      other_child = parent->positive_child();
    }
    for (BSPPolygonId id = 0; id < tree_.next_id(); ++id) {
      EXPECT_EQ(other_child->GetPWNForId(id), parent->GetPWNForId(id))
        << "id=" << id;
    }
    return return_child;
  }

  // Add a polygon to `tree_`, flipping it first if configured by the test
  // parameters.
  template <typename InputPolygon>
  void AddContent(BSPPolygonId id, const InputPolygon& polygon) {
    auto added_to_leaf = [&](BSPNode<>& leaf) { };
    if (std::get<1>(GetParam())) {
      // flip the input polygon
      std::vector<typename InputPolygon::HomoPoint3Rep> vertices;
      for (auto it = polygon.edges().rbegin(); it != polygon.edges().rend();
           ++it) {
        vertices.push_back(it->vertex);
      }
      using ConvexPolygonRep = ConvexPolygon<InputPolygon::point3_bits>;
      tree_.AddContent(ConvexPolygonRep(-polygon.plane(),
                                        polygon.drop_dimension(), vertices),
                       id, added_to_leaf);
    } else {
      tree_.AddContent(polygon, id, added_to_leaf);
    }
  }

  // Returns 1 if the input polygons were not flipped or -1 if the input
  // polygons were flipped, based on the test parameter.
  int GetPWNFlip() const {
    if (std::get<1>(GetParam())) {
      return -1;
    } else {
      return 1;
    }
  }

  BSPTree<> tree_;
};

TEST_P(BSPTreePWN, EmptyCube) {
  BSPPolygonId id = tree_.AllocateId();
  EXPECT_EQ(id, 0);

  BSPNode<>* inside_cube = SplitToCube();
  EXPECT_EQ(inside_cube->GetPWNForId(id), 0);

  BSPNode<>* child = SplitNorthWest(inside_cube, cube_top, cube_north, 0.75);
  EXPECT_EQ(child->GetPWNForId(id), 0);
}

TEST_P(BSPTreePWN, BeforeCrossing) {
  BSPPolygonId id = tree_.AllocateId();
  EXPECT_EQ(id, 0);

  RectangularPrism<> prism(/*min_x=*/cube_north_west.x(),
                           /*min_y=*/cube_north.y()*3/4,
                           /*min_z=*/cube_bottom.z(),
                           /*max_x=*/cube_north_east.x(),
                           /*max_y=*/cube_north.y() + 10,
                           /*max_z=*/cube_top.z());

  for (const ConvexPolygon<>& wall : prism.GetWalls()) {
    AddContent(id, wall);
  }

  BSPNode<>* inside_cube = SplitToCube();
  EXPECT_EQ(inside_cube->GetPWNForId(id), 0);
  EXPECT_TRUE(inside_cube->IsLeaf());
  EXPECT_THAT(inside_cube->border_contents(), IsEmpty());
  EXPECT_THAT(inside_cube->contents(), SizeIs(1));

  BSPNode<>* child = SplitNorthWest(inside_cube, cube_top, cube_north, 0.5);
  EXPECT_TRUE(child->IsLeaf());
  EXPECT_THAT(child->border_contents(), IsEmpty());
  EXPECT_THAT(child->contents(), SizeIs(1));
  EXPECT_EQ(child->GetPWNForId(id), 0);
}

TEST_P(BSPTreePWN, SimpleCrossing) {
  BSPPolygonId id = tree_.AllocateId();
  EXPECT_EQ(id, 0);

  int y_dist = cube_north.y().ToInt() - cube_north_west.y().ToInt();

  Point3<>::BigIntRep polygon_y = cube_north_west.y() + y_dist/2;
  // A rectangle that goes from
  // (cube_north_west.x(), polygon_y, cube_bottom.z()) to
  // (cube_north_east.x(), polygon_y, cube_top.z()).
  //
  // The unflipped version's normal points towards (0, 0, 0).
  Point3<> polygon_vertices[] = {
    Point3<>(cube_north_east.x(), polygon_y, cube_top.z()),
    Point3<>(cube_north_west.x(), polygon_y, cube_top.z()),
    Point3<>(cube_north_west.x(), polygon_y, cube_bottom.z()),
    Point3<>(cube_north_east.x(), polygon_y, cube_bottom.z())
  };

  AddContent(id, MakeConvexPolygon(polygon_vertices));

  // To make this test case simpler, only split the tree with the top left and
  // top right sides of the cube.
  std::vector<BSPNode<>::HalfSpace3Rep> split_planes;
  split_planes.emplace_back(cube_top, cube_north, cube_north_west);
  split_planes.emplace_back(cube_north_east, cube_north, cube_top);
  BSPNode<>* inside = &tree_.root;
  for (BSPNode<>::HalfSpace3Rep& split_plane : split_planes) {
    inside->Split(split_plane);
    inside = inside->negative_child();
  }

  EXPECT_EQ(inside->GetPWNForId(id), 0);
  EXPECT_TRUE(inside->IsLeaf());
  EXPECT_THAT(inside->border_contents(), IsEmpty());
  EXPECT_THAT(inside->contents(), SizeIs(1));

  BSPNode<>* child = SplitNorthWest(inside, cube_north_west, cube_north, 0.75);
  EXPECT_TRUE(child->IsLeaf());
  EXPECT_THAT(child->border_contents(), IsEmpty());
  EXPECT_THAT(child->contents(), SizeIs(1));
  EXPECT_EQ(child->GetPWNForId(id), 1 * GetPWNFlip());
}

TEST_P(BSPTreePWN, SimpleCrossing2) {
  // This is almost the same as SimpleCrossing, except a full cubes are used.
  BSPPolygonId id = tree_.AllocateId();
  EXPECT_EQ(id, 0);

  int y_dist = cube_north.y().ToInt() - cube_north_west.y().ToInt();

  Point3<>::BigIntRep polygon_y = cube_north_west.y() + y_dist/2;
  RectangularPrism<> prism(/*min_x=*/cube_north_west.x(),
                           /*min_y=*/polygon_y,
                           /*min_z=*/cube_bottom.z(),
                           /*max_x=*/cube_north_east.x(),
                           /*max_y=*/cube_north.y(),
                           /*max_z=*/cube_top.z());

  for (const ConvexPolygon<>& wall : prism.GetWalls()) {
    AddContent(id, wall);
  }

  BSPNode<>* inside_cube = SplitToCube();
  EXPECT_EQ(inside_cube->GetPWNForId(id), 0);
  EXPECT_TRUE(inside_cube->IsLeaf());
  EXPECT_THAT(inside_cube->border_contents(), IsEmpty());
  EXPECT_THAT(inside_cube->contents(), SizeIs(1));

  BSPNode<>* child = SplitNorthWest(inside_cube, cube_north_west, cube_north,
                                    0.75);
  EXPECT_TRUE(child->IsLeaf());
  EXPECT_THAT(child->border_contents(), IsEmpty());
  EXPECT_THAT(child->contents(), SizeIs(1));
  EXPECT_EQ(child->GetPWNForId(id), 1 * GetPWNFlip());
}

TEST_P(BSPTreePWN, MinimumExcluded) {
  // Verify that minimums are excluded from the PWN calculation.
  BSPPolygonId id = tree_.AllocateId();
  EXPECT_EQ(id, 0);

  Point3<>::BigIntRep polygon_x(cube_south_east.x().ToInt()/2);
  RectangularPrism<> prism(/*min_x=*/polygon_x,
                           /*min_y=*/cube_south.y(),
                           /*min_z=*/cube_bottom.z(),
                           /*max_x=*/cube_north_east.x(),
                           /*max_y=*/cube_north.y(),
                           /*max_z=*/cube_top.z());

  for (const ConvexPolygon<>& wall : prism.GetWalls()) {
    AddContent(id, wall);
  }

  BSPNode<>* inside_cube = SplitToCube();
  EXPECT_EQ(inside_cube->GetPWNForId(id), 0);
  EXPECT_TRUE(inside_cube->IsLeaf());
  EXPECT_THAT(inside_cube->border_contents(), IsEmpty());
  EXPECT_THAT(inside_cube->contents(), SizeIs(1));

  BSPNode<>* child = SplitNorthWest(inside_cube, cube_top, cube_south_east,
                                    0.75);
  EXPECT_TRUE(child->IsLeaf());
  EXPECT_THAT(child->border_contents(), IsEmpty());
  EXPECT_THAT(inside_cube->contents(), SizeIs(1));
  EXPECT_EQ(child->GetPWNForId(id), 0);
}

TEST_P(BSPTreePWN, IPathBends) {
  // This test verifies that the I-path correctly follows the bends around the
  // object, instead of blindly using the split normal.
  //
  // The test uses the I-path that follows along the north edge of the cube,
  // then the outer north-west edge towards the north-west vertex. Overlapping
  // with the titled cube on the north end are two other rectangular prisms.
  // The I-path will enter prism1, then prism2, leave prism2, then leave
  // prism1. This test verifies a point on the I-path that is inside prism1 but
  // outside prism2.
  int y_dist = cube_north.y().ToInt() - cube_north_west.y().ToInt();

  BSPPolygonId prism1_id = tree_.AllocateId();
  Point3<>::BigIntRep prism1_y = cube_north_west.y() + y_dist/5;
  RectangularPrism<> prism1(/*min_x=*/cube_north_west.x(),
                            /*min_y=*/prism1_y,
                            /*min_z=*/cube_bottom.z(),
                            /*max_x=*/cube_north_east.x(),
                            /*max_y=*/cube_north.y(),
                            /*max_z=*/cube_top.z());
  for (const ConvexPolygon<>& wall : prism1.GetWalls()) {
    AddContent(prism1_id, wall);
  }

  BSPPolygonId prism2_id = tree_.AllocateId();
  Point3<>::BigIntRep prism2_y = cube_north_west.y() + y_dist*3/5;
  RectangularPrism<> prism2(/*min_x=*/cube_north_west.x(),
                            /*min_y=*/prism2_y,
                            /*min_z=*/cube_bottom.z(),
                            /*max_x=*/cube_north_east.x(),
                            /*max_y=*/cube_north.y(),
                            /*max_z=*/cube_top.z());
  for (const ConvexPolygon<>& wall : prism2.GetWalls()) {
    AddContent(prism2_id, wall);
  }

  BSPNode<>* inside_cube = SplitToCube();
  EXPECT_EQ(inside_cube->GetPWNForId(prism1_id), 0);
  EXPECT_EQ(inside_cube->GetPWNForId(prism2_id), 0);
  EXPECT_TRUE(inside_cube->IsLeaf());
  EXPECT_THAT(inside_cube->border_contents(), IsEmpty());
  EXPECT_THAT(inside_cube->contents(), SizeIs(2));

  BSPNode<>* inside_prism1 = SplitNorthWest(inside_cube, cube_north_west,
                                            cube_north, 2.0/5);
  EXPECT_TRUE(inside_prism1->IsLeaf());
  EXPECT_THAT(inside_prism1->border_contents(), IsEmpty());
  EXPECT_THAT(inside_prism1->contents(), SizeIs(1));
  EXPECT_EQ(inside_prism1->GetPWNForId(prism1_id), 1 * GetPWNFlip());
  EXPECT_EQ(inside_prism1->GetPWNForId(prism2_id), 0);
}

TEST_P(BSPTreePWN, SkipEdgesAlongIPath) {
  // This test verifies that the I-path skips polygon edges that follow the
  // I-path.
  //
  // The test starts with a rectangular prism, then cuts off the top so that
  // there is an edge that runs parallel to the tilted cube's north edge, but
  // the cut off prism's top facet only touches the tilted cube's north-west
  // facet at the north edge.

  BSPPolygonId id = tree_.AllocateId();
  Point3<>::BigIntRep bounding_start_y(cube_north.y().ToInt() / 4);
  Point3<>::BigIntRep bounding_end_y(cube_north.y().ToInt() * 3 / 4);
  RectangularPrism<> bounding_box(/*min_x=*/cube_north_west.x(),
                                  /*min_y=*/bounding_start_y,
                                  /*min_z=*/cube_bottom.z(),
                                  /*max_x=*/cube_top.x(),
                                  /*max_y=*/bounding_end_y,
                                  /*max_z=*/cube_top.z());
  walnut::BSPTree<> slice_top_tree;
  const Point3<> cube_north_west_down(cube_north_west.x(), cube_north_west.y(),
      Point3<>::BigIntRep(cube_north_west.z() - 1));
  walnut::HalfSpace3<> slice_top_plane(cube_top, cube_north,
                                       cube_north_west_down);
  slice_top_tree.root.Split(slice_top_plane);
  // Use the negative half of slice_top_plane.
  std::vector<bool> node_path { false };
  for (const walnut::BSPTree<>::OutputPolygon& wall :
       slice_top_tree.GetNodeBorder(node_path.begin(), node_path.end(),
                                    bounding_box)) {
    AddContent(id, wall);
  }

  BSPNode<>* inside_cube = SplitToCube();
  EXPECT_EQ(inside_cube->GetPWNForId(id), 0);
  EXPECT_TRUE(inside_cube->IsLeaf());
  EXPECT_THAT(inside_cube->border_contents(), IsEmpty());
  // Front, back, side, and top. The bottom is not inside the titled cube.
  EXPECT_THAT(inside_cube->contents(), SizeIs(4));

  BSPNode<>* along_edge = SplitNorthWest(inside_cube, cube_top,
                                         cube_north, 2.0/4);
  EXPECT_TRUE(along_edge->IsLeaf());
  EXPECT_THAT(along_edge->border_contents(), IsEmpty());
  EXPECT_THAT(along_edge->contents(), SizeIs(4));
  EXPECT_EQ(along_edge->GetPWNForId(id), 0);
}

TEST_P(BSPTreePWN, IPathGoesThroughLeftSide) {
  // This test verifies that the I-path goes through the left side of the edges
  // it follows.
  //
  // The test has a rectangular prism that overlaps with the tilted cube on the
  // left side of the north edge. Since the I-path follows along the left side
  // of its edges, it should enter the prism.

  BSPPolygonId id = tree_.AllocateId();
  Point3<>::BigIntRep prism_start_y(cube_north.y().ToInt() / 4);
  Point3<>::BigIntRep prism_end_y(cube_north.y().ToInt() * 3 / 4);
  RectangularPrism<> prism(/*min_x=*/cube_north_west.x(),
                           /*min_y=*/prism_start_y,
                           /*min_z=*/cube_bottom.z(),
                           /*max_x=*/cube_top.x(),
                           /*max_y=*/prism_end_y,
                           /*max_z=*/cube_top.z());
  for (const ConvexPolygon<>& wall : prism.GetWalls()) {
    AddContent(id, wall);
  }

  BSPNode<>* inside_cube = SplitToCube();
  EXPECT_EQ(inside_cube->GetPWNForId(id), 0);
  EXPECT_TRUE(inside_cube->IsLeaf());
  EXPECT_THAT(inside_cube->border_contents(), IsEmpty());
  EXPECT_THAT(inside_cube->contents(), SizeIs(3));

  BSPNode<>* inside_prism1 = SplitNorthWest(inside_cube, cube_top,
                                            cube_north, 2.0/4);
  EXPECT_TRUE(inside_prism1->IsLeaf());
  EXPECT_THAT(inside_prism1->border_contents(), IsEmpty());
  EXPECT_THAT(inside_prism1->contents(), SizeIs(3));
  EXPECT_EQ(inside_prism1->GetPWNForId(id), 1 * GetPWNFlip());
}

TEST_P(BSPTreePWN, IPathSkipsRightSide) {
  // This test verifies that the I-path goes through the left side of the edges
  // it follows, and does not pass through a polygon that does not touch the
  // facet to the left of the edge.
  //
  // The test has a rectangular prism that overlaps with the tilted cube on the
  // right side of the north edge. Since the I-path follows along the left side
  // of its edges, the PWN should not be affected by the prism.

  BSPPolygonId id = tree_.AllocateId();
  Point3<>::BigIntRep prism_start_y(cube_north.y().ToInt() / 4);
  Point3<>::BigIntRep prism_end_y(cube_north.y().ToInt() * 3 / 4);
  RectangularPrism<> prism(/*min_x=*/cube_top.x(),
                           /*min_y=*/prism_start_y,
                           /*min_z=*/cube_bottom.z(),
                           /*max_x=*/cube_north_east.x(),
                           /*max_y=*/prism_end_y,
                           /*max_z=*/cube_top.z());
  for (const ConvexPolygon<>& wall : prism.GetWalls()) {
    AddContent(id, wall);
  }

  BSPNode<>* inside_cube = SplitToCube();
  EXPECT_EQ(inside_cube->GetPWNForId(id), 0);
  EXPECT_TRUE(inside_cube->IsLeaf());
  EXPECT_THAT(inside_cube->border_contents(), IsEmpty());
  EXPECT_THAT(inside_cube->contents(), SizeIs(3));

  BSPNode<>* inside_prism1 = SplitNorthWest(inside_cube, cube_top,
                                            cube_north, 2.0/4);
  EXPECT_TRUE(inside_prism1->IsLeaf());
  EXPECT_THAT(inside_prism1->border_contents(), IsEmpty());
  EXPECT_THAT(inside_prism1->contents(), SizeIs(3));
  EXPECT_EQ(inside_prism1->GetPWNForId(id), 0);
}

TEST_P(BSPTreePWN, BothEdgeSidesTouchIPath) {
  // This test verifies that the I-path sees a crossing, even if both sides of
  // the crossed edge are on the I-path, but on from different edges of the
  // I-path.
  //
  // The test has a rectangular prism that overlaps with the tilted cube on the
  // left side of the north edge. Since the I-path follows along the left side
  // of its edges, it should enter the prism.

  BSPPolygonId id = tree_.AllocateId();
  Point3<>::BigIntRep prism_start_y(cube_north.y().ToInt() / 4);
  Point3<>::BigIntRep prism_end_y(cube_north.y().ToInt() * 3 / 4);
  RectangularPrism<> prism(/*min_x=*/cube_north_west.x(),
                           /*min_y=*/prism_start_y,
                           /*min_z=*/cube_bottom.z(),
                           /*max_x=*/cube_top.x(),
                           /*max_y=*/prism_end_y,
                           /*max_z=*/cube_top.z());
  for (const ConvexPolygon<>& wall : prism.GetWalls()) {
    AddContent(id, wall);
  }

  BSPNode<>* inside_cube = SplitToCube();
  EXPECT_EQ(inside_cube->GetPWNForId(id), 0);
  EXPECT_TRUE(inside_cube->IsLeaf());
  EXPECT_THAT(inside_cube->border_contents(), IsEmpty());
  EXPECT_THAT(inside_cube->contents(), SizeIs(3));

  BSPNode<>* inside_prism1 = SplitNorthWest(inside_cube, cube_top,
                                            cube_north, 7.0/8);
  EXPECT_TRUE(inside_prism1->IsLeaf());
  EXPECT_THAT(inside_prism1->border_contents(), IsEmpty());
  EXPECT_THAT(inside_prism1->contents(), SizeIs(3));
  EXPECT_EQ(inside_prism1->GetPWNForId(id), 0);
}

TEST_P(BSPTreePWN, SplitTwice) {
  // This is almost the same as SimpleCrossing, except that the child is split
  // again. The only polygon is already passed, so the second split should not
  // impact the PWN.
  BSPPolygonId id = tree_.AllocateId();
  EXPECT_EQ(id, 0);

  int y_dist = cube_north.y().ToInt() - cube_north_west.y().ToInt();

  Point3<>::BigIntRep polygon_y = cube_north_west.y() + y_dist/2;
  // A rectangle that goes from
  // (cube_north_west.x(), polygon_y, cube_bottom.z()) to
  // (cube_north_east.x(), polygon_y, cube_top.z()).
  //
  // The unflipped version's normal points towards (0, 0, 0).
  Point3<> polygon_vertices[] = {
    Point3<>(cube_north_east.x(), polygon_y, cube_top.z()),
    Point3<>(cube_north_west.x(), polygon_y, cube_top.z()),
    Point3<>(cube_north_west.x(), polygon_y, cube_bottom.z()),
    Point3<>(cube_north_east.x(), polygon_y, cube_bottom.z())
  };

  AddContent(id, MakeConvexPolygon(polygon_vertices));

  // To make this test case simpler, only split the tree with the top left and
  // top right sides of the cube.
  std::vector<BSPNode<>::HalfSpace3Rep> split_planes;
  split_planes.emplace_back(cube_top, cube_north, cube_north_west);
  split_planes.emplace_back(cube_north_east, cube_north, cube_top);
  BSPNode<>* inside = &tree_.root;
  for (BSPNode<>::HalfSpace3Rep& split_plane : split_planes) {
    inside->Split(split_plane);
    inside = inside->negative_child();
  }

  EXPECT_EQ(inside->GetPWNForId(id), 0);
  EXPECT_TRUE(inside->IsLeaf());
  EXPECT_THAT(inside->border_contents(), IsEmpty());
  EXPECT_THAT(inside->contents(), SizeIs(1));

  BSPNode<>* child = SplitNorthWest(inside, cube_north_west, cube_north,
                                    3.0/4);
  EXPECT_TRUE(child->IsLeaf());
  EXPECT_THAT(child->border_contents(), IsEmpty());
  EXPECT_THAT(child->contents(), SizeIs(1));
  EXPECT_EQ(child->GetPWNForId(id), 1 * GetPWNFlip());

  BSPNode<>* child2 = SplitNorthWest(child, cube_north_west, cube_north,
                                     7.0/8);
  EXPECT_TRUE(child2->IsLeaf());
  EXPECT_THAT(child2->border_contents(), IsEmpty());
  EXPECT_THAT(child2->contents(), SizeIs(1));
  EXPECT_EQ(child2->GetPWNForId(id), 1 * GetPWNFlip());
}

TEST_P(BSPTreePWN, IPathOvershootsMValue) {
  // This test creates a rectangular prism that overlaps the north half of the
  // titled cube, starting at cube_top. Even though the prism ends at the cube
  // top vertex, the titled cube splits overshoot the prism, which means they
  // enter then exit it for a PWN of 0.
  //
  // Later the titled cube is split again on the north-west side. For that
  // split, the prism is re-entered.
  BSPPolygonId id = tree_.AllocateId();
  EXPECT_EQ(id, 0);

  Point3<>::BigIntRep polygon_y(cube_north.y().ToInt()/4);
  RectangularPrism<> prism(/*min_x=*/cube_north_west.x(),
                           /*min_y=*/cube_top.y(),
                           /*min_z=*/cube_bottom.z(),
                           /*max_x=*/cube_north_east.x(),
                           /*max_y=*/cube_north.y(),
                           /*max_z=*/cube_top.z());

  for (const ConvexPolygon<>& wall : prism.GetWalls()) {
    AddContent(id, wall);
  }

  BSPNode<>* inside_cube = SplitToCube();
  EXPECT_TRUE(inside_cube->IsLeaf());
  EXPECT_THAT(inside_cube->border_contents(), IsEmpty());
  EXPECT_THAT(inside_cube->contents(), SizeIs(1));
  EXPECT_EQ(inside_cube->GetPWNForId(id), 0);

  BSPNode<>* child = SplitNorthWest(inside_cube, cube_top, cube_north,
                                    1.0/2);
  EXPECT_TRUE(child->IsLeaf());
  EXPECT_THAT(child->border_contents(), IsEmpty());
  EXPECT_THAT(child->contents(), SizeIs(1));
  EXPECT_EQ(child->GetPWNForId(id), 1 * GetPWNFlip());
}

INSTANTIATE_TEST_SUITE_P(, BSPTreePWN,
    testing::Combine(testing::Bool(), testing::Bool()));

}  // walnut
