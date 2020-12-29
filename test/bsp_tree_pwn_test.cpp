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
//                  north
//                   /|\
//                --- | ---
//  north_west   /    |    \   north_east
//            /--     |     --\
//            |      top &    |
//            |     bottom    |
//            |    /     \    |
//            | ---       --- |
//            |/             \|
// south_west \               / south_east
//             ---         ---
//                \       /
//                 --- ---
//                    v
//                  south
//
const Point3<> cube_top(0, 0, 11);
const Point3<> cube_bottom(0, 0, -11);
const Point3<> cube_north(0, 10, 4);
const Point3<> cube_north_west(-9, 5, -4);
const Point3<> cube_south_west(-9, -5, 4);
const Point3<> cube_south(0, -10, -4);
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

class BSPTreePWN : public testing::TestWithParam<std::tuple<bool, bool>> {
 protected:
  // Splits `tree` 6 times to form all sides of the tilted cube. The descendant
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
    for (BSPNode<>::HalfSpace3Rep& split_plane : split_planes) {
      pos->Split(split_plane);
      pos = pos->negative_child();
    }
    return pos;
  }


  // Splits the node with a vertical plane with a normal that is facing north
  // west or south east (depending on the test's first parameter). The distance
  // of the split plane is adjusted so that (on_border_x, on_border_y,
  // arbitrary_z) is on the split plane.
  //
  // The child on the north-west side is returned.
  //
  // If the input node is the cube, the M-int path will follow along the east
  // of the north edge, then to the south-east of the outer north-west edge.
  //
  //         /|\
  //      --- | ---
  //     / << |    \
  //  /-- / ^ |     --\
  //  |M</  M |       |
  //  |     _/ \_     |
  //  |    /     \    |
  //  | ---       --- |
  //  |/             \|
  //  \               /
  //   ---         ---
  //      \       /
  //       --- ---
  //          v
  //
  BSPNode<>* SplitNorthWest(BSPNode<>* parent, int on_border_x,
                            int on_border_y) {
    EXPECT_TRUE(parent->IsLeaf());
    std::pair<int, int> normal = std::get<0>(GetParam()) ?
      std::pair<int, int>(-1, 1) :
      std::pair<int, int>(1, -1);

    int dist = normal.first*on_border_x + normal.second*on_border_y;
    HalfSpace3<> split(normal.first, normal.second, 0, dist);
    parent->Split(split);
    if (std::get<0>(GetParam())) {
      return parent->positive_child();
    } else {
      return parent->negative_child();
    }
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
      tree_.AddContent(InputPolygon(-polygon.plane(), polygon.drop_dimension(),
                                    vertices), id,
                       added_to_leaf);
    } else {
      tree_.AddContent(polygon, id, added_to_leaf);
    }
  }

  BSPTree<> tree_;
};

TEST_P(BSPTreePWN, EmptyCube) {
  BSPPolygonId id = tree_.AllocateId();
  EXPECT_EQ(id, 0);

  BSPNode<>* inside_cube = SplitToCube();
  EXPECT_EQ(inside_cube->GetPWNForId(id), 0);

  BSPNode<>* child = SplitNorthWest(inside_cube, 0,
                                    cube_north.y().ToInt() * 3 / 4);
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

  BSPNode<>* child = SplitNorthWest(inside_cube, 0,
                                    cube_north.y().ToInt() * 3 / 4);
  EXPECT_TRUE(child->IsLeaf());
  EXPECT_THAT(child->border_contents(), IsEmpty());
  EXPECT_THAT(child->contents(), SizeIs(1));
  EXPECT_EQ(child->GetPWNForId(id), 0);
}

INSTANTIATE_TEST_SUITE_P(, BSPTreePWN,
    testing::Combine(testing::Bool(), testing::Bool()));

}  // walnut
