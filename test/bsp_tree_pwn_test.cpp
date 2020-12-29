#include "walnut/bsp_tree.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/convex_polygon_factory.h"

namespace walnut {

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

// Splits `root` 6 times to form all sides of the tilted cube. The descendant
// node for the inside of the cube is returned.
BSPNode<>* SplitToCube(BSPNode<>& root, bool make_pos_child = true) {
  EXPECT_TRUE(root.IsLeaf());

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

  BSPNode<>* pos = &root;
  for (BSPNode<>::HalfSpace3Rep& split_plane : split_planes) {
    if (make_pos_child) {
      pos->Split(-split_plane);
      pos = pos->positive_child();
    } else { 
      pos->Split(split_plane);
      pos = pos->negative_child();
    }
  }
  return pos;
}

TEST(BSPTreePWN, EmptyCube) {
  BSPTree<> tree;
  auto leaf_added = [&](BSPNode<>& leaf) {
    EXPECT_EQ(&leaf, &tree.root);
  };

  BSPPolygonId id = tree.AllocateId();
  EXPECT_EQ(id, 0);

  BSPNode<>* inside_cube = SplitToCube(tree.root, /*make_pos_child=*/false);
  EXPECT_EQ(inside_cube->GetPWNForId(id), 0);
}

}  // walnut
