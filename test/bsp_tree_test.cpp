#include "walnut/bsp_tree.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/convex_polygon_factory.h"

namespace walnut {

using testing::ContainerEq;
using testing::ElementsAre;
using testing::IsEmpty;

template<typename Container>
auto
MakeUnsortedConvexPolygon(const Container& vertices) ->
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


template<typename Container>
auto
MakeConvexPolygon(const Container& vertices) ->
ConvexPolygon<std::iterator_traits<
    decltype(std::begin(vertices))>::value_type::component_bits> {
  auto result = MakeUnsortedConvexPolygon(vertices);
  result.SortVertices();
  return result;
}

template <int point3_bits, typename VertexData>
std::vector<ConvexPolygon<point3_bits>> DropVertexData(
    const std::vector<ConvexPolygon<point3_bits, VertexData>> &input) {
  return std::vector<ConvexPolygon<point3_bits>>(input.begin(), input.end());
}

TEST(BSPTree, AddContentsToLeaf) {
  Point3<32> triangles[][3] = {
    {
      Point3<32>(0, 0, 10),
      Point3<32>(1, 0, 10),
      Point3<32>(1, 1, 10),
    },
    {
      Point3<32>(0, 0, 11),
      Point3<32>(1, 0, 11),
      Point3<32>(1, 1, 11),
    },
  };

  std::vector<ConvexPolygon<>> polygons;
  for (Point3<32> (&triangle)[3] : triangles) {
    polygons.push_back(MakeConvexPolygon(triangle));
  }

  BSPTree<> tree;
  auto leaf_added = [&](BSPNode<>& leaf) {
    EXPECT_EQ(&leaf, &tree.root);
  };
  tree.AddContents(polygons.begin(), polygons.end(), leaf_added);
  EXPECT_THAT(DropVertexData(tree.root.contents()),
              ContainerEq(polygons));
}

TEST(BSPTree, SplitTo1Child) {
  Point3<32> triangle[3] =
  {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10),
    Point3<32>(1, 1, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(triangle);

  std::vector<Point3<32>> above_points;
  for (const Point3<32>& p : triangle) {
    above_points.emplace_back(p.x(), p.y(), BigInt<32>(p.z() + BigInt<32>(1)));
  }
  HalfSpace3<> above_up(above_points[0], above_points[1], above_points[2]);
  HalfSpace3<> above_down = -above_up;

  {
    BSPTree<> tree;
    auto leaf_added = [&](BSPNode<>& leaf) {};
    tree.AddContent(polygon, leaf_added);

    tree.root.Split(above_up);
    EXPECT_FALSE(tree.root.IsLeaf());
    EXPECT_THAT(DropVertexData(tree.root.negative_child()->contents()),
                ElementsAre(polygon));
    EXPECT_THAT(DropVertexData(tree.root.positive_child()->contents()),
                IsEmpty());
  }

  {
    BSPTree<> tree;
    auto leaf_added = [&](BSPNode<>& leaf) {};
    tree.AddContent(polygon, leaf_added);

    tree.root.Split(above_down);
    EXPECT_FALSE(tree.root.IsLeaf());
    EXPECT_THAT(DropVertexData(tree.root.negative_child()->contents()),
                IsEmpty());
    EXPECT_THAT(DropVertexData(tree.root.positive_child()->contents()),
                ElementsAre(polygon));
  }
}

TEST(BSPTree, SplitOnPlane) {
  Point3<32> triangle[3] =
  {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10),
    Point3<32>(1, 1, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(triangle);

  {
    BSPTree<> tree;
    auto leaf_added = [&](BSPNode<>& leaf) {};
    tree.AddContent(polygon, leaf_added);

    tree.root.Split(polygon.plane());
    EXPECT_FALSE(tree.root.IsLeaf());
    EXPECT_THAT(DropVertexData(tree.root.negative_child()->border_contents()),
                ElementsAre(polygon));
    EXPECT_THAT(DropVertexData(tree.root.positive_child()->border_contents()),
                IsEmpty());
    EXPECT_THAT(DropVertexData(tree.root.negative_child()->contents()),
                IsEmpty());
    EXPECT_THAT(DropVertexData(tree.root.positive_child()->contents()),
                IsEmpty());
  }

  {
    BSPTree<> tree;
    auto leaf_added = [&](BSPNode<>& leaf) {};
    tree.AddContent(polygon, leaf_added);

    tree.root.Split(-polygon.plane());
    EXPECT_FALSE(tree.root.IsLeaf());
    EXPECT_THAT(DropVertexData(tree.root.negative_child()->border_contents()),
                IsEmpty());
    EXPECT_THAT(DropVertexData(tree.root.positive_child()->border_contents()),
                ElementsAre(polygon));
    EXPECT_THAT(DropVertexData(tree.root.negative_child()->contents()),
                IsEmpty());
    EXPECT_THAT(DropVertexData(tree.root.positive_child()->contents()),
                IsEmpty());
  }
}

TEST(BSPTree, SplitTo2Children) {
  //
  // p[3] <--------- p[2]
  //  |       |       ^
  //  |       |pos->  |
  //  v       |       |
  // p[0] ---------> p[1]
  //
  Point3<32> p[4] = {
    Point3<32>(0, 0, 10),
    Point3<32>(2, 0, 10),
    Point3<32>(2, 1, 10),
    Point3<32>(0, 1, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(p);
  HalfSpace3<> half_space(/*x=*/1, /*y=*/0, /*z=*/0, /*dist=*/1);

  Point3<32> expected_neg[4] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(0, 1, 10),
  };

  Point3<32> expected_pos[4] = {
    Point3<32>(1, 0, 10),
    Point3<32>(2, 0, 10),
    Point3<32>(2, 1, 10),
    Point3<32>(1, 1, 10),
  };

  BSPTree<> tree;
  auto leaf_added = [&](BSPNode<>& leaf) {};
  tree.AddContent(polygon, leaf_added);

  tree.root.Split(half_space);
  EXPECT_FALSE(tree.root.IsLeaf());
  ASSERT_EQ(tree.root.negative_child()->contents().size(), 1);
  ASSERT_EQ(tree.root.positive_child()->contents().size(), 1);

  EXPECT_EQ(tree.root.negative_child()->contents()[0],
            MakeConvexPolygon(expected_neg));
  EXPECT_EQ(tree.root.positive_child()->contents()[0],
            MakeConvexPolygon(expected_pos));

  for (const BSPNode<>::ConvexPolygonRep::EdgeRep& edge :
       tree.root.negative_child()->contents()[0].edges()) {
    if (edge.vertex == expected_neg[1]) {
      EXPECT_EQ(edge.data().split_by(), &tree.root);
    } else {
      EXPECT_EQ(edge.data().split_by(), nullptr);
    }
  }
  for (const BSPNode<>::ConvexPolygonRep::EdgeRep& edge :
       tree.root.positive_child()->contents()[0].edges()) {
    if (edge.vertex == expected_pos[3]) {
      EXPECT_EQ(edge.data().split_by(), &tree.root);
    } else {
      EXPECT_EQ(edge.data().split_by(), nullptr);
    }
  }
}

TEST(BSPTree, SplitBorderTo2Children) {
  //
  // p[3] <--------- p[2]
  //  |       |       ^
  //  |       |pos->  |
  //  v       |       |
  // p[0] ---------> p[1]
  //
  Point3<32> p[4] = {
    Point3<32>(0, 0, 10),
    Point3<32>(2, 0, 10),
    Point3<32>(2, 1, 10),
    Point3<32>(0, 1, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(p);
  HalfSpace3<> half_space(/*x=*/1, /*y=*/0, /*z=*/0, /*dist=*/1);

  Point3<32> expected_neg[4] = {
    Point3<32>(0, 0, 10),
    Point3<32>(1, 0, 10),
    Point3<32>(1, 1, 10),
    Point3<32>(0, 1, 10),
  };

  Point3<32> expected_pos[4] = {
    Point3<32>(1, 0, 10),
    Point3<32>(2, 0, 10),
    Point3<32>(2, 1, 10),
    Point3<32>(1, 1, 10),
  };

  BSPTree<> tree;
  auto leaf_added = [&](BSPNode<>& leaf) {};
  tree.AddContent(polygon, leaf_added);

  // Split the root such that the polygon becomes a border polygon of the
  // negative child.
  tree.root.Split(polygon.plane());
  ASSERT_FALSE(tree.root.IsLeaf());
  EXPECT_THAT(DropVertexData(tree.root.negative_child()->border_contents()),
              ElementsAre(polygon));

  // Split the negative child such that the polygon is split into 2 pieces.
  tree.root.negative_child()->Split(half_space);
  ASSERT_FALSE(tree.root.negative_child()->IsLeaf());

  const BSPTree<>::BSPNodeRep* neg_leaf =
    tree.root.negative_child()->negative_child();
  const BSPTree<>::BSPNodeRep* pos_leaf =
    tree.root.negative_child()->positive_child();
  ASSERT_NE(neg_leaf, nullptr);
  ASSERT_NE(pos_leaf, nullptr);
  ASSERT_EQ(neg_leaf->border_contents().size(), 1);
  ASSERT_EQ(pos_leaf->border_contents().size(), 1);

  EXPECT_EQ(neg_leaf->border_contents()[0], MakeConvexPolygon(expected_neg));
  EXPECT_EQ(pos_leaf->border_contents()[0], MakeConvexPolygon(expected_pos));

  for (const BSPNode<>::ConvexPolygonRep::EdgeRep& edge :
       neg_leaf->border_contents()[0].edges()) {
    if (edge.vertex == expected_neg[1]) {
      EXPECT_EQ(edge.data().split_by(), tree.root.negative_child());
    } else {
      EXPECT_EQ(edge.data().split_by(), nullptr);
    }
  }
  for (const BSPNode<>::ConvexPolygonRep::EdgeRep& edge :
       pos_leaf->border_contents()[0].edges()) {
    if (edge.vertex == expected_pos[3]) {
      EXPECT_EQ(edge.data().split_by(), tree.root.negative_child());
    } else {
      EXPECT_EQ(edge.data().split_by(), nullptr);
    }
  }
}

}  // walnut
