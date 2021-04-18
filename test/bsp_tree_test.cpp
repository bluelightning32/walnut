#include "walnut/bsp_tree.h"

// For std::sort
#include <algorithm>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/convex_polygon_factory.h"

namespace walnut {

using testing::ContainerEq;
using testing::ElementsAre;
using testing::IsEmpty;
using testing::UnorderedElementsAreArray;

template <typename Polygon>
bool PolygonLt(const Polygon& a, const Polygon& b) {
  return std::lexicographical_compare(a.edges().begin(),
      a.edges().end(),
      b.edges().begin(), b.edges().end(),
      Polygon::EdgeRep::LexicographicallyLt);
}

template <typename Polygon>
void SortPolygons(std::vector<Polygon>& polygons) {
  for (Polygon& polygon : polygons) {
    polygon.SortVertices();
  }

  std::sort(polygons.begin(), polygons.end(), PolygonLt<Polygon>);
}

template<typename Container>
MutableConvexPolygon<> MakeUnsortedConvexPolygon(const Container& vertices) {
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
MutableConvexPolygon<> MakeConvexPolygon(const Container& vertices) {
  auto result = MakeUnsortedConvexPolygon(vertices);
  result.SortVertices();
  return result;
}

template <typename InputConvexPolygon>
std::vector<ConvexPolygon<>> DropVertexData(
    const std::vector<InputConvexPolygon> &input) {
  return std::vector<ConvexPolygon<>>(
      input.begin(), input.end());
}

TEST(BSPTree, AddContentsToLeaf) {
  Point3 triangles[][3] = {
    {
      Point3(0, 0, 10),
      Point3(1, 0, 10),
      Point3(1, 1, 10),
    },
    {
      Point3(0, 0, 11),
      Point3(1, 0, 11),
      Point3(1, 1, 11),
    },
  };

  BSPTree<> tree;
  auto leaf_added = [&](BSPNode<>& leaf) {
    EXPECT_EQ(&leaf, &tree.root);
  };
  std::vector<ConvexPolygon<>> polygons;
  for (Point3 (&triangle)[3] : triangles) {
    polygons.push_back(MakeConvexPolygon(triangle));
    tree.AddContent(MakeConvexPolygon(triangle), leaf_added);
  }
  EXPECT_THAT(DropVertexData(tree.root.contents()),
              ContainerEq(polygons));
}

TEST(BSPTree, PickSplitPlaneEmpty) {
  BSPTree<> tree;
  EXPECT_EQ(tree.root.PickSplitPlane(), nullptr);
}

TEST(BSPTree, PickSplitPlaneSinglePolygon) {
  Point3 triangle[3] =
  {
    Point3(0, 0, 10),
    Point3(1, 0, 10),
    Point3(1, 1, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(triangle);

  BSPTree<> tree;
  auto leaf_added = [&](BSPNode<>& leaf) {};
  tree.AddContent(polygon, leaf_added);
  EXPECT_TRUE(tree.root.PickSplitPlane()->IsSameOrOpposite(polygon.plane()));
}

TEST(BSPTree, PickSplitPlanePickLowestCount) {
  BSPTree<> tree;
  auto leaf_added = [&](BSPNode<>& leaf) {};
  BSPContentId common_id = tree.AllocateId();
  BSPContentId single_id = tree.AllocateId();
  HalfSpace3 expected_plane;
  for (int i = 0; i < 10; ++i) {
    Point3 triangle[3] =
    {
      Point3(0, 0, i),
      Point3(1, 0, i),
      Point3(1, 1, i),
    };

    ConvexPolygon<> polygon = MakeConvexPolygon(triangle);
    BSPContentId id;
    if (i == 3) {
      id = single_id;
      expected_plane = polygon.plane();
    } else {
      id = common_id;
    }
    tree.AddContent(id, polygon, leaf_added);
  }

  EXPECT_TRUE(tree.root.PickSplitPlane()->IsSameOrOpposite(expected_plane));
}

TEST(BSPTree, PickSplitPlaneSplitInTwo) {
  BSPTree<> tree;
  auto leaf_added = [&](BSPNode<>& leaf) {};
  BSPContentId id = tree.AllocateId();
  for (int i = 0; i < 2; ++i) {
    Point3 triangle[3] =
    {
      Point3(0, 0, i),
      Point3(1, 0, i),
      Point3(1, 1, i),
    };

    ConvexPolygon<> polygon = MakeConvexPolygon(triangle);
    tree.AddContent(id, polygon, leaf_added);
  }

  tree.root.Split(*tree.root.PickSplitPlane());
  EXPECT_TRUE(!tree.root.negative_child()->contents().empty() ||
              !tree.root.negative_child()->border_contents().empty() ||
              !tree.root.positive_child()->border_contents().empty());
  EXPECT_TRUE(!tree.root.positive_child()->contents().empty() ||
              !tree.root.negative_child()->border_contents().empty() ||
              !tree.root.positive_child()->border_contents().empty());
}

TEST(BSPTree, SplitTo1Child) {
  Point3 triangle[3] =
  {
    Point3(0, 0, 10),
    Point3(1, 0, 10),
    Point3(1, 1, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(triangle);

  std::vector<Point3> above_points;
  for (const Point3& p : triangle) {
    above_points.emplace_back(p.x(), p.y(), BigInt(p.z() + BigInt(1)));
  }
  HalfSpace3 above_up(above_points[0], above_points[1], above_points[2]);
  HalfSpace3 above_down = -above_up;

  {
    BSPTree<> tree;
    auto leaf_added = [&](BSPNode<>& leaf) {};
    tree.AddContent(polygon, leaf_added);

    tree.root.Split(above_up);
    EXPECT_FALSE(tree.root.IsLeaf());
    EXPECT_THAT(tree.root.contents(), IsEmpty());
    EXPECT_THAT(tree.root.border_contents(), IsEmpty());
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
    EXPECT_THAT(tree.root.contents(), IsEmpty());
    EXPECT_THAT(tree.root.border_contents(), IsEmpty());
    EXPECT_THAT(DropVertexData(tree.root.negative_child()->contents()),
                IsEmpty());
    EXPECT_THAT(DropVertexData(tree.root.positive_child()->contents()),
                ElementsAre(polygon));
  }
}

TEST(BSPTree, SplitOnPlane) {
  Point3 triangle[3] =
  {
    Point3(0, 0, 10),
    Point3(1, 0, 10),
    Point3(1, 1, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(triangle);

  {
    BSPTree<> tree;
    auto leaf_added = [&](BSPNode<>& leaf) {};
    tree.AddContent(polygon, leaf_added);

    tree.root.Split(polygon.plane());
    EXPECT_FALSE(tree.root.IsLeaf());
    ASSERT_THAT(DropVertexData(tree.root.negative_child()->border_contents()),
                ElementsAre(polygon));
    const auto& on_node_plane =
      tree.root.negative_child()->border_contents()[0].on_node_plane;
    EXPECT_EQ(on_node_plane.split, &tree.root.split());
    EXPECT_FALSE(on_node_plane.pos_side);
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
    ASSERT_THAT(DropVertexData(tree.root.positive_child()->border_contents()),
                ElementsAre(polygon));
    const auto& on_node_plane =
      tree.root.positive_child()->border_contents()[0].on_node_plane;
    EXPECT_EQ(on_node_plane.split, &tree.root.split());
    EXPECT_TRUE(on_node_plane.pos_side);
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
  Point3 p[4] = {
    Point3(0, 0, 10),
    Point3(2, 0, 10),
    Point3(2, 1, 10),
    Point3(0, 1, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(p);
  HalfSpace3 half_space(/*x=*/1, /*y=*/0, /*z=*/0, /*dist=*/1);

  Point3 expected_neg[4] = {
    Point3(0, 0, 10),
    Point3(1, 0, 10),
    Point3(1, 1, 10),
    Point3(0, 1, 10),
  };

  Point3 expected_pos[4] = {
    Point3(1, 0, 10),
    Point3(2, 0, 10),
    Point3(2, 1, 10),
    Point3(1, 1, 10),
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

  for (const BSPNode<>::EdgeRep& edge :
       tree.root.negative_child()->contents()[0].edges()) {
    if (edge.vertex() == expected_neg[1]) {
      EXPECT_EQ(edge.edge_first_coincident.split, &tree.root.split());
      EXPECT_FALSE(edge.edge_first_coincident.pos_side);
    } else {
      EXPECT_EQ(edge.edge_first_coincident.split, nullptr);
    }
  }
  for (const BSPNode<>::EdgeRep& edge :
       tree.root.positive_child()->contents()[0].edges()) {
    if (edge.vertex() == expected_pos[3]) {
      EXPECT_EQ(edge.edge_first_coincident.split, &tree.root.split());
      EXPECT_TRUE(edge.edge_first_coincident.pos_side);
    } else {
      EXPECT_EQ(edge.edge_first_coincident.split, nullptr);
    }
  }
}

TEST(BSPTree, SplitTwiceVertexData) {
  //
  // p[3] <-------q3--------- p[2]
  //  |           | pos_child2 ^
  //  |           |            |
  //  |           |       ^    |
  //  |           | split2|    |
  //  |           | pos   |    |
  //  |           q2----------q1
  //  |           | neg_child2 |
  //  |           |            |
  //  |           |split1      |
  //  |           |pos->       |
  //  |           |            |
  //  v           |            |
  // p[0] --------q4--------> p[1]
  //
  // The edge of neg_child2 that starts at q2 is split by split1.
  // The edge of neg_child2 that starts at q1 is split by split2.
  // The edge of pos_child2 that starts at q3 is split by split1.
  // The edge of pos_child2 that starts at q2 is split by split2.
  //
  // neg_child2 last split plane touched at each vertex:
  // q1: split2.normal()
  // q2: split2.normal()
  // q4: -split1.normal()
  //
  // pos_child2 last split plane touched at each vertex:
  // q1: -split2.normal()
  // q2: -split2.normal()
  // q3: -split1.normal()
  Point3 p[4] = {
    Point3(0, 0, 10),
    Point3(2, 0, 10),
    Point3(2, 2, 10),
    Point3(0, 2, 10),
  };

  Point3 q1(2, 1, 10);
  Point3 q2(1, 1, 10);
  Point3 q3(1, 2, 10);
  Point3 q4(1, 0, 10);

  ConvexPolygon<> polygon = MakeConvexPolygon(p);
  HalfSpace3 split1(/*x=*/1, /*y=*/0, /*z=*/0, /*dist=*/1);
  HalfSpace3 split2(/*x=*/0, /*y=*/1, /*z=*/0, /*dist=*/1);

  BSPTree<> tree;
  auto leaf_added = [&](BSPNode<>& leaf) {};
  tree.AddContent(polygon, leaf_added);

  tree.root.Split(split1);
  BSPNode<>& pos_child1 = *tree.root.positive_child();
  EXPECT_EQ(pos_child1.contents().size(), 1);

  pos_child1.Split(split2);
  BSPNode<>& neg_child2 = *pos_child1.negative_child();
  BSPNode<>& pos_child2 = *pos_child1.positive_child();
  ASSERT_EQ(neg_child2.contents().size(), 1);
  ASSERT_EQ(pos_child2.contents().size(), 1);

  // Validate neg_child2.
  for (const BSPNode<>::EdgeRep& edge :
       neg_child2.contents()[0].edges()) {
    if (edge.vertex() == q2) {
      EXPECT_EQ(edge.edge_first_coincident.split, &tree.root.split());
      EXPECT_TRUE(edge.edge_first_coincident.pos_side);
      EXPECT_EQ(edge.edge_last_coincident.split, &tree.root.split());
      EXPECT_TRUE(edge.edge_last_coincident.pos_side);
      EXPECT_EQ(edge.vertex_last_coincident.split, &pos_child1.split());
      EXPECT_FALSE(edge.vertex_last_coincident.pos_side);
    } else if (edge.vertex() == q1) {
      EXPECT_EQ(edge.edge_first_coincident.split, &pos_child1.split());
      EXPECT_FALSE(edge.edge_first_coincident.pos_side);
      EXPECT_EQ(edge.edge_last_coincident.split, &pos_child1.split());
      EXPECT_FALSE(edge.edge_last_coincident.pos_side);
      EXPECT_EQ(edge.vertex_last_coincident.split, &pos_child1.split());
      EXPECT_FALSE(edge.vertex_last_coincident.pos_side);
    } else if (edge.vertex() == q4) {
      EXPECT_EQ(edge.edge_first_coincident.split, nullptr);
      EXPECT_EQ(edge.edge_last_coincident.split, nullptr);
      EXPECT_EQ(edge.vertex_last_coincident.split, &tree.root.split());
      EXPECT_TRUE(edge.vertex_last_coincident.pos_side);
    } else {
      EXPECT_EQ(edge.edge_first_coincident.split, nullptr);
      EXPECT_EQ(edge.edge_last_coincident.split, nullptr);
      EXPECT_EQ(edge.vertex_last_coincident.split, nullptr);
    }
  }
  // Validate pos_child2.
  for (const BSPNode<>::EdgeRep& edge :
       pos_child2.contents()[0].edges()) {
    if (edge.vertex() == q3) {
      EXPECT_EQ(edge.edge_first_coincident.split, &tree.root.split());
      EXPECT_TRUE(edge.edge_first_coincident.pos_side);
      EXPECT_EQ(edge.edge_last_coincident.split, &tree.root.split());
      EXPECT_TRUE(edge.edge_last_coincident.pos_side);
      EXPECT_EQ(edge.vertex_last_coincident.split, &tree.root.split());
      EXPECT_TRUE(edge.vertex_last_coincident.pos_side);
    } else if (edge.vertex() == q2) {
      EXPECT_EQ(edge.edge_first_coincident.split, &pos_child1.split());
      EXPECT_TRUE(edge.edge_first_coincident.pos_side);
      EXPECT_EQ(edge.edge_last_coincident.split, &pos_child1.split());
      EXPECT_TRUE(edge.edge_last_coincident.pos_side);
      EXPECT_EQ(edge.vertex_last_coincident.split, &pos_child1.split());
      EXPECT_TRUE(edge.vertex_last_coincident.pos_side);
    } else if (edge.vertex() == q1) {
      EXPECT_EQ(edge.edge_first_coincident.split, nullptr);
      EXPECT_EQ(edge.edge_last_coincident.split, nullptr);
      EXPECT_EQ(edge.vertex_last_coincident.split, &pos_child1.split());
      EXPECT_TRUE(edge.vertex_last_coincident.pos_side);
    } else {
      EXPECT_EQ(edge.edge_first_coincident.split, nullptr);
      EXPECT_EQ(edge.edge_last_coincident.split, nullptr);
      EXPECT_EQ(edge.vertex_last_coincident.split, nullptr);
    }
  }
}

TEST(BSPTree, SplitVertThenDiagVertexData) {
  //
  // p[3] <----q1----------- p[2]
  //  |        | neg_child2/  ^
  //  |        |          /   |
  //  |        |split1   /    |
  //  |        |pos->   /     |
  //  |        |       /      |
  //  |        |      /       |
  //  |        |     /split2  |
  //  |        |    /pos      |
  //  |        |   /    \     |
  //  |        |  /      v    |
  //  |        | /            |
  //  v        |/  pos_child2 |
  // p[0] -----q2----------> p[1]
  //
  // The edge of neg_child2 that starts at q1 is split by split1.
  // The edge of neg_child2 that starts at q2 is split by split2.
  // The edge of pos_child2 that starts at p[2] is split by split2.
  //
  // neg_child2 last split plane touched at each vertex:
  // p[2]: split2.normal()
  // q1: -split1.normal()
  // q2: split2.normal()
  //
  // pos_child2 most clockwise split plane touched by each vertex:
  // p[2]: -split2.normal()
  // q2: -split2.normal()
  //
  Point3 p[4] = {
    Point3(0, 0, 10),
    Point3(2, 0, 10),
    Point3(2, 1, 10),
    Point3(0, 1, 10),
  };

  Point3 q1(1, 1, 10);
  Point3 q2(1, 0, 10);

  ConvexPolygon<> polygon = MakeConvexPolygon(p);
  HalfSpace3 split1(/*x=*/1, /*y=*/0, /*z=*/0, /*dist=*/1);
  HalfSpace3 split2(/*x=*/1, /*y=*/-1, /*z=*/0,
                      /*dist=*/q1.x().ToInt() - q2.y().ToInt());

  BSPTree<> tree;
  auto leaf_added = [&](BSPNode<>& leaf) {};
  tree.AddContent(polygon, leaf_added);

  tree.root.Split(split1);
  BSPNode<>& pos_child1 = *tree.root.positive_child();
  EXPECT_EQ(pos_child1.contents().size(), 1);

  pos_child1.Split(split2);
  BSPNode<>& neg_child2 = *pos_child1.negative_child();
  BSPNode<>& pos_child2 = *pos_child1.positive_child();
  ASSERT_EQ(neg_child2.contents().size(), 1);
  ASSERT_EQ(pos_child2.contents().size(), 1);

  // Validate neg_child2.
  EXPECT_EQ(neg_child2.contents()[0].vertex_count(), 3);
  for (const BSPNode<>::EdgeRep& edge :
       neg_child2.contents()[0].edges()) {
    if (edge.vertex() == q1) {
      EXPECT_EQ(edge.edge_first_coincident.split, &tree.root.split());
      EXPECT_TRUE(edge.edge_first_coincident.pos_side);
      EXPECT_EQ(edge.edge_last_coincident.split, &tree.root.split());
      EXPECT_TRUE(edge.edge_last_coincident.pos_side);
      EXPECT_EQ(edge.vertex_last_coincident.split, &tree.root.split());
      EXPECT_TRUE(edge.vertex_last_coincident.pos_side);
    } else if (edge.vertex() == q2) {
      EXPECT_EQ(edge.edge_first_coincident.split, &pos_child1.split());
      EXPECT_FALSE(edge.edge_first_coincident.pos_side);
      EXPECT_EQ(edge.edge_last_coincident.split, &pos_child1.split());
      EXPECT_FALSE(edge.edge_last_coincident.pos_side);
      EXPECT_EQ(edge.vertex_last_coincident.split, &pos_child1.split());
      EXPECT_FALSE(edge.vertex_last_coincident.pos_side);
    } else {
      EXPECT_EQ(edge.edge_first_coincident.split, nullptr);
      EXPECT_EQ(edge.edge_last_coincident.split, nullptr);
      EXPECT_EQ(edge.vertex_last_coincident.split, &pos_child1.split());
      EXPECT_FALSE(edge.vertex_last_coincident.pos_side);
    }
  }
  // Validate pos_child2.
  EXPECT_EQ(pos_child2.contents()[0].vertex_count(), 3);
  for (const BSPNode<>::EdgeRep& edge :
       pos_child2.contents()[0].edges()) {
    if (edge.vertex() == p[2]) {
      EXPECT_EQ(edge.edge_first_coincident.split, &pos_child1.split());
      EXPECT_TRUE(edge.edge_first_coincident.pos_side);
      EXPECT_EQ(edge.edge_last_coincident.split, &pos_child1.split());
      EXPECT_TRUE(edge.edge_last_coincident.pos_side);
      EXPECT_EQ(edge.vertex_last_coincident.split, &pos_child1.split());
      EXPECT_TRUE(edge.vertex_last_coincident.pos_side);
    } else if (edge.vertex() == q2) {
      EXPECT_EQ(edge.edge_first_coincident.split, nullptr);
      EXPECT_EQ(edge.edge_last_coincident.split, nullptr);
      EXPECT_EQ(edge.vertex_last_coincident.split, &pos_child1.split());
      EXPECT_TRUE(edge.vertex_last_coincident.pos_side);
    } else {
      EXPECT_EQ(edge.edge_first_coincident.split, nullptr);
      EXPECT_EQ(edge.edge_last_coincident.split, nullptr);
      EXPECT_EQ(edge.vertex_last_coincident.split, nullptr);
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
  Point3 p[4] = {
    Point3(0, 0, 10),
    Point3(2, 0, 10),
    Point3(2, 1, 10),
    Point3(0, 1, 10),
  };

  ConvexPolygon<> polygon = MakeConvexPolygon(p);
  HalfSpace3 half_space(/*x=*/1, /*y=*/0, /*z=*/0, /*dist=*/1);

  Point3 expected_neg[4] = {
    Point3(0, 0, 10),
    Point3(1, 0, 10),
    Point3(1, 1, 10),
    Point3(0, 1, 10),
  };

  Point3 expected_pos[4] = {
    Point3(1, 0, 10),
    Point3(2, 0, 10),
    Point3(2, 1, 10),
    Point3(1, 1, 10),
  };

  BSPTree<> tree;
  auto leaf_added = [&](BSPNode<>& leaf) {};
  tree.AddContent(polygon, leaf_added);

  // Split the root such that the polygon becomes a border polygon of the
  // negative child.
  tree.root.Split(polygon.plane());
  ASSERT_FALSE(tree.root.IsLeaf());
  ASSERT_THAT(DropVertexData(tree.root.negative_child()->border_contents()),
              ElementsAre(polygon));
  EXPECT_EQ(
      tree.root.negative_child()->border_contents()[0].on_node_plane.split,
      &tree.root.split());
  EXPECT_FALSE(
      tree.root.negative_child()->border_contents()[0].on_node_plane.pos_side);

  for (const BSPNode<>::EdgeRep& edge :
       tree.root.negative_child()->border_contents()[0].edges()) {
    EXPECT_EQ(edge.edge_last_coincident.split, &tree.root.split());
    EXPECT_EQ(edge.edge_first_coincident.split, &tree.root.split());
  }

  // Split the negative child such that the polygon is split into 2 pieces.
  tree.root.negative_child()->Split(half_space);
  ASSERT_FALSE(tree.root.negative_child()->IsLeaf());
  EXPECT_THAT(tree.root.negative_child()->contents(), IsEmpty());
  EXPECT_THAT(tree.root.negative_child()->border_contents(), IsEmpty());

  const BSPTree<>::BSPNodeRep* neg_leaf =
    tree.root.negative_child()->negative_child();
  const BSPTree<>::BSPNodeRep* pos_leaf =
    tree.root.negative_child()->positive_child();
  ASSERT_NE(neg_leaf, nullptr);
  ASSERT_NE(pos_leaf, nullptr);
  ASSERT_EQ(neg_leaf->border_contents().size(), 1);
  ASSERT_EQ(pos_leaf->border_contents().size(), 1);

  EXPECT_EQ(neg_leaf->border_contents()[0], MakeConvexPolygon(expected_neg));
  EXPECT_EQ(neg_leaf->border_contents()[0].on_node_plane.split,
            &tree.root.split());
  EXPECT_FALSE(neg_leaf->border_contents()[0].on_node_plane.pos_side);
  EXPECT_EQ(pos_leaf->border_contents()[0], MakeConvexPolygon(expected_pos));
  EXPECT_EQ(pos_leaf->border_contents()[0].on_node_plane.split,
            &tree.root.split());
  EXPECT_FALSE(pos_leaf->border_contents()[0].on_node_plane.pos_side);

  for (const BSPNode<>::EdgeRep& edge :
       neg_leaf->border_contents()[0].edges()) {
    if (edge.vertex() == expected_neg[1]) {
      EXPECT_EQ(edge.edge_last_coincident.split,
                &tree.root.negative_child()->split());
    } else {
      EXPECT_EQ(edge.edge_last_coincident.split, &tree.root.split());
    }
    EXPECT_EQ(edge.edge_first_coincident.split, &tree.root.split());
  }
  for (const BSPNode<>::EdgeRep& edge :
       pos_leaf->border_contents()[0].edges()) {
    if (edge.vertex() == expected_pos[3]) {
      EXPECT_EQ(edge.edge_last_coincident.split,
                &tree.root.negative_child()->split());
    } else {
      EXPECT_EQ(edge.edge_last_coincident.split, &tree.root.split());
    }
    EXPECT_EQ(edge.edge_first_coincident.split, &tree.root.split());
  }
}

TEST(BSPTree, GetNodeBorderEmptyTree) {
  BSPTree<> tree;

  AABB bounding_box(Point3(-1, -1, -1), Point3(2, 2, 2));
  std::vector<bool> node_path = { };

  BSPTree<>::MappedBSPNode node_border_root;
  BSPTree<>::MappedBSPNode* node_border_leaf =
    tree.GetNodeBorder(node_path.begin(), node_path.end(), bounding_box,
                       node_border_root);
  std::vector<BSPTree<>::OutputPolygon> facets = node_border_leaf->contents();

  Point3 p[] = {
    Point3{-1, -1, -1},
    Point3{ 2, -1, -1},
    Point3{ 2,  2, -1},
    Point3{-1,  2, -1},
    Point3{-1, -1,  2},
    Point3{ 2, -1,  2},
    Point3{ 2,  2,  2},
    Point3{-1,  2,  2},
  };

  std::vector<std::vector<Point3>> expected_facet_vertices = {
    // bottom
    {p[0], p[3], p[2], p[1]},
    // min x side
    {p[3], p[0], p[4], p[7]},
    // min y side
    {p[0], p[1], p[5], p[4]},
    // max x side
    {p[1], p[2], p[6], p[5]},
    // max y side
    {p[2], p[3], p[7], p[6]},
    // top
    {p[4], p[5], p[6], p[7]},
  };
  std::vector<MutableConvexPolygon<>> expected_facets;
  for (const std::vector<Point3>& vertices : expected_facet_vertices) {
    expected_facets.push_back(MakeConvexPolygon(vertices));
  }
  SortPolygons(expected_facets);
  SortPolygons(facets);
  ASSERT_EQ(facets.size(), expected_facets.size());
  for (size_t i = 0; i < facets.size(); ++i) {
    EXPECT_EQ(facets[i], expected_facets[i])
      << "i=" << i << std::endl
      << "facets[i]=" << facets[i] << std::endl
      << "facets[i].plane=" << facets[i].plane() << std::endl
      << "expected_facets[i].plane=" << expected_facets[i].plane();
  }
}

TEST(BSPTree, GetNodeBorder1Split) {
  Point3 p[] = {
    Point3{-1, -1, -1},
    Point3{ 2, -1, -1},
    Point3{ 2,  2, -1},
    Point3{-1,  2, -1},
    Point3{-1, -1,  2},
    Point3{ 2, -1,  2},
    Point3{ 2,  2,  2},
    Point3{-1,  2,  2},
  };

  for (bool pos_side : {false, true}) {
    BSPTree<> tree;
    HalfSpace3 split(p[1], p[3], p[4]);
    tree.root.Split(pos_side ? -split : split);
    std::vector<bool> node_path = {pos_side};
    AABB bounding_box(Point3(-1, -1, -1), Point3(2, 2, 2));

    BSPTree<>::MappedBSPNode node_border_root;
    BSPTree<>::MappedBSPNode* node_border_leaf =
      tree.GetNodeBorder(node_path.begin(), node_path.end(), bounding_box,
                         node_border_root);
    std::vector<BSPTree<>::OutputPolygon> facets = node_border_leaf->contents();
    facets.insert(facets.end(), node_border_leaf->border_contents().begin(),
                  node_border_leaf->border_contents().end());

    std::vector<std::vector<Point3>> expected_facet_vertices = {
      // bottom
      {p[0], p[3], p[1]},
      // min x side
      {p[0], p[4], p[3]},
      // min y side
      {p[0], p[1], p[4]},
      // diag
      {p[1], p[3], p[4]},
    };
    std::vector<MutableConvexPolygon<>> expected_facets;
    for (const std::vector<Point3>& vertices : expected_facet_vertices) {
      expected_facets.push_back(MakeConvexPolygon(vertices));
    }
    SortPolygons(expected_facets);
    SortPolygons(facets);
    ASSERT_EQ(facets.size(), expected_facets.size())
      << "pos_side=" << pos_side;
    for (size_t i = 0; i < facets.size(); ++i) {
      EXPECT_EQ(facets[i], expected_facets[i])
        << "i=" << i << std::endl
        << "facets[i]=" << facets[i] << std::endl
        << "facets[i].plane=" << facets[i].plane() << std::endl
        << "expected_facets[i].plane=" << expected_facets[i].plane();
    }
  }
}

}  // walnut
