#include "walnut/convex_polygon_vertex_iterator.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/convex_polygon_edge.h"

namespace walnut {

TEST(ConvexPolygonVertexIterator, CopyConstructible) {
  std::vector<ConvexPolygonEdge<>> edges;

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  const ConvexPolygonVertexIterator<VectorIterator> it1(edges.begin());
  ConvexPolygonVertexIterator<VectorIterator> it2(it1);
  EXPECT_EQ(it1, it2);
}

TEST(ConvexPolygonVertexIterator, ConstructConstFromNonConst) {
  std::vector<ConvexPolygonEdge<>> edges;

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  using ConstVectorIterator = std::vector<ConvexPolygonEdge<>>::const_iterator;
  const ConvexPolygonVertexIterator<VectorIterator> it1(edges.begin());
  ConvexPolygonVertexIterator<ConstVectorIterator> it2(it1);
  EXPECT_EQ(it1, it2);
}

TEST(ConvexPolygonVertexIterator, Assign) {
  std::vector<ConvexPolygonEdge<>> edges;

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  ConvexPolygonVertexIterator<VectorIterator> it1(edges.begin());
  ConvexPolygonVertexIterator<VectorIterator> it2;
  it2 = it1;
  EXPECT_EQ(it1, it2);
}

TEST(ConvexPolygonVertexIterator, AssignConstFromNonConst) {
  std::vector<ConvexPolygonEdge<>> edges;

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  using ConstVectorIterator = std::vector<ConvexPolygonEdge<>>::const_iterator;
  ConvexPolygonVertexIterator<VectorIterator> it1(edges.begin());
  ConvexPolygonVertexIterator<ConstVectorIterator> it2;
  it2 = it1;
  EXPECT_EQ(it1, it2);
}

TEST(ConvexPolygonVertexIterator, Swap) {
  std::vector<ConvexPolygonEdge<>> edges{
    {Point3<>(0, 0, 0), Point3<>(1, 1, 1)},
    {Point3<>(1, 1, 1), Point3<>(0, 0, 0)},
  };

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  using VertexIterator = ConvexPolygonVertexIterator<VectorIterator>;
  VertexIterator it1(edges.begin() + 0);
  VertexIterator it2(edges.begin() + 1);

  using std::swap;
  swap(it1, it2);
  EXPECT_EQ(it1, VertexIterator(edges.begin() + 1));
  EXPECT_EQ(it2, VertexIterator(edges.begin() + 0));
}

TEST(ConvexPolygonVertexIterator, IteratorCategory) {
  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  using VertexIterator = ConvexPolygonVertexIterator<VectorIterator>;
  using VectorIteratorCategory =
    std::iterator_traits<VectorIterator>::iterator_category;
  using VertexIteratorCategory =
    std::iterator_traits<VertexIterator>::iterator_category;
  EXPECT_TRUE((std::is_same<VectorIteratorCategory,
                            VertexIteratorCategory>::value));
}

TEST(ConvexPolygonVertexIterator, Dereference) {
  std::vector<ConvexPolygonEdge<>> edges{
    {Point3<>(0, 0, 0), Point3<>(1, 1, 1)},
  };

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  using VertexIterator = ConvexPolygonVertexIterator<VectorIterator>;
  const VertexIterator it(edges.begin());

  EXPECT_EQ(&*it, &edges.begin()->vertex);
}

TEST(ConvexPolygonVertexIterator, PrefixIncrement) {
  std::vector<ConvexPolygonEdge<>> edges{
    {Point3<>(0, 0, 0), Point3<>(1, 1, 1)},
    {Point3<>(1, 1, 1), Point3<>(0, 0, 0)},
  };

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  using VertexIterator = ConvexPolygonVertexIterator<VectorIterator>;
  VertexIterator it1(edges.begin() + 0);
  VertexIterator it2(edges.begin() + 1);

  EXPECT_EQ(&(++it1), &it1);
  EXPECT_EQ(it1, it2);
}

TEST(ConvexPolygonVertexIterator, PostfixIncrement) {
  std::vector<ConvexPolygonEdge<>> edges{
    {Point3<>(0, 0, 0), Point3<>(1, 1, 1)},
    {Point3<>(1, 1, 1), Point3<>(0, 0, 0)},
  };

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  using VertexIterator = ConvexPolygonVertexIterator<VectorIterator>;
  VertexIterator it1(edges.begin() + 0);
  VertexIterator it2(edges.begin() + 1);

  VertexIterator it(it1);
  EXPECT_EQ(it++, it1);
  EXPECT_EQ(it, it2);
}

TEST(ConvexPolygonVertexIterator, PrefixDecrement) {
  std::vector<ConvexPolygonEdge<>> edges{
    {Point3<>(0, 0, 0), Point3<>(1, 1, 1)},
    {Point3<>(1, 1, 1), Point3<>(0, 0, 0)},
  };

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  using VertexIterator = ConvexPolygonVertexIterator<VectorIterator>;
  VertexIterator it1(edges.begin() + 0);
  VertexIterator it2(edges.begin() + 1);

  EXPECT_EQ(&(--it2), &it2);
  EXPECT_EQ(it2, it1);
}

TEST(ConvexPolygonVertexIterator, PostfixDecrement) {
  std::vector<ConvexPolygonEdge<>> edges{
    {Point3<>(0, 0, 0), Point3<>(1, 1, 1)},
    {Point3<>(1, 1, 1), Point3<>(0, 0, 0)},
  };

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  using VertexIterator = ConvexPolygonVertexIterator<VectorIterator>;
  VertexIterator it1(edges.begin() + 0);
  VertexIterator it2(edges.begin() + 1);

  VertexIterator it(it2);
  EXPECT_EQ(it--, it2);
  EXPECT_EQ(it, it1);
}

TEST(ConvexPolygonVertexIterator, Equality) {
  std::vector<ConvexPolygonEdge<>> edges{
    {Point3<>(0, 0, 0), Point3<>(1, 1, 1)},
    {Point3<>(1, 1, 1), Point3<>(0, 0, 0)},
  };

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  using VertexIterator = ConvexPolygonVertexIterator<VectorIterator>;
  const VertexIterator it1(edges.begin() + 0);
  const VertexIterator it2(edges.begin() + 1);

  EXPECT_EQ(it1, it1);
  EXPECT_NE(it1, it2);
}

TEST(ConvexPolygonVertexIterator, PointsTo) {
  std::vector<ConvexPolygonEdge<>> edges{
    {Point3<>(0, 0, 0), Point3<>(1, 1, 1)},
  };

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  using VertexIterator = ConvexPolygonVertexIterator<VectorIterator>;
  const VertexIterator it(edges.begin());

  EXPECT_EQ(it->x(), edges[0].vertex.x());
}

TEST(ConvexPolygonVertexIterator, AddOffset) {
  std::vector<ConvexPolygonEdge<>> edges{
    {Point3<>(0, 0, 0), Point3<>(1, 1, 1)},
    {Point3<>(1, 1, 1), Point3<>(0, 0, 0)},
  };

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  using VertexIterator = ConvexPolygonVertexIterator<VectorIterator>;
  const VertexIterator it1(edges.begin() + 0);
  const VertexIterator it2(edges.begin() + 1);

  EXPECT_EQ(it1 + 1, it2);
}

TEST(ConvexPolygonVertexIterator, SubtractOffset) {
  std::vector<ConvexPolygonEdge<>> edges{
    {Point3<>(0, 0, 0), Point3<>(1, 1, 1)},
    {Point3<>(1, 1, 1), Point3<>(0, 0, 0)},
  };

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  using VertexIterator = ConvexPolygonVertexIterator<VectorIterator>;
  const VertexIterator it1(edges.begin() + 0);
  const VertexIterator it2(edges.begin() + 1);

  EXPECT_EQ(it2 - 1, it1);
}

TEST(ConvexPolygonVertexIterator, SubtractIterator) {
  std::vector<ConvexPolygonEdge<>> edges{
    {Point3<>(0, 0, 0), Point3<>(1, 1, 1)},
    {Point3<>(1, 1, 1), Point3<>(0, 0, 0)},
  };

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  using VertexIterator = ConvexPolygonVertexIterator<VectorIterator>;
  const VertexIterator it1(edges.begin() + 0);
  const VertexIterator it2(edges.begin() + 1);

  EXPECT_EQ(it2 - it1, 1);
}

TEST(ConvexPolygonVertexIterator, PlusEqual) {
  std::vector<ConvexPolygonEdge<>> edges{
    {Point3<>(0, 0, 0), Point3<>(1, 1, 1)},
    {Point3<>(1, 1, 1), Point3<>(0, 0, 0)},
  };

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  using VertexIterator = ConvexPolygonVertexIterator<VectorIterator>;
  VertexIterator it1(edges.begin() + 0);
  VertexIterator it2(edges.begin() + 1);

  VertexIterator it(it1);
  EXPECT_EQ(&(it += 1), &it);
  EXPECT_EQ(it, it2);
}

TEST(ConvexPolygonVertexIterator, MinusEqual) {
  std::vector<ConvexPolygonEdge<>> edges{
    {Point3<>(0, 0, 0), Point3<>(1, 1, 1)},
    {Point3<>(1, 1, 1), Point3<>(0, 0, 0)},
  };

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  using VertexIterator = ConvexPolygonVertexIterator<VectorIterator>;
  VertexIterator it1(edges.begin() + 0);
  VertexIterator it2(edges.begin() + 1);

  VertexIterator it(it2);
  EXPECT_EQ(&(it -= 1), &it);
  EXPECT_EQ(it, it1);
}

TEST(ConvexPolygonVertexIterator, IndexOffset) {
  std::vector<ConvexPolygonEdge<>> edges{
    {Point3<>(0, 0, 0), Point3<>(1, 1, 1)},
    {Point3<>(1, 1, 1), Point3<>(0, 0, 0)},
  };

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  using VertexIterator = ConvexPolygonVertexIterator<VectorIterator>;
  const VertexIterator it1(edges.begin() + 0);
  const VertexIterator it2(edges.begin() + 1);

  EXPECT_EQ(&it1[1], &*it2);
}

TEST(ConvexPolygonVertexIterator, LtGtCompare) {
  std::vector<ConvexPolygonEdge<>> edges{
    {Point3<>(0, 0, 0), Point3<>(1, 1, 1)},
    {Point3<>(1, 1, 1), Point3<>(0, 0, 0)},
  };

  using VectorIterator = std::vector<ConvexPolygonEdge<>>::iterator;
  using VertexIterator = ConvexPolygonVertexIterator<VectorIterator>;
  const VertexIterator it1(edges.begin() + 0);
  const VertexIterator it2(edges.begin() + 1);

  EXPECT_FALSE(it1 < it1);
  EXPECT_TRUE(it1 <= it1);
  EXPECT_TRUE(it1 >= it1);
  EXPECT_FALSE(it1 > it1);

  EXPECT_TRUE(it1 < it2);
  EXPECT_TRUE(it1 <= it2);
  EXPECT_FALSE(it1 >= it2);
  EXPECT_FALSE(it1 > it2);

  EXPECT_FALSE(it2 < it1);
  EXPECT_FALSE(it2 <= it1);
  EXPECT_TRUE(it2 >= it1);
  EXPECT_TRUE(it2 > it1);
}

}  // walnut
