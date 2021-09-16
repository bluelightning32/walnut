#include "walnut/mesh.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "walnut/aabb.h"
#include "walnut/convex_polygon_factory.h"

namespace walnut {

std::vector<ConvexPolygon<>> BuildMesh(
    const std::vector<std::vector<HomoPoint3>>& polygons) {
  std::vector<ConvexPolygon<>> result;
  class ResultCollector : public ConvexPolygonFactory<HomoPoint3> {
   public:
    ResultCollector(std::vector<ConvexPolygon<>>& result) : result_(result) { }
   protected:
    void Emit(ConvexPolygonRep&& polygon) override {
      result_.push_back(std::move(polygon));
    }
   private:
    std::vector<ConvexPolygon<>>& result_;
  } collector(result);

  for (const std::vector<HomoPoint3>& polygon : polygons) {
    collector.Build(polygon.begin(), polygon.end());
  }
  return result;
}

TEST(Mesh, GetCentroid1x1x1Cube) {
  AABB box(0, 0, 0, 1, 1, 1, /*denom=*/1);
  EXPECT_EQ(GetCentroid(box.GetWalls()), HomoPoint3(1, 1, 1, 2));
}

TEST(Mesh, GetCentroid1x2x1Cube) {
  AABB box(0, 0, 0, 1, 2, 1, /*denom=*/1);
  EXPECT_EQ(GetCentroid(box.GetWalls()), HomoPoint3(1, 2, 1, 2));
}

TEST(Mesh, GetCentroid1x1x4Cube) {
  AABB box(0, 0, 0, 1, 1, 4, /*denom=*/1);
  EXPECT_EQ(GetCentroid(box.GetWalls()), HomoPoint3(1, 1, 4, 2));
}

TEST(Mesh, GetCentroidOffset1x1x1Cube) {
  AABB box(10, 10, 10, 11, 11, 11, /*denom=*/1);
  EXPECT_EQ(GetCentroid(box.GetWalls()), HomoPoint3(21, 21, 21, 2));
}

TEST(Mesh, GetCentroidOffsetHalfCube) {
  AABB box(1, 1, 1, 2, 2, 2, /*denom=*/2);
  EXPECT_EQ(GetCentroid(box.GetWalls()), HomoPoint3(3, 3, 3, 4));
}

TEST(Mesh, GetCentroidTetrahedron) {
  std::vector<std::vector<HomoPoint3>> polygons = {
    {HomoPoint3(1, 0, 1, 1), HomoPoint3(1, 1, 0, 1), HomoPoint3(1, 1, 1, 1)},
    {HomoPoint3(1, 0, 1, 1), HomoPoint3(1, 1, 1, 1), HomoPoint3(0, 1, 1, 1)},
    {HomoPoint3(0, 1, 1, 1), HomoPoint3(1, 1, 1, 1), HomoPoint3(1, 1, 0, 1)},
    {HomoPoint3(1, 1, 0, 1), HomoPoint3(1, 0, 1, 1), HomoPoint3(0, 1, 1, 1)},
  };

  std::vector<ConvexPolygon<>> mesh = BuildMesh(polygons);
  EXPECT_EQ(GetCentroid(mesh), HomoPoint3(3, 3, 3, 4));
}

TEST(Mesh, GetCentroidTetrahedronDifferentDenoms) {
  std::vector<std::vector<HomoPoint3>> polygons = {
    {HomoPoint3(2, 0, 2, 2), HomoPoint3(3, 3, 0, 3), HomoPoint3(1, 1, 1, 1)},
    {HomoPoint3(2, 0, 2, 2), HomoPoint3(1, 1, 1, 1), HomoPoint3(0, 1, 1, 1)},
    {HomoPoint3(0, 1, 1, 1), HomoPoint3(1, 1, 1, 1), HomoPoint3(3, 3, 0, 3)},
    {HomoPoint3(3, 3, 0, 3), HomoPoint3(2, 0, 2, 2), HomoPoint3(0, 7, 7, 7)},
  };

  std::vector<ConvexPolygon<>> mesh = BuildMesh(polygons);
  EXPECT_EQ(GetCentroid(mesh), HomoPoint3(3, 3, 3, 4));
}

}  // walnut
