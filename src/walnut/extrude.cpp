#include "walnut/extrude.h"

#include "walnut/convex_polygon_factory.h"
#include "walnut/homo_point3.h"
#include "walnut/mutable_convex_polygon.h"
#include "walnut/vector3.h"

namespace walnut {

std::vector<MutableConvexPolygon<>> Extrude(
    const std::vector<HomoPoint3>& bottom_cap,
    const Vector3& direction, const BigInt& direction_denom) {

  class Collector : public ConvexPolygonFactory<HomoPoint3> {
   public:
    using ConvexPolygonFactory<HomoPoint3>::ConvexPolygonRep;
    std::vector<ConvexPolygonRep> result;

   protected:
    void Emit(ConvexPolygonRep&& polygon) override {
      result.push_back(std::move(polygon));
    }
  };

  Collector polygon_factory;
  // Add the bottom cap
  polygon_factory.Build(bottom_cap.begin(), bottom_cap.end());

  // Add the sides
  for (size_t i = 0; i < bottom_cap.size(); ++i) {
    HomoPoint3 side_vertices[4];
    side_vertices[0] = bottom_cap[(i + 1) % bottom_cap.size()];
    side_vertices[1] = bottom_cap[i];
    side_vertices[2] = side_vertices[1].AddOffset(direction, direction_denom);
    side_vertices[3] = side_vertices[0].AddOffset(direction, direction_denom);
    polygon_factory.Build(std::begin(side_vertices), std::end(side_vertices));
  }

  // Add the top cap
  std::vector<HomoPoint3> top_cap;
  for (auto it = bottom_cap.rbegin(); it != bottom_cap.rend(); ++it) {
    top_cap.push_back(it->AddOffset(direction, direction_denom));
  }
  polygon_factory.Build(top_cap.begin(), top_cap.end());

  return std::move(polygon_factory.result);
}

}  // walnut
