#include "walnut/aabb.h"

namespace walnut {

std::vector<MutableConvexPolygon<>> AABB::GetWalls() const {
  HomoPoint3 p[] = {
    HomoPoint3(min_point_num_.x(), min_point_num_.y(), min_point_num_.z(),
               denom_),
    HomoPoint3(max_point_num_.x(), min_point_num_.y(), min_point_num_.z(),
               denom_),
    HomoPoint3(max_point_num_.x(), max_point_num_.y(), min_point_num_.z(),
               denom_),
    HomoPoint3(min_point_num_.x(), max_point_num_.y(), min_point_num_.z(),
               denom_),
    HomoPoint3(min_point_num_.x(), min_point_num_.y(), max_point_num_.z(),
               denom_),
    HomoPoint3(max_point_num_.x(), min_point_num_.y(), max_point_num_.z(),
               denom_),
    HomoPoint3(max_point_num_.x(), max_point_num_.y(), max_point_num_.z(),
               denom_),
    HomoPoint3(min_point_num_.x(), max_point_num_.y(), max_point_num_.z(),
               denom_),
  };

  struct FacetInfo {
    int normal_dimension;
    int vertex_indices[4];
  } facet_infos[6] = {
    // bottom
    {2, {0, 3, 2, 1}},
    // min x side
    {0, {3, 0, 4, 7}},
    // min y side
    {1, {0, 1, 5, 4}},
    // max x side
    {0, {1, 2, 6, 5}},
    // max y side
    {1, {2, 3, 7, 6}},
    // top
    {2, {4, 5, 6, 7}},
  };

  std::vector<MutableConvexPolygon<>> result;
  std::vector<HomoPoint3> vertices;
  vertices.reserve(4);
  for (int side = 0; side < 6; ++side) {
    const FacetInfo& facet_info = facet_infos[side];
    if (min_point_num_.components()[(facet_info.normal_dimension + 1) % 3] ==
        max_point_num_.components()[(facet_info.normal_dimension + 1) % 3]) {
      // The facet is 0 length in one of its dimensions.
      continue;
    }
    if (min_point_num_.components()[(facet_info.normal_dimension + 2) % 3] ==
        max_point_num_.components()[(facet_info.normal_dimension + 2) % 3]) {
      // The facet is 0 length in one of its dimensions.
      continue;
    }
    vertices.clear();
    for (int i = 0; i < 4; ++i) {
      vertices.push_back(p[facet_info.vertex_indices[i]]);
    }
    Vector3 normal = Vector3::Zero();
    normal.components()[facet_info.normal_dimension] =
      side < 3 ? -denom_ : denom_;
    HalfSpace3 plane(normal, /*dist=*/side < 3 ?
        -min_point_num_.components()[facet_info.normal_dimension] : 
        max_point_num_.components()[facet_info.normal_dimension]);
    result.emplace_back(plane, facet_info.normal_dimension, vertices);
  }
  return result;
}

std::ostream& operator<<(std::ostream& out, const AABB& rect) {
  out << "[ min=" << rect.min_point_num();
  out << " max=" << rect.max_point_num();
  out << " / " << rect.denom() << " ]";
  return out;
}

}  // walnut
