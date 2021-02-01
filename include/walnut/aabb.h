#ifndef WALNUT_AABB_H__
#define WALNUT_AABB_H__

#include <vector>

#include "walnut/mutable_convex_polygon.h"
#include "walnut/point3.h"

namespace walnut {

// Determines which sides of the plane an AABB is present on. The AABB is
// implicitly defined by its component coordinates.
//
// The caller is responsible for ensuring these invariants:
// min_x <= max_x
// min_y <= max_y
// min_z <= max_z
//
// Returns -1 if the AABB is only present in the negative half-space.
// Returns 0 if the AABB is present on both sides of the plane, or one of its
// vertices is touching the plane.
// Returns 1 if the AABB is only present in the positive half-space.
template <size_t component_bits, size_t half_space_bits>
int GetAABBPlaneSide(const BigInt<component_bits>& min_x,
                     const BigInt<component_bits>& min_y,
                     const BigInt<component_bits>& min_z,
                     const BigInt<component_bits>& max_x,
                     const BigInt<component_bits>& max_y,
                     const BigInt<component_bits>& max_z,
                     const HalfSpace3<half_space_bits>& plane);

// Determines which sides of the plane an AABB is present on. The AABB is
// implicitly defined by its component coordinates.
//
// The caller is responsible for ensuring these invariants:
// min_x * sign(denom) <= max_x * sign(denom)
// min_y * sign(denom) <= max_y * sign(denom)
// min_z * sign(denom) <= max_z * sign(denom)
//
// `denom` is a denominator that applies to all of the coordinates.
//
// Returns -1 if the AABB is only present in the negative half-space.
// Returns 0 if the AABB is present on both sides of the plane, or one of its
// vertices is touching the plane.
// Returns 1 if the AABB is only present in the positive half-space.
template <size_t component_bits, size_t denom_bits, size_t half_space_bits>
int GetAABBPlaneSide(const BigInt<component_bits>& min_x,
                     const BigInt<component_bits>& min_y,
                     const BigInt<component_bits>& min_z,
                     const BigInt<component_bits>& max_x,
                     const BigInt<component_bits>& max_y,
                     const BigInt<component_bits>& max_z,
                     const BigInt<denom_bits>& denom,
                     const HalfSpace3<half_space_bits>& plane);

// Axis-aligned bounding box
//
// This is a rectangular prism whose sides are perpendicular to the axes.
template <size_t point3_bits_template = 32>
struct AABB {
  using Point3Rep = Point3<point3_bits_template>;
  using BigIntRep = typename Point3Rep::BigIntRep;
  using HalfSpace3Rep =
    typename HalfSpace3FromPoint3Builder<point3_bits_template>::HalfSpace3Rep;

  static constexpr size_t point3_bits = point3_bits_template;

  // Creates an AABB that does not contain any points.
  AABB() : min_point(1, 1, 1), max_point(0, 0, 0) { }

  AABB(Point3Rep min_point, Point3Rep max_point) :
    min_point(min_point),
    max_point(max_point) { }

  AABB(const BigIntRep& min_x, const BigIntRep& min_y,
       const BigIntRep& min_z, const BigIntRep& max_x,
       const BigIntRep& max_y, const BigIntRep& max_z) :
    AABB(Point3Rep(min_x, min_y, min_z), Point3Rep(max_x, max_y, max_z)) { }

  AABB(int min_x, int min_y, int min_z, int max_x, int max_y, int max_z) :
    AABB(Point3Rep(min_x, min_y, min_z), Point3Rep(max_x, max_y, max_z)) { }

  AABB(int radius) :
    min_point(/*x=*/-radius, /*y=*/-radius, /*z=*/-radius),
    max_point(/*x=*/radius, /*y=*/radius, /*z=*/radius) { }

  // Returns true if `p` is on the border (but still inside) of the prism.
  bool IsOnBorder(const Point3Rep& p) const {
    if (!IsInside(p)) return false;
    for (int i = 0; i < 3; ++i) {
      if (p.components()[i] == min_point.components()[i] ||
          p.components()[i] == max_point.components()[i]) return true;
    }
    return false;
  }

  // Returns true if `p` is on the border (but still inside) of the prism.
  template <size_t num_bits, size_t denom_bits>
  bool IsOnBorder(const HomoPoint3<num_bits, denom_bits>& p) const {
    if (!IsInside(p)) return false;
    for (int i = 0; i < 3; ++i) {
      const BigInt<num_bits>& p_comp = p.vector_from_origin().components()[i];
      if (p_comp == min_point.components()[i] * p.w() ||
          p_comp == max_point.components()[i] * p.w()) return true;
    }
    return false;
  }

  // Returns true if `p` is inside the prism.
  bool IsInside(const Point3Rep& p) const {
    for (int i = 0; i < 3; ++i) {
      if (p.components()[i] < min_point.components()[i] ||
          p.components()[i] > max_point.components()[i]) return false;
    }
    return true;
  }

  // Returns true if `p` is inside the prism.
  template <size_t num_bits, size_t denom_bits>
  bool IsInside(const HomoPoint3<num_bits, denom_bits>& p) const {
    const int mult = p.w().GetAbsMult();
    for (int i = 0; i < 3; ++i) {
      auto p_flipped = p.vector_from_origin().components()[i] * mult;
      if (p_flipped < min_point.components()[i] * p.w() * mult) return false;
      if (p_flipped > max_point.components()[i] * p.w() * mult) return false;
    }
    return true;
  }

  // Returns a ConvexPolygon for the intersection of this rectangular prism and
  // a plane (represented as a HalfSpace3).
  template <typename ConvexPolygonRep = MutableConvexPolygon<point3_bits>>
  ConvexPolygonRep IntersectPlane(const HalfSpace3Rep& plane) const;

  // Determines which sides of the plane the AABB is present on.
  //
  // `negative_side` is set to true if part of the AABB is present on the
  // negative side of `plane`. `positive_side` is set to true if part of the
  // AABB is present on the positive side of `plane`.
  //
  // If the AABB has a vertex touching the plane, then both `negative_side` and
  // `positive_side` are set to true.
  //
  // Returns -1 if the AABB is only present in the negative half-space.
  // Returns 0 if the AABB is present on both sides of the plane, or one of its
  // vertices is touching the plane.
  // Returns 1 if the AABB is only present in the positive half-space.
  template <size_t half_space_bits>
  int GetPlaneSide(const HalfSpace3<half_space_bits>& plane) const {
    return GetAABBPlaneSide(min_point.x(), min_point.y(), min_point.z(),
                     max_point.x(), max_point.y(), max_point.z(), plane);
  }

  // Returns all 6 sides of the prism.
  std::vector<ConvexPolygon<point3_bits>> GetWalls() const;

  // This point is considered part of the prism
  Point3Rep min_point;
  // This point is considered part of the prism
  Point3Rep max_point;
};

template <size_t point3_bits>
template <typename ConvexPolygonRep>
ConvexPolygonRep AABB<point3_bits>::IntersectPlane(
    const HalfSpace3Rep& plane) const {
  int drop_dimension = plane.normal().GetFirstNonzeroDimension();
  if (drop_dimension == -1) {
    // The plane is invalid. So return an empty polygon.
    return ConvexPolygonRep();
  }
  int flip = plane.normal().components()[drop_dimension].GetAbsMult();

  // First create a parallelogram by intersecting `plane` with 4 sides of the
  // prism.
  using Vector3Rep = typename HalfSpace3Rep::VectorRep;
  const int dim1 = (drop_dimension + 1) % 3;
  const int dim2 = (drop_dimension + 2) % 3;
  Vector3Rep dir1 = Vector3Rep::Zero();
  Vector3Rep dir2 = Vector3Rep::Zero();
  Vector3Rep dir3 = Vector3Rep::Zero();
  dir1.components()[dim1] = 1;
  dir2.components()[dim2] = 1;
  dir3.components()[drop_dimension] = 1;
  HalfSpace3Rep parallelogram_planes[4] = {
    HalfSpace3Rep(-dir1,
                  -BigInt<point3_bits + 1>(min_point.components()[dim1])),
    HalfSpace3Rep(-dir2,
                  -BigInt<point3_bits + 1>(min_point.components()[dim2])),
    HalfSpace3Rep(dir1, max_point.components()[dim1]),
    HalfSpace3Rep(dir2, max_point.components()[dim2]),
  };

  std::vector<typename ConvexPolygonRep::EdgeRep> edges;
  edges.reserve(4);
  const HalfSpace3Rep* prev_plane = &parallelogram_planes[(4 + -1*flip) % 4];
  for (int i = 0; i < 4; ++i) {
    const HalfSpace3Rep* cur_plane = &parallelogram_planes[(4 + i*flip) % 4];
    typename ConvexPolygonRep::LineRep line(plane, *cur_plane);
    edges.emplace_back(line.Intersect(*prev_plane), std::move(line));
    prev_plane = cur_plane;
  }
  ConvexPolygonRep result(plane, drop_dimension, std::move(edges));

  // Now split the parallelogram on the remaining 2 sides of the prism.
  HalfSpace3Rep drop_top(dir3, max_point.components()[drop_dimension]);
  auto split1 = result.GetSplitInfo(drop_top);
  if (!split1.ShouldEmitNegativeChild()) {
    if (split1.ShouldEmitOnPlane()) {
      return result;
    }
    // `plane` is outside of the prism.
    return ConvexPolygonRep();
  }
  if (split1.ShouldEmitPositiveChild()) {
    // Keep the negative child
    result = std::move(result).CreateSplitChildren(std::move(split1)).first;
  }

  HalfSpace3Rep drop_bottom(dir3, min_point.components()[drop_dimension]);
  auto split2 = result.GetSplitInfo(drop_bottom);
  if (!split2.ShouldEmitPositiveChild()) {
    if (split2.ShouldEmitOnPlane()) {
      return result;
    }
    // `plane` is outside of the prism.
    return ConvexPolygonRep();
  }
  if (split2.ShouldEmitNegativeChild()) {
    // Keep the positive child
    result = std::move(result).CreateSplitChildren(std::move(split2)).second;
  }
  return result;
}

template <size_t point3_bits>
std::vector<ConvexPolygon<AABB<point3_bits>::point3_bits>>
AABB<point3_bits>::GetWalls() const {
  Point3<point3_bits> p[] = {
    Point3<point3_bits>(min_point.x(), min_point.y(), min_point.z()),
    Point3<point3_bits>(max_point.x(), min_point.y(), min_point.z()),
    Point3<point3_bits>(max_point.x(), max_point.y(), min_point.z()),
    Point3<point3_bits>(min_point.x(), max_point.y(), min_point.z()),
    Point3<point3_bits>(min_point.x(), min_point.y(), max_point.z()),
    Point3<point3_bits>(max_point.x(), min_point.y(), max_point.z()),
    Point3<point3_bits>(max_point.x(), max_point.y(), max_point.z()),
    Point3<point3_bits>(min_point.x(), max_point.y(), max_point.z()),
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

  std::vector<ConvexPolygon<point3_bits>> result;
  std::vector<Point3<point3_bits>> vertices;
  vertices.reserve(4);
  for (int side = 0; side < 6; ++side) {
    const FacetInfo& facet_info = facet_infos[side];
    vertices.clear();
    for (int i = 0; i < 4; ++i) {
      vertices.push_back(p[facet_info.vertex_indices[i]]);
    }
    Vector3<point3_bits> normal = Vector3<point3_bits>::Zero();
    normal.components()[facet_info.normal_dimension] = side < 3 ? -1 : 1;
    HalfSpace3Rep plane(normal, /*dist=*/side < 3 ?
        -min_point.components()[facet_info.normal_dimension] : 
        max_point.components()[facet_info.normal_dimension]);
    result.emplace_back(plane, facet_info.normal_dimension, vertices);
  }
  return result;
}

template <size_t component_bits, size_t half_space_bits>
int GetAABBPlaneSide(const BigInt<component_bits>& min_x,
                     const BigInt<component_bits>& min_y,
                     const BigInt<component_bits>& min_z,
                     const BigInt<component_bits>& max_x,
                     const BigInt<component_bits>& max_y,
                     const BigInt<component_bits>& max_z,
                     const HalfSpace3<half_space_bits>& plane) {
  const bool x_pos = plane.x().GetSign() > 0;
  const bool y_pos = plane.y().GetSign() > 0;
  const bool z_pos = plane.z().GetSign() > 0;
  const auto min_value = (x_pos ? min_x : max_x)*plane.x() +
                         (y_pos ? min_y : max_y)*plane.y() +
                         (z_pos ? min_z : max_z)*plane.z();
  if (min_value > plane.d()) {
    return 1;
  }
  const auto max_value = (x_pos ? max_x : min_x)*plane.x() +
                         (y_pos ? max_y : min_y)*plane.y() +
                         (z_pos ? max_z : min_z)*plane.z();
  if (max_value < plane.d()) {
    return -1;
  } else {
    return 0;
  }
}

template <size_t component_bits, size_t denom_bits, size_t half_space_bits>
int GetAABBPlaneSide(const BigInt<component_bits>& min_x,
                     const BigInt<component_bits>& min_y,
                     const BigInt<component_bits>& min_z,
                     const BigInt<component_bits>& max_x,
                     const BigInt<component_bits>& max_y,
                     const BigInt<component_bits>& max_z,
                     const BigInt<denom_bits>& denom,
                     const HalfSpace3<half_space_bits>& plane) {
  const bool x_pos = plane.x().GetSign() > 0;
  const bool y_pos = plane.y().GetSign() > 0;
  const bool z_pos = plane.z().GetSign() > 0;
  // min_value is the distance along the plane normal of the AABB's point that
  // is farthest on the negative side of the plane.
  const auto min_value = (x_pos ? min_x : max_x)*plane.x() +
                         (y_pos ? min_y : max_y)*plane.y() +
                         (z_pos ? min_z : max_z)*plane.z();
  const auto scaled_d = plane.d() * denom;
  if (min_value.Compare(scaled_d) * denom.GetAbsMult() > 0) {
    // Even min_value is in the positive half-space. So the entire AABB is in
    // the positive half-space.
    return 1;
  }
  // max_value is the distance along the plane normal of the AABB's point that
  // is farthest on the positive side of the plane.
  const auto max_value = (x_pos ? max_x : min_x)*plane.x() +
                         (y_pos ? max_y : min_y)*plane.y() +
                         (z_pos ? max_z : min_z)*plane.z();
  if (max_value.Compare(scaled_d) * denom.GetAbsMult() < 0) {
    // Even max_value is in the negative half-space. So the entire AABB is in
    // the negative half-space.
    return -1;
  } else {
    return 0;
  }
}

template <size_t point3_bits = 32>
std::ostream& operator<<(std::ostream& out,
                         const AABB<point3_bits>& rect) {
  out << "min_point=" << rect.min_point;
  out << " max_point=" << rect.max_point;
  return out;
}

}  // walnut

#endif // WALNUT_AABB_H__
