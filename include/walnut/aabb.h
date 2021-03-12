#ifndef WALNUT_AABB_H__
#define WALNUT_AABB_H__

#include <vector>

#include "walnut/mutable_convex_polygon.h"
#include "walnut/point3.h"

namespace walnut {

// Axis-aligned bounding box
//
// This is a rectangular prism whose sides are perpendicular to the axes.
template <size_t num_bits_template = 32, size_t denom_bits_template = 32>
class AABB {
 public:
  using HomoPoint3Rep = HomoPoint3<num_bits_template, denom_bits_template>;
  using VectorRep = typename HomoPoint3Rep::VectorRep;
  using NumInt = typename HomoPoint3Rep::NumInt;
  using DenomInt = typename HomoPoint3Rep::DenomInt;
  using HalfSpace3Rep =
    typename HalfSpace3FromPoint3Builder<num_bits_template>::HalfSpace3Rep;

  static constexpr size_t num_bits = num_bits_template;
  static constexpr size_t denom_bits = denom_bits_template;

  // Creates an AABB that does not contain any points.
  AABB() : min_point_num_(1, 1, 1), max_point_num_(0, 0, 0) { }

  template <size_t other_num_bits>
  AABB(Point3<other_num_bits> min_point, Point3<other_num_bits> max_point) :
    min_point_num_(min_point.vector_from_origin()),
    max_point_num_(max_point.vector_from_origin()) { }

  AABB(VectorRep min_point, VectorRep max_point, const DenomInt& denom) :
      min_point_num_(min_point), max_point_num_(max_point), denom_(denom) {
    if (denom_.GetSign() < 0) {
      min_point_num_.Negate();
      max_point_num_.Negate();
      denom_.Negate();
    }
  }

  AABB(VectorRep min_point, VectorRep max_point, int denom) :
    AABB(min_point, max_point, DenomInt(denom)) { }

  AABB(const NumInt& min_x, const NumInt& min_y,
       const NumInt& min_z, const NumInt& max_x,
       const NumInt& max_y, const NumInt& max_z) :
    AABB(min_x, min_y, min_z, max_x, max_y, max_z, DenomInt(1)) { }

  AABB(const NumInt& min_x, const NumInt& min_y,
       const NumInt& min_z, const NumInt& max_x,
       const NumInt& max_y, const NumInt& max_z, const DenomInt& denom) :
    AABB(VectorRep(min_x, min_y, min_z), VectorRep(max_x, max_y, max_z),
         denom) { }

  AABB(int min_x, int min_y, int min_z, int max_x, int max_y, int max_z,
       int denom) :
    AABB(VectorRep(min_x, min_y, min_z), VectorRep(max_x, max_y, max_z),
         denom) { }

  AABB(int radius) :
    min_point_num_(/*x=*/-radius, /*y=*/-radius, /*z=*/-radius),
    max_point_num_(/*x=*/radius, /*y=*/radius, /*z=*/radius) { }

  // Returns true if `p` is on the border (but still inside) of the prism.
  template <size_t other_bits>
  bool IsOnBorder(const Point3<other_bits>& p) const {
    if (!IsInside(p)) return false;
    for (int i = 0; i < 3; ++i) {
      if (p.components()[i] * denom_ == min_point_num_.components()[i] ||
          p.components()[i] * denom_ == max_point_num_.components()[i]) {
        return true;
      }
    }
    return false;
  }

  // Returns true if `p` is on the border (but still inside) of the prism.
  template <size_t num_bits, size_t denom_bits>
  bool IsOnBorder(const HomoPoint3<num_bits, denom_bits>& p) const {
    if (!IsInside(p)) return false;
    for (int i = 0; i < 3; ++i) {
      const BigInt<num_bits>& p_comp =
        p.vector_from_origin().components()[i] * denom_;
      if (p_comp == min_point_num_.components()[i] * p.w() ||
          p_comp == max_point_num_.components()[i] * p.w()) return true;
    }
    return false;
  }

  // Returns true if `p` is inside or on the border of the prism.
  template <size_t other_bits>
  bool IsInside(const Point3<other_bits>& p) const {
    for (int i = 0; i < 3; ++i) {
      if (p.components()[i] * denom_ < min_point_num_.components()[i] ||
          p.components()[i] * denom_ > max_point_num_.components()[i]) {
        return false;
      }
    }
    return true;
  }

  // Returns true if `p` is inside or on the border of the prism.
  template <size_t num_bits, size_t denom_bits>
  bool IsInside(const HomoPoint3<num_bits, denom_bits>& p) const {
    const bool flip = p.w().GetSign() < 0;
    for (int i = 0; i < 3; ++i) {
      auto p_scaled = p.vector_from_origin().components()[i] * denom_;
      if (p_scaled.LessThan(flip, min_point_num_.components()[i] * p.w())) {
        return false;
      }
      if ((max_point_num_.components()[i] * p.w()).LessThan(flip, p_scaled)) {
        return false;
      }
    }
    return true;
  }

  // Returns a ConvexPolygon for the intersection of this rectangular prism and
  // a plane (represented as a HalfSpace3).
  template <typename ConvexPolygonRep = MutableConvexPolygon<num_bits>>
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
  int GetPlaneSide(const HalfSpace3<half_space_bits>& plane) const;

  // Returns all 6 sides of the prism.
  std::vector<ConvexPolygon<num_bits>> GetWalls() const;

  HomoPoint3Rep min_point() const {
    return HomoPoint3Rep(min_point_num_, denom_);
  }

  VectorRep min_point_num() const {
    return min_point_num_;
  }

  HomoPoint3Rep max_point() const {
    return HomoPoint3Rep(max_point_num_, denom_);
  }

  VectorRep max_point_num() const {
    return max_point_num_;
  }

  const DenomInt& denom() const {
    return denom_;
  }

  template <size_t other_num_bits, size_t other_denom_bits>
  bool operator==(const AABB<other_num_bits, other_denom_bits>& other) const {
    if (min_point_num() * other.denom() != other.min_point_num() * denom()) {
      return false;
    }
    if (max_point_num() * other.denom() != other.max_point_num() * denom()) {
      return false;
    }
    return true;
  }

  template <size_t other_num_bits, size_t other_denom_bits>
  bool operator!=(const AABB<other_num_bits, other_denom_bits>& other) const {
    return !(*this == other);
  }

 private:
  // min_point_num_/denom_ is part of the prism
  VectorRep min_point_num_;
  // max_point_num_/denom_ is part of the prism
  VectorRep max_point_num_;

  // Must be positive.
  DenomInt denom_ = DenomInt(1);
};

template <size_t num_bits, size_t denom_bits>
template <typename ConvexPolygonRep>
ConvexPolygonRep AABB<num_bits, denom_bits>::IntersectPlane(
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
  dir1.components()[dim1] = denom_;
  dir2.components()[dim2] = denom_;
  dir3.components()[drop_dimension] = denom_;
  HalfSpace3Rep parallelogram_planes[4] = {
    HalfSpace3Rep(-dir1,
                  -BigInt<num_bits + 1>(min_point_num_.components()[dim1])),
    HalfSpace3Rep(-dir2,
                  -BigInt<num_bits + 1>(min_point_num_.components()[dim2])),
    HalfSpace3Rep(dir1, max_point_num_.components()[dim1]),
    HalfSpace3Rep(dir2, max_point_num_.components()[dim2]),
  };

  typename ConvexPolygonRep::EdgeVector edges;
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
  HalfSpace3Rep drop_top(dir3, max_point_num_.components()[drop_dimension]);
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

  HalfSpace3Rep drop_bottom(dir3, min_point_num_.components()[drop_dimension]);
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

template <size_t num_bits, size_t denom_bits>
std::vector<ConvexPolygon<AABB<num_bits, denom_bits>::num_bits>>
AABB<num_bits, denom_bits>::GetWalls() const {
  HomoPoint3<num_bits, denom_bits> p[] = {
    HomoPoint3<num_bits, denom_bits>(
        min_point_num_.x(), min_point_num_.y(), min_point_num_.z(), denom_),
    HomoPoint3<num_bits, denom_bits>(
        max_point_num_.x(), min_point_num_.y(), min_point_num_.z(), denom_),
    HomoPoint3<num_bits, denom_bits>(
        max_point_num_.x(), max_point_num_.y(), min_point_num_.z(), denom_),
    HomoPoint3<num_bits, denom_bits>(
        min_point_num_.x(), max_point_num_.y(), min_point_num_.z(), denom_),
    HomoPoint3<num_bits, denom_bits>(
        min_point_num_.x(), min_point_num_.y(), max_point_num_.z(), denom_),
    HomoPoint3<num_bits, denom_bits>(
        max_point_num_.x(), min_point_num_.y(), max_point_num_.z(), denom_),
    HomoPoint3<num_bits, denom_bits>(
        max_point_num_.x(), max_point_num_.y(), max_point_num_.z(), denom_),
    HomoPoint3<num_bits, denom_bits>(
        min_point_num_.x(), max_point_num_.y(), max_point_num_.z(), denom_),
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

  std::vector<ConvexPolygon<num_bits>> result;
  std::vector<HomoPoint3<num_bits, denom_bits>> vertices;
  vertices.reserve(4);
  for (int side = 0; side < 6; ++side) {
    const FacetInfo& facet_info = facet_infos[side];
    vertices.clear();
    for (int i = 0; i < 4; ++i) {
      vertices.push_back(p[facet_info.vertex_indices[i]]);
    }
    Vector3<num_bits> normal = Vector3<num_bits>::Zero();
    normal.components()[facet_info.normal_dimension] =
      side < 3 ? -denom_ : denom_;
    HalfSpace3Rep plane(normal, /*dist=*/side < 3 ?
        -min_point_num_.components()[facet_info.normal_dimension] : 
        max_point_num_.components()[facet_info.normal_dimension]);
    result.emplace_back(plane, facet_info.normal_dimension, vertices);
  }
  return result;
}

template <size_t num_bits, size_t denom_bits>
template <size_t half_space_bits>
int AABB<num_bits, denom_bits>::GetPlaneSide(
    const HalfSpace3<half_space_bits>& plane) const {
  const bool x_pos = plane.x().GetSign() > 0;
  const bool y_pos = plane.y().GetSign() > 0;
  const bool z_pos = plane.z().GetSign() > 0;
  // min_value is the distance along the plane normal of the AABB's point that
  // is farthest on the negative side of the plane.
  const auto min_value =
    (x_pos ? min_point_num_.x() : max_point_num_.x())*plane.x() +
    (y_pos ? min_point_num_.y() : max_point_num_.y())*plane.y() +
    (z_pos ? min_point_num_.z() : max_point_num_.z())*plane.z();
  const auto scaled_d = plane.d() * denom_;
  if (min_value.Compare(scaled_d) * denom_.GetAbsMult() > 0) {
    // Even min_value is in the positive half-space. So the entire AABB is in
    // the positive half-space.
    return 1;
  }
  // max_value is the distance along the plane normal of the AABB's point that
  // is farthest on the positive side of the plane.
  const auto max_value =
    (x_pos ? max_point_num_.x() : min_point_num_.x())*plane.x() +
    (y_pos ? max_point_num_.y() : min_point_num_.y())*plane.y() +
    (z_pos ? max_point_num_.z() : min_point_num_.z())*plane.z();
  if (max_value.Compare(scaled_d) * denom_.GetAbsMult() < 0) {
    // Even max_value is in the negative half-space. So the entire AABB is in
    // the negative half-space.
    return -1;
  } else {
    return 0;
  }
}

template <size_t num_bits, size_t denom_bits>
std::ostream& operator<<(std::ostream& out,
                         const AABB<num_bits, denom_bits>& rect) {
  out << "[ min=" << rect.min_point_num();
  out << " max=" << rect.max_point_num();
  out << " / " << rect.denom() << " ]";
  return out;
}

}  // walnut

#endif // WALNUT_AABB_H__
