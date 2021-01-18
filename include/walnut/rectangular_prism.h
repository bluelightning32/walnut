#ifndef WALNUT_RECTANGULAR_PRISM_H__
#define WALNUT_RECTANGULAR_PRISM_H__

#include <vector>

#include "walnut/convex_polygon.h"
#include "walnut/point3.h"

namespace walnut {

template <size_t point3_bits_template = 32>
struct RectangularPrism {
  using Point3Rep = Point3<point3_bits_template>;
  using BigIntRep = typename Point3Rep::BigIntRep;
  using HalfSpace3Rep =
    typename HalfSpace3FromPoint3Builder<point3_bits_template>::HalfSpace3Rep;

  static constexpr size_t point3_bits = point3_bits_template;

  RectangularPrism(Point3Rep min_point, Point3Rep max_point) :
    min_point(min_point),
    max_point(max_point) { }

  RectangularPrism(const BigIntRep& min_x, const BigIntRep& min_y,
                   const BigIntRep& min_z, const BigIntRep& max_x,
                   const BigIntRep& max_y, const BigIntRep& max_z) :
    RectangularPrism(Point3Rep(min_x, min_y, min_z),
                     Point3Rep(max_x, max_y, max_z)) { }

  RectangularPrism(int radius) :
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
  template <typename ConvexPolygonRep = ConvexPolygon<point3_bits>>
  ConvexPolygonRep IntersectPlane(const HalfSpace3Rep& plane) const;

  // Returns all 6 sides of the prism.
  std::vector<ConvexPolygon<point3_bits>> GetWalls() const;

  // This point is considered part of the prism
  Point3Rep min_point;
  // This point is considered part of the prism
  Point3Rep max_point;
};

template <size_t point3_bits>
template <typename ConvexPolygonRep>
ConvexPolygonRep RectangularPrism<point3_bits>::IntersectPlane(
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
std::vector<ConvexPolygon<RectangularPrism<point3_bits>::point3_bits>>
RectangularPrism<point3_bits>::GetWalls() const {
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

template <size_t point3_bits = 32>
std::ostream& operator<<(std::ostream& out,
                         const RectangularPrism<point3_bits>& rect) {
  out << "min_point=" << rect.min_point;
  out << " max_point=" << rect.max_point;
  return out;
}

}  // walnut

#endif // WALNUT_RECTANGULAR_PRISM_H__
