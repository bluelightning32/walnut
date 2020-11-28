#ifndef WALNUT_RECTANGULAR_PRISM_H__
#define WALNUT_RECTANGULAR_PRISM_H__

#include "walnut/convex_polygon.h"
#include "walnut/point3.h"

namespace walnut {

template <int point3_bits_template = 32>
struct RectangularPrism {
  using Point3Rep = Point3<point3_bits_template>;
  using HalfSpace3Rep =
    typename HalfSpace3FromPoint3Builder<point3_bits_template>::HalfSpace3Rep;
  using ConvexPolygonRep = ConvexPolygon<point3_bits_template>;

  static constexpr int point3_bits = point3_bits_template;

  RectangularPrism(Point3Rep min_point, Point3Rep max_point) :
    min_point(min_point),
    max_point(max_point) { }

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

  // Returns true if `p` is inside the prism.
  bool IsInside(const Point3Rep& p) const {
    for (int i = 0; i < 3; ++i) {
      if (p.components()[i] < min_point.components()[i] ||
          p.components()[i] > max_point.components()[i]) return false;
    }
    return true;
  }

  // Returns a ConvexPolygon for the intersection of this rectangular prism and
  // a plane (represented as a HalfSpace3).
  ConvexPolygonRep IntersectPlane(const HalfSpace3Rep& plane) const;

  // This point is considered part of the prism
  Point3Rep min_point;
  // This point is considered part of the prism
  Point3Rep max_point;
};

template <int point3_bits_template>
typename RectangularPrism<point3_bits_template>::ConvexPolygonRep
RectangularPrism<point3_bits_template>::IntersectPlane(
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
  auto split1 = std::move(result).GetSplitKey(drop_top);
  if (!split1.ShouldEmitNegativeChild()) {
    if (split1.ShouldEmitOnPlane()) {
      return split1.CreateOnPlaneChild();
    }
    // `plane` is outside of the prism.
    return ConvexPolygonRep();
  }
  if (split1.ShouldEmitPositiveChild()) {
    ConvexPolygonRep neg_child;
    ConvexPolygonRep pos_child;
    split1.CreateSplitChildren(neg_child, pos_child);
    result = std::move(neg_child);
  } else {
    result = split1.CreateNegativeChild();
  }

  HalfSpace3Rep drop_bottom(dir3, min_point.components()[drop_dimension]);
  auto split2 = std::move(result).GetSplitKey(drop_bottom);
  if (!split2.ShouldEmitPositiveChild()) {
    if (split2.ShouldEmitOnPlane()) {
      return split2.CreateOnPlaneChild();
    }
    // `plane` is outside of the prism.
    return ConvexPolygonRep();
  }
  if (split2.ShouldEmitNegativeChild()) {
    ConvexPolygonRep neg_child;
    ConvexPolygonRep pos_child;
    split2.CreateSplitChildren(neg_child, pos_child);
    result = std::move(pos_child);
  } else {
    result = split2.CreatePositiveChild();
  }
  return result;
}

}  // walnut

#endif // WALNUT_RECTANGULAR_PRISM_H__