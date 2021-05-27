#ifndef WALNUT_CONVEX_POLYGON_FACTORY_H__
#define WALNUT_CONVEX_POLYGON_FACTORY_H__

#include "walnut/monotone_range.h"
#include "walnut/mutable_convex_polygon.h"
#include "walnut/orienting_monotone_decomposer.h"
#include "walnut/planar_range.h"

namespace walnut {

template <typename InputPoint3Template = Point3,
          typename ConvexPolygonTemplate = MutableConvexPolygon<
            typename GetVertexData<InputPoint3Template>::VertexData>
         >
class ConvexPolygonFactory :
  private OrientingMonotoneDecomposer<InputPoint3Template> {
 public:
  using InputPoint3 = InputPoint3Template;
  using ConvexPolygonRep = ConvexPolygonTemplate;

  // `Point3Iterator` should produce Point3Reps.
  template <typename Point3Iterator>
  void Build(Point3Iterator begin, Point3Iterator end) {
    using PlanarRangeRep = PlanarRange<Point3Iterator>;
    PlanarRangeRep planar_range;
    using MonotoneRangeRep =
      MonotoneRange<typename PlanarRangeRep::OutputIterator>;
    MonotoneRangeRep monotone_range;

    while (begin != end) {
      planar_range.Build(begin, end);
      plane_ = planar_range.plane();
      drop_dimension_ = plane_.normal().GetFirstNonzeroDimension();
      // drop_dimension_ is -1 if the points are collinear.
      if (drop_dimension_ == -1) continue;
      int monotone_dimension = (~drop_dimension_) & 1;
      plane_orientation_ =
        GetPlaneOrientationAfterProjection(plane_.normal(), drop_dimension_);
      typename PlanarRangeRep::OutputIterator planar_begin =
        planar_range.begin();
      typename PlanarRangeRep::OutputIterator planar_end =
        planar_range.end();

      while (planar_begin != planar_end) {
        monotone_range.Build(monotone_dimension, planar_begin, planar_end);

        using Chain1Iterator =
          typename MonotoneRangeRep::ConcatRangeRep::const_iterator;
        using Chain2Iterator =
          typename MonotoneRangeRep::ConcatRangeRep::const_reverse_iterator;
        Chain1Iterator chain1_begin, chain1_end;
        Chain2Iterator chain2_begin, chain2_end;

        monotone_range.GetChains(chain1_begin, chain1_end,
                                 chain2_begin, chain2_end);
        Parent::Build(drop_dimension_, monotone_dimension, chain1_begin,
                      chain1_end, chain2_begin, chain2_end);
      }
    }
  }

  // Determines whether the vertex is reflex or not after projection.
  //
  // cross_product is the cross product of 2 vectors coming out of the vertex.
  // drop_dimension is the dimension that will be removed from the 3D to 2D
  // projection.
  //
  // This function is likely only useful internally. It is publically exposed
  // for testing purposes.
  //
  // Returns -1 if the vertex will be convex after projection, or 1 if the
  // vertex will be reflex. The return value is undefined if
  // cross_product.IsZero().
  static int GetPlaneOrientationAfterProjection(const Vector3& cross_product,
                                                int drop_dimension) {
    return -cross_product.components()[drop_dimension].GetAbsMult();
  }
 
 protected:
  virtual void Emit(ConvexPolygonRep&& polygon) = 0;

 private:
  using Parent = OrientingMonotoneDecomposer<InputPoint3>;

  void EmitOriented(int orientation,
                    typename Parent::const_reverse_iterator range1_begin,
                    typename Parent::const_reverse_iterator range1_end,
                    typename Parent::const_iterator range2_begin,
                    typename Parent::const_iterator range2_end) override {
    if (orientation == 0) {
      // Skip collinear polygons
      return;
    }
    typename ConvexPolygonRep::EdgeVector edges;
    edges.reserve((range1_end - range1_begin) + (range2_end - range2_begin));
    const InputPoint3* prev;
    const InputPoint3* first;
    if (range1_begin != range1_end) {
      auto pos1 = range1_begin;
      prev = &*pos1;
      first = prev;
      ++pos1;
      while (pos1 != range1_end) {
        PluckerLine line(*prev, *pos1);
        if (line.IsValid()) {
          edges.emplace_back(*prev, std::move(line));
          prev = &*pos1;
        }
        ++pos1;
      }
      for (auto pos2 = range2_begin; pos2 != range2_end; ++pos2) {
        PluckerLine line(*prev, *pos2);
        if (line.IsValid()) {
          edges.emplace_back(*prev, std::move(line));
          prev = &*pos2;
        }
      }
    } else {
      auto pos2 = range2_begin;
      prev = &*pos2;
      first = prev;
      ++pos2;
      while (pos2 != range2_end) {
        PluckerLine line(*prev, *pos2);
        if (line.IsValid()) {
          edges.emplace_back(*prev, std::move(line));
          prev = &*pos2;
        }
        ++pos2;
      }
    }
    PluckerLine line(*prev, *first);
    if (line.IsValid()) {
      edges.emplace_back(*prev, std::move(line));
    }
    // orientation is -1 if the polygon is counter-clockwise.
    // plane_orientation_ is -1 if plane_ is already the normal of a
    // counter-clockwise polygon. If both are -1, then plane_ is already
    // correct, and they should cancel out.
    const int flip_orientation = (orientation ^ plane_orientation_) | 1;
    Emit(ConvexPolygonRep(HalfSpace3(plane_.normal() * flip_orientation,
                                     plane_.d() * flip_orientation),
                          drop_dimension_,
                          std::move(edges)));
  }

  int drop_dimension_;
  HalfSpace3 plane_;
  // This is -1 if plane_ already represents the normal of a counter-clockwise
  // polygon, or 1 if plane_ represents the normal of a clockwise polygon.
  int plane_orientation_;
};

}  // walnut

#endif // WALNUT_CONVEX_POLYGON_FACTORY_H__
