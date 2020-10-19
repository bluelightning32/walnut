#ifndef WALNUT_CONVEX_POLYGON_FACTORY_H__
#define WALNUT_CONVEX_POLYGON_FACTORY_H__

#include "walnut/convex_polygon.h"
#include "walnut/monotone_range.h"
#include "walnut/orienting_monotone_decomposer.h"
#include "walnut/planar_range.h"

namespace walnut {

template <int vertex3_bits_template>
class ConvexPolygon<vertex3_bits_template>::Factory :
  private OrientingMonotoneDecomposer<vertex3_bits_template> {
 public:
  using Vertex3Rep = ConvexPolygon::Vertex3Rep;
  using ConvexPolygonRep = ConvexPolygon<vertex3_bits_template>;
  using PlaneRep = typename ConvexPolygonRep::PlaneRep;

  template <typename Vertex3Iterator>
  void Build(Vertex3Iterator begin, Vertex3Iterator end) {
    using PlanarRangeRep = PlanarRange<Vertex3Iterator>;
    PlanarRangeRep planar_range;
    using MonotoneRangeRep =
      MonotoneRange<typename PlanarRangeRep::OutputIterator>;
    MonotoneRangeRep monotone_range;

    while (begin != end) {
      planar_range.Build(begin, end);
      plane_ = planar_range.plane();
      int monotone_dimension;
      if (plane_.x() != 0) {
        drop_dimension_ = 0;
        monotone_dimension = 1;
      } else if (plane_.y() != 0) {
        drop_dimension_ = 1;
        monotone_dimension = 0;
      } else {
        drop_dimension_ = 2;
        monotone_dimension = 0;
      }
      plane_orientation_ =
        plane_.normal().coords()[drop_dimension_].GetSign() < 0 ? 1 : -1;
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
 
 protected:
  virtual void Emit(ConvexPolygon&& polygon) = 0;

 private:
  using Parent = OrientingMonotoneDecomposer<vertex3_bits_template>;

  void EmitOriented(int orientation,
                    typename Parent::const_reverse_iterator range1_begin,
                    typename Parent::const_reverse_iterator range1_end,
                    typename Parent::const_iterator range2_begin,
                    typename Parent::const_iterator range2_end) override {
    if (orientation == 0) {
      // Skip collinear polygons
      return;
    }
    std::vector<ConvexPolygonRep::VertexInfo> vertices;
    vertices.reserve((range1_end - range1_begin) + (range2_end - range2_begin));
    const Vertex3Rep* prev;
    const Vertex3Rep* first;
    if (range1_begin != range1_end) {
      auto pos1 = range1_begin;
      prev = &*pos1;
      first = prev;
      ++pos1;
      while (pos1 != range1_end) {
        vertices.emplace_back(*prev, *pos1);
        prev = &*pos1;
        ++pos1;
      }
      for (auto pos2 = range2_begin; pos2 != range2_end; ++pos2) {
        vertices.emplace_back(*prev, *pos2);
        prev = &*pos2;
      }
    } else {
      auto pos2 = range2_begin;
      prev = &*pos2;
      first = prev;
      ++pos2;
      while (pos2 != range2_end) {
        vertices.emplace_back(*prev, *pos2);
        prev = &*pos2;
        ++pos2;
      }
    }
    vertices.emplace_back(*prev, *first);
    // orientation is -1 if the polygon is counter-clockwise.
    // plane_orientation_ is -1 if plane_ is already the normal of a
    // counter-clockwise polygon. If both are -1, then plane_ is already
    // correct, and they should cancel out.
    const int flip_orientation = orientation ^ plane_orientation_;
    Emit(ConvexPolygon(PlaneRep(plane_.normal() * flip_orientation,
                                plane_.d() * flip_orientation),
                       drop_dimension_,
                       std::move(vertices)));
  }

  int drop_dimension_;
  PlaneRep plane_;
  // This is -1 if plane_ already represents the normal of a counter-clockwise
  // polygon, or 1 if plane_ represents the normal of a clockwise polygon.
  int plane_orientation_;
};

}  // walnut

#endif // WALNUT_CONVEX_POLYGON_FACTORY_H__
