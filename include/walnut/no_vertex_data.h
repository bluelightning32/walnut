#ifndef WALNUT_NO_VERTEX_DATA_H__
#define WALNUT_NO_VERTEX_DATA_H__

#include "walnut/homo_point3.h"
#include "walnut/plucker_line.h"

namespace walnut {

// This is the default data that is attached to every vertex of a
// ConvexPolygon.
struct NoVertexData {
  constexpr NoVertexData() = default;

  template <typename OtherVertexData>
  constexpr explicit NoVertexData(const OtherVertexData&) { }

  template <int num_bits, int denom_bits>
  NoVertexData(const NoVertexData& parent,
               const HomoPoint3<num_bits, denom_bits>& new_source) { }

  template <int d_bits, int m_bits>
  NoVertexData(const NoVertexData& parent,
               const PluckerLine<d_bits, m_bits>& new_line) { }

  template <int num_bits, int denom_bits, int d_bits, int m_bits>
  NoVertexData(const NoVertexData& parent,
               const HomoPoint3<num_bits, denom_bits>& new_source,
               const PluckerLine<d_bits, m_bits>& new_line) { }

  template <typename OtherVertexData>
  constexpr bool operator!=(const OtherVertexData& other) const {
    return false;
  }

  template <typename OtherVertexData>
  constexpr NoVertexData& operator=(const OtherVertexData& other) {
    return *this;
  }
};

}  // walnut

#endif // WALNUT_NO_VERTEX_DATA_H__
