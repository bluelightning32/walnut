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

  template <size_t num_bits, size_t denom_bits>
  NoVertexData(const NoVertexData& parent,
               const HomoPoint3<num_bits, denom_bits>& new_source) { }

  template <size_t d_bits, size_t m_bits>
  NoVertexData(const NoVertexData& parent,
               const PluckerLine<d_bits, m_bits>& new_line) { }

  template <size_t num_bits, size_t denom_bits, size_t d_bits, size_t m_bits>
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
