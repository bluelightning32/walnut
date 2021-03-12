#ifndef WALNUT_EDGE_INFO_ROOT_H__
#define WALNUT_EDGE_INFO_ROOT_H__

#include "walnut/homo_point3.h"
#include "walnut/plucker_line.h"

namespace walnut {

// This functions as the root of the inheritance chain of the EdgeInfo class in
// a ConvexPolygon.
struct EdgeInfoRoot {
  constexpr EdgeInfoRoot() = default;

  template <typename OtherVertexData>
  constexpr explicit EdgeInfoRoot(const OtherVertexData&) { }

  template <size_t num_bits, size_t denom_bits>
  EdgeInfoRoot(const EdgeInfoRoot& parent,
               const HomoPoint3<num_bits, denom_bits>& new_source) { }

  template <size_t d_bits, size_t m_bits>
  EdgeInfoRoot(const EdgeInfoRoot& parent,
               const PluckerLine<d_bits, m_bits>& new_line) { }

  template <size_t num_bits, size_t denom_bits, size_t d_bits, size_t m_bits>
  EdgeInfoRoot(const EdgeInfoRoot& parent,
               const HomoPoint3<num_bits, denom_bits>& new_source,
               const PluckerLine<d_bits, m_bits>& new_line) { }

  template <typename OtherVertexData>
  constexpr bool operator!=(const OtherVertexData& other) const {
    return false;
  }

  template <typename OtherVertexData>
  constexpr EdgeInfoRoot& operator=(const OtherVertexData& other) {
    return *this;
  }
};

}  // walnut

#endif // WALNUT_EDGE_INFO_ROOT_H__
