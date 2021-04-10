#ifndef WALNUT_EDGE_INFO_ROOT_H__
#define WALNUT_EDGE_INFO_ROOT_H__

#include "walnut/homo_point3.h"
#include "walnut/plucker_line.h"

namespace walnut {

// This functions as the root of the inheritance chain of the EdgeInfo class in
// a ConvexPolygon.
struct EdgeInfoRoot {
  constexpr EdgeInfoRoot() = default;

  constexpr explicit EdgeInfoRoot(const EdgeInfoRoot&) = default;

  EdgeInfoRoot(const EdgeInfoRoot& parent, const HomoPoint3& new_source) { }

  EdgeInfoRoot(const EdgeInfoRoot& parent, const PluckerLine& new_line) { }

  EdgeInfoRoot(const EdgeInfoRoot& parent, const HomoPoint3& new_source,
               const PluckerLine& new_line) { }

  constexpr bool operator==(const EdgeInfoRoot& other) const {
    return true;
  }

  constexpr bool operator!=(const EdgeInfoRoot& other) const {
    return false;
  }

  constexpr EdgeInfoRoot& operator=(const EdgeInfoRoot& other) {
    return *this;
  }

  // Returns true if this edge can be merged (removed) when `prev` is the
  // previous edge.
  //
  // The caller ensures that this edge is on the same line and points the same
  // direction as `prev.
  bool CanMerge(const EdgeInfoRoot& prev) const {
    return true;
  }
};

}  // walnut

#endif // WALNUT_EDGE_INFO_ROOT_H__
