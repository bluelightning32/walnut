#ifndef WALNUT_EDGE_INFO_ROOT_H__
#define WALNUT_EDGE_INFO_ROOT_H__

#include <ostream>

#include "walnut/assignable_wrapper.h"
#include "walnut/homo_point3.h"
#include "walnut/plucker_line.h"

namespace walnut {

template <typename EdgeParent>
class ConvexPolygon;

// This functions as the root of the inheritance chain of the EdgeInfo class in
// a ConvexPolygon.
struct EdgeInfoRoot {
  constexpr EdgeInfoRoot() = default;

  constexpr explicit EdgeInfoRoot(const EdgeInfoRoot&) = default;

  constexpr EdgeInfoRoot(RValueKey<EdgeInfoRoot> other) noexcept { }

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

  constexpr EdgeInfoRoot& operator=(RValueKey<EdgeInfoRoot> other) noexcept {
    return *this;
  }

  bool CanMerge(const EdgeInfoRoot& next) const {
    return true;
  }

  std::ostream& Approximate(std::ostream& out) const {
    return out;
  }

 protected:
  void Merge(EdgeInfoRoot& next) { }

  template <typename EdgeParent>
  void EdgeMoved(ConvexPolygon<EdgeParent>& target) { }
};

}  // walnut

#endif // WALNUT_EDGE_INFO_ROOT_H__
