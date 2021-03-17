#ifndef WALNUT_HALF_SPACE2_SIDELESS_COMPARE_H__
#define WALNUT_HALF_SPACE2_SIDELESS_COMPARE_H__

#include "walnut/half_space2.h"

namespace walnut {

// Compares two HalfSpaces as 2D lines. That is, any half-space is considered
// equivalent to its negation.
//
// The order that this imposes is implementation defined, but it is guaranteed
// to meet the std Compare requirements, including transitivity.
template <size_t vector_bits_template = 31*2 + 3,
          size_t dist_bits_template = 31*3 + 3>
struct HalfSpace2SidelessCompare {
  using HalfSpace2Rep = HalfSpace2<vector_bits_template, dist_bits_template>;
  using VectorRep = typename HalfSpace2Rep::VectorRep;

  static constexpr size_t vector_bits = HalfSpace2Rep::vector_bits;
  static constexpr size_t dist_bits = HalfSpace2Rep::dist_bits;

  bool operator()(const HalfSpace2Rep& h1, const HalfSpace2Rep& h2) const {
    if (h1.normal().IsHalfRotationLessThan(h2.normal())) {
      return true;
    }
    if (h2.normal().IsHalfRotationLessThan(h1.normal())) {
      return false;
    }
    // The vectors are equivalent. Try comparing h1.d/h1.n against h2.d/h2.n,
    // where n is something to normalize the distance. If x is non-zero, then
    // it is used as n. Otherwise y is non-zero and is used as n.
    if (!h1.x().IsZero()) {
      return (h1.d() * h2.x()).LessThan(
          /*flip=*/h1.x().HasDifferentSign(h2.x()), h2.d() * h1.x());
    } else {
      return (h1.d() * h2.y()).LessThan(
          /*flip=*/h1.y().HasDifferentSign(h2.x()), h2.d() * h1.y());
    }
  }
};

}  // walnut

#endif // WALNUT_HALF_SPACE2_SIDELESS_COMPARE_H__
