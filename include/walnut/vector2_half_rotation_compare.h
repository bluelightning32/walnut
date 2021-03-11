#ifndef WALNUT_VECTOR2_HALF_ROTATION_COMPARE_H__
#define WALNUT_VECTOR2_HALF_ROTATION_COMPARE_H__

#include "walnut/vector2.h"

namespace walnut {

// Compares two Vector2 by their rotation from the x axis through the y axis
// and up to (but not including) the negative axis.
//
// More specifically for two vectors v and u, where the counter-clockwise
// rotation from the x axis to each vector is in the range [0, 0.5), then
// compare(v, u) returns true if the v's rotation is strictly less than u's.
// That is, compare(v, u) acts like v < u. If v or u's rotation is outside of
// that range, then the rotation of its negation is compared instead.
//
// This class meets the std requirements for a Comparison. Notably it does meet
// the transitivity requirements.
template <size_t coord_bits_template = 33>
struct Vector2HalfRotationCompare {
 public:
  using Vector2Rep = Vector2<coord_bits_template>;

  static constexpr size_t coord_bits = coord_bits_template;

  Vector2HalfRotationCompare() = default;

  bool operator()(const Vector2Rep& v, const Vector2Rep& u) const {
    int sign_adjust = v.y().GetAbsMult(u.y());
    bool result = v.y() * u.x() * sign_adjust < u.y() * v.x() * sign_adjust;
    return result;
  }
};

}  // walnut

#endif // WALNUT_VECTOR2_HALF_ROTATION_COMPARE_H__
