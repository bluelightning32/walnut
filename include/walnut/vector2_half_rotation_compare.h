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
    // For determining whether to negate y, look at the sign of v.y(), when
    // v.y() != 0. If v.y() == 0, use the sign of v.x() instead.
    //
    // v_y_0_adjust will be 1 if v.y() is non-negative and v.x() is negative.
    //
    // So the sign of (v.y() - v_y_0_adjust) can be used to determine whether
    // to negate v.
    bool v_y_0_adjust = (v.y().GetSign() >= 0) & (v.x().GetSign() < 0);
    bool u_y_0_adjust = (u.y().GetSign() >= 0) & (u.x().GetSign() < 0);
    const bool flip = (v.y() - int(v_y_0_adjust)).HasDifferentSign(
        u.y() - int(u_y_0_adjust));
    return (v.y() * u.x()).LessThan(flip, u.y() * v.x());
  }
};

}  // walnut

#endif // WALNUT_VECTOR2_HALF_ROTATION_COMPARE_H__
