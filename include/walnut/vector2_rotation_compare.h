#ifndef WALNUT_VECTOR2_ROTATION_COMPARE_H__
#define WALNUT_VECTOR2_ROTATION_COMPARE_H__

#include "walnut/vector2.h"

namespace walnut {

// Compares two Vector2 by their counter-clockwise rotation from the x axis.
//
// More specifically for two vectors v and u, where the counter-clockwise
// rotation from the x axis to each vector is in the range [0, 1), then
// compare(v, u) returns true if the v's rotation is strictly less than u's.
// That is, compare(v, u) acts like v < u.
//
// This class meets the std requirements for a Compare. Notably it meets the
// transitivity requirements.
template <size_t coord_bits_template = 33>
struct Vector2RotationCompare {
 public:
  using Vector2Rep = Vector2<coord_bits_template>;

  static constexpr size_t coord_bits = coord_bits_template;

  Vector2RotationCompare() = default;

  bool operator()(const Vector2Rep& v, const Vector2Rep& u) const {
    bool v_y_sign = v.y().GetSign() < 0;
    bool u_y_sign = u.y().GetSign() < 0;
    // v_packed_sign is a quadrant that v is in, although the quadrant numbers
    // are out of order.
    char v_packed_sign = (char(v_y_sign) << 1) | char(v.x().GetSign() < 0);
    // u_packed_sign is a quadrant that u is in, although the quadrant numbers
    // are out of order.
    char u_packed_sign = (char(u_y_sign) << 1) | char(u.x().GetSign() < 0);
    if (v_packed_sign != u_packed_sign) {
      // v and u are in different quadrants. So properly order the quadrants,
      // then compare the quadrant numbers.
      return (v_packed_sign ^ v_y_sign) < (u_packed_sign ^ u_y_sign);
    }
    // v and u are in the same quadrant. Compare their x/y ratios.
    return v.y() * u.x() < u.y() * v.x();
  }
};

}  // walnut

#endif // WALNUT_VECTOR2_ROTATION_COMPARE_H__
