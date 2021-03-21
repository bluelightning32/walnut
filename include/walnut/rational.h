// Contains helper functions for dealing with rational numbers represented as
// an integer numerator and integer denominator.
#ifndef WALNUT_CONVEX_RATIONAL_H__
#define WALNUT_CONVEX_RATIONAL_H__

#include "walnut/big_int.h"

namespace walnut {

namespace rational {

// Takes an input rational in the form: numerator/old_denominator and finds the
// closest rational that is greater than or equal to the old one and in the
// form: new_num/new_denominator. new_num is returned.
template <size_t num_bits, size_t old_denom_bits, size_t new_denom_bits>
BigInt<num_bits + new_denom_bits> RoundUp(
    const BigInt<num_bits>& numerator,
    const BigInt<old_denom_bits>& old_denominator,
    const BigInt<new_denom_bits>& new_denominator) {
  BigInt<std::min(num_bits + new_denom_bits, old_denom_bits)> remainder;
  BigInt<num_bits + new_denom_bits> new_num =
    (numerator * new_denominator).DivideRemainder(old_denominator, &remainder);
  // The exact result is:
  //   new_num/new_denominator + (remainder/old_denominator)/new_denominator
  //
  // So as part of rounding up, add new_denominator.GetAbsMult to new_num if
  //   (remainder/old_denominator)/new_denominator > 0
  //   remainder*old_denominator*new_denominator > 0
  if ((remainder.GetSign() ^ old_denominator.GetSign() ^
       new_denominator.GetSign()) >= 0 && !remainder.IsZero()) {
    new_num += new_denominator.GetAbsMult();
  }
  return new_num;
}

// Takes an input rational in the form: numerator/old_denominator and finds the
// closest rational that is less than or equal to the old one and in the
// form: new_num/new_denominator. new_num is returned.
template <size_t num_bits, size_t old_denom_bits, size_t new_denom_bits>
BigInt<num_bits + new_denom_bits> RoundDown(
    const BigInt<num_bits>& numerator,
    const BigInt<old_denom_bits>& old_denominator,
    const BigInt<new_denom_bits>& new_denominator) {
  BigInt<std::min(num_bits + new_denom_bits, old_denom_bits)> remainder;
  BigInt<num_bits + new_denom_bits> new_num =
    (numerator * new_denominator).DivideRemainder(old_denominator, &remainder);
  // The exact result is:
  //   new_num/new_denominator + (remainder/old_denominator)/new_denominator
  //
  // So as part of rounding down, subtract new_denominator.GetAbsMult from
  // new_num if
  //   (remainder/old_denominator)/new_denominator < 0
  //   remainder*old_denominator*new_denominator < 0
  BigIntWord combined_sign = (remainder.GetSign() ^ old_denominator.GetSign() ^
       new_denominator.GetSign());
  if (combined_sign < 0 && !remainder.IsZero()) {
    new_num -= new_denominator.GetAbsMult();
  }
  return new_num;
}

template <size_t num1_bits, size_t denom1_bits, size_t num2_bits,
          size_t denom2_bits>
bool IsLessThan(
    const BigInt<num1_bits>& num1,
    const BigInt<denom1_bits>& denom1,
    const BigInt<num2_bits>& num2,
    const BigInt<denom2_bits>& denom2) {
  // num1 / denom1 <? num2 / denom2
  // num1 * denom2 <? num2 * denom1 (flip if (denom1 * denom2) is negative)
  return (num1 * denom2).LessThan(
      /*flip=*/(denom1.GetSign() < 0) ^ (denom2.GetSign() < 0),
      num2 * denom1);
}

// Returns true if num1/denom1 == num2/denom2.
template <size_t num_bits1, size_t denom_bits1,
          size_t num_bits2, size_t denom_bits2>
bool Equals(const BigInt<num_bits1>& num1, const BigInt<denom_bits1>& denom1,
            const BigInt<num_bits2>& num2, const BigInt<denom_bits2>& denom2) {
  return num1 * denom2 == num2 * denom1;
}

// Returns true if the counter-clockwise angle from the x-axis to normalized
// <x1, y1> is less than the angle from the x-axis to normalized <x2, y2>.
//
// In this context normalized means that if the counter-clockwise rotation
// from the x-axis to the vector is half a rotation or greater, then the
// vector is negated.
//
// `y1_x2` must be set to y1*x2, and `y2_x1` must be set to y2*x1.
//
// Note that this comparison has the transitive property.
template <size_t coord_bits, size_t mult_bits>
inline bool IsHalfRotationLessThan(
    const BigInt<coord_bits>& x1, const BigInt<coord_bits>& y1,
    const BigInt<coord_bits>& x2, const BigInt<coord_bits>& y2,
    const BigInt<mult_bits>& y1_x2,
    const BigInt<mult_bits>& y2_x1) {
  // For determining whether to negate `this`, look at the sign of y1, when
  // y1 != 0. If y1 == 0, use the sign of x1 instead.
  //
  // y1_0_adjust will be 1 if y1 is non-negative and x1 is negative.
  //
  // So the sign of (y1 - y1_0_adjust) can be used to determine whether
  // to negate the first vector.
  bool y1_0_adjust = (y1.GetSign() >= 0) & (x1.GetSign() < 0);
  bool y2_0_adjust = (y2.GetSign() >= 0) & (x2.GetSign() < 0);
  const bool flip = (y1 - int(y1_0_adjust)).HasDifferentSign(
      y2 - int(y2_0_adjust));
  return y1_x2.LessThan(flip, y2_x1);
}

} // rational

}  // walnut

#endif // WALNUT_CONVEX_VERTEX_AABB_TRACKER_H__
