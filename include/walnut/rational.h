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
inline BigInt RoundUp(const BigInt& numerator,
                          const BigInt& old_denominator,
                          const BigInt& new_denominator) {
  BigInt remainder;
  BigInt new_num =
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
inline BigInt RoundDown(const BigInt& numerator,
                            const BigInt& old_denominator,
                            const BigInt& new_denominator) {
  BigInt remainder;
  BigInt new_num =
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

inline bool IsLessThan(const BigInt& num1, const BigInt& denom1,
                       const BigInt& num2, const BigInt& denom2) {
  // num1 / denom1 <? num2 / denom2
  // num1 * denom2 <? num2 * denom1 (flip if (denom1 * denom2) is negative)
  return (num1 * denom2).LessThan(
      /*flip=*/(denom1.GetSign() < 0) ^ (denom2.GetSign() < 0),
      num2 * denom1);
}

// Returns true if num1/denom1 == num2/denom2.
inline bool Equals(const BigInt& num1, const BigInt& denom1,
                   const BigInt& num2, const BigInt& denom2) {
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
inline bool IsHalfRotationLessThan(const BigInt& x1, const BigInt& y1,
                                   const BigInt& x2, const BigInt& y2,
                                   const BigInt& y1_x2, const BigInt& y2_x1) {
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
