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

// Returns true if num1/denom1 == num2/denom2.
template <size_t num_bits1, size_t denom_bits1,
          size_t num_bits2, size_t denom_bits2>
bool Equals(const BigInt<num_bits1>& num1, const BigInt<denom_bits1>& denom1,
            const BigInt<num_bits2>& num2, const BigInt<denom_bits2>& denom2) {
  return num1 * denom2 == num2 * denom1;
}

} // rational

}  // walnut

#endif // WALNUT_CONVEX_VERTEX_AABB_TRACKER_H__
