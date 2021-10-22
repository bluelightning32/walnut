#ifndef WALNUT_DOUBLE_H__
#define WALNUT_DOUBLE_H__

#include <cfloat>
#include <cmath>

#include "walnut/big_uint_word.h"

namespace walnut {

// Decomposes `input` into a mantissa and exponent.
//
// The return value and exponent meet this criteria:
//   ret * 2^exp = input
// 
// Of the possible return values that satisfy the above criteria, the one with
// the smallest absolute value is returned.
//
// If the input is 0, 0 is returned with `exp` set to 0.
inline int64_t Decompose(double input, int* exp) {
  static_assert(FLT_RADIX == 2, "Only binary doubles are supported.");
  int64_t result = std::scalbn(std::frexp(input, exp), DBL_MANT_DIG);
  if (!result) {
    assert(*exp == 0);
    return result;
  }
  *exp -= DBL_MANT_DIG;
  unsigned trailing_zeros = BigUIntWord(result).GetTrailingZeros();
  result >>= trailing_zeros;
  *exp += trailing_zeros;
  return result;
}

}  // walnut

#endif // WALNUT_DOUBLE_H__
