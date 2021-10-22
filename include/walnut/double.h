#ifndef WALNUT_DOUBLE_H__
#define WALNUT_DOUBLE_H__

#include <cfloat>
#include <cmath>
#include <emscripten/emscripten.h>
#include <unistd.h>

#include "walnut/big_uint_word.h"

namespace walnut {


  struct s_mallinfo {
	int arena;    /* non-mmapped space allocated from system */
	int ordblks;  /* number of free chunks */
	int smblks;   /* always 0 */
	int hblks;    /* always 0 */
	int hblkhd;   /* space in mmapped regions */
	int usmblks;  /* maximum total allocated space */
	int fsmblks;  /* always 0 */
	int uordblks; /* total allocated space */
	int fordblks; /* total free space */
	int keepcost; /* releasable (via malloc_trim) space */
};

extern "C" {
	extern s_mallinfo mallinfo();
}

inline unsigned int UsedMemory()
{
	s_mallinfo i = mallinfo();
	unsigned int dynamicTop = (unsigned int)sbrk(0);
	return dynamicTop - i.fordblks;
}

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
