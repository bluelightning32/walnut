#ifndef WALNUT_M_LOG_N_ESTIMATOR_H__
#define WALNUT_M_LOG_N_ESTIMATOR_H__

#include "walnut/big_uint_word.h"

namespace walnut {

// Estimates the value of m*log_2(n).
//
// The estimated value is within (1 + 0.033*v) of the correct value, v, when
// m <= 4*n.
class MLogNEstimator {
  public:
   size_t Estimate(size_t m, size_t n) {
     if (n >> last_n_bits_ != 1) {
       last_n_bits_ = BigUIntWord(n).GetHighestSetBit() - 1;
     }

#ifdef __SIZEOF_INT128__
     const unsigned __int128 large_m = static_cast<unsigned __int128>(m);
     const unsigned __int128 large_n = static_cast<unsigned __int128>(n);
#else
     const uint64_t large_m = static_cast<uint64_t>(m);
     const uint64_t large_n = static_cast<uint64_t>(n);
#endif
     return static_cast<size_t>(large_m*large_n >> last_n_bits_) +
            m*(last_n_bits_ - 1);
   }

  private:
   int last_n_bits_ = 0;
};

}  // walnut

#endif // WALNUT_M_LOG_N_ESTIMATOR_H__
