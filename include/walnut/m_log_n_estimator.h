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

// Estimates the amount of remaining work after performing a split such that
// `neg_total` polygons remain on the negative side and `pos_total` polygons
// remaining on the positive side.
//
// If the parent node contained entries from multiple meshes, one of those
// meshes may be excluded from part of the calcuation to estimate the effort
// saved from not having to fully subdivide nodes that are not part of the
// output mesh. Whichever mesh is chosen as the excluded one, `neg_exclude` and
// `pos_exclude` may be set to the number of polygons of that mesh which are on
// each side of the split. If the parent node only contains entries from a
// single mesh, `neg_exclude` and `pos_exclude` should both be set to 0.
//
// The returned value is within (1 + 0.018*value) of the following value:
//  neg_total * log(neg_total - neg_exclude + 1) +
//  pos_total * log(pos_total - pos_exclude + 1)
size_t GetSplitCost(size_t neg_total, size_t neg_exclude, size_t pos_total,
                    size_t pos_exclude);

}  // walnut

#endif // WALNUT_M_LOG_N_ESTIMATOR_H__
