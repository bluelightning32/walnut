#include "walnut/m_log_n_estimator.h"

namespace walnut {

size_t GetSplitCost(size_t neg_total, size_t neg_exclude, size_t pos_total,
                    size_t pos_exclude) {
  MLogNEstimator estimator;
  return estimator.Estimate(neg_total, neg_total - neg_exclude + 1) +
         estimator.Estimate(pos_total, pos_total - pos_exclude + 1);
}

}  // walnut
