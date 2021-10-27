#include "walnut/n_log_n_estimator.h"

#include <cmath>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

TEST(NLogNEstimator, Zero) {
  NLogNEstimator estimator;
  EXPECT_EQ(estimator.Estimate(0), 0);
}

TEST(NLogNEstimator, ErrorUpTo1024) {
  NLogNEstimator estimator;
  for (size_t i = 1; i < 1024; ++i) {
    EXPECT_NEAR(estimator.Estimate(i), i * std::log2(i),
                1.0 + 0.015 * i * std::log2(i)) << "i=" << i;
  }
}

}  // walnut
