#include "walnut/m_log_n_estimator.h"

#include <cmath>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

TEST(MLogNEstimator, Zero) {
  MLogNEstimator estimator;
  EXPECT_EQ(estimator.Estimate(0, 0), 0);
}

TEST(MLogNEstimator, ErrorUpTo1024) {
  MLogNEstimator estimator;
  for (size_t i = 1; i < 1024; ++i) {
    EXPECT_NEAR(estimator.Estimate(i, i), i * std::log2(i),
                1.0 + 0.015 * i * std::log2(i)) << "i=" << i;
  }
}

TEST(MLogNEstimator, HalfM) {
  MLogNEstimator estimator;
  for (size_t i = 1; i < 1024; ++i) {
    EXPECT_NEAR(estimator.Estimate(i/2, i), (i/2) * std::log2(i),
                1.0 + 0.018 * (i/2) * std::log2(i)) << "i=" << i;
  }
}

TEST(MLogNEstimator, QuarterM) {
  MLogNEstimator estimator;
  for (size_t i = 1; i < 1024; ++i) {
    EXPECT_NEAR(estimator.Estimate(i/4, i), (i/4) * std::log2(i),
                1.0 + 0.018 * (i/4) * std::log2(i)) << "i=" << i;
  }
}

TEST(MLogNEstimator, QuarterN) {
  MLogNEstimator estimator;
  for (size_t i = 4; i < 1024; ++i) {
    EXPECT_NEAR(estimator.Estimate(i, i/4), i * std::log2(i/4),
                1.0 + 0.033 * i * std::log2(i/4)) << "i=" << i;
  }
}

}  // walnut
