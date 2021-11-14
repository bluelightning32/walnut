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

void TestSplit(size_t base_polys, size_t neg_extra, size_t pos_extra) {
  for (size_t i = 0; i < base_polys; ++i) {
    const size_t neg_total = i + neg_extra;
    const size_t pos_total = base_polys - i + pos_extra;
    EXPECT_NEAR(GetSplitCost(neg_total, /*neg_exclude=*/neg_extra, pos_total,
                             /*pos_exclude=*/pos_extra),
                neg_total * std::log2(i + 1) +
                  pos_total * std::log2(base_polys - i + 1),
                1 + 0.018 * (neg_total * std::log2(i + 1) + pos_total *
                  std::log2(base_polys - i + 1)))
      << "i=" << i;
  }
}

TEST(GetSplitCost, NoExclude1024Polys) {
  TestSplit(1024, /*neg_extra=*/0, /*pos_extra=*/0);
}

TEST(GetSplitCost, NoExclude64Polys) {
  TestSplit(64, /*neg_extra=*/0, /*pos_extra=*/0);
}

TEST(GetSplitCost, NoExclude4Polys) {
  TestSplit(4, /*neg_extra=*/0, /*pos_extra=*/0);
}

TEST(GetSplitCost, NoExclude1Poly) {
  TestSplit(1, /*neg_extra=*/0, /*pos_extra=*/0);
}

TEST(GetSplitCost, FiveExtraOnBoth1024) {
  TestSplit(1024, /*neg_extra=*/5, /*pos_extra=*/5);
}

TEST(GetSplitCost, FiveExtraOnNeg1024) {
  TestSplit(1024, /*neg_extra=*/5, /*pos_extra=*/0);
}

}  // walnut
