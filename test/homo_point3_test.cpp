#include "walnut/homo_point3.h"

#include <unordered_map>

#include "gtest/gtest.h"

namespace walnut {

TEST(HomoPoint3, EqualityOperator) {
  HomoPoint3 p(0, 1, 2, 7);
  EXPECT_EQ(p, p);

  // Test an equivalent point with double the w.
  {
    HomoPoint3 compare(0, 2, 4, 14);
    EXPECT_EQ(p, compare);
    EXPECT_EQ(compare, p);
  }

  // Test an equivalent point with opposite w.
  {
    HomoPoint3 compare(0, -1, -2, -7);
    EXPECT_EQ(p, compare);
    EXPECT_EQ(compare, p);
  }

  // Test a different point
  {
    HomoPoint3 compare(0, 1, 1, 7);
    EXPECT_NE(p, compare);
    EXPECT_NE(compare, p);
  }
}

TEST(HomoPoint3, LexicographicallyLtBasic) {
  HomoPoint3 p1(0, 0, 1, 1);
  HomoPoint3 p2(0, 0, 2, 1);
  EXPECT_TRUE(HomoPoint3::LexicographicallyLt(p1, p2));
  EXPECT_FALSE(HomoPoint3::LexicographicallyLt(p2, p1));
}

TEST(HomoPoint3, LexicographicallyLtDifferentDist) {
  HomoPoint3 p1(0, 0, 3, 5);
  HomoPoint3 p2(0, 0, 3, 2);
  EXPECT_TRUE(HomoPoint3::LexicographicallyLt(p1, p2));
  EXPECT_FALSE(HomoPoint3::LexicographicallyLt(p2, p1));
}

TEST(HomoPoint3, LexicographicallyLtDifferentDistSign) {
  HomoPoint3 p1(0, 0, 1, 1);
  HomoPoint3 p2(0, 0, -2, -1);
  EXPECT_TRUE(HomoPoint3::LexicographicallyLt(p1, p2));
  EXPECT_FALSE(HomoPoint3::LexicographicallyLt(p2, p1));
}

TEST(HomoPoint3, ReduceAllPos) {
  HomoPoint3 original(2, 4, 8, 2);

  HomoPoint3 reduced = original;
  reduced.Reduce();
  EXPECT_EQ(reduced, original);
  EXPECT_EQ(reduced.w(), 1);
}

TEST(HomoPoint3, ReduceNegDenom) {
  HomoPoint3 original(2, -4, 8, -2);

  HomoPoint3 reduced = original;
  reduced.Reduce();
  EXPECT_EQ(reduced, original);
  EXPECT_EQ(reduced.w(), 1);
}

TEST(HomoPoint3, ReduceAllIntMin) {
  HomoPoint3 original(/*x=*/BigInt::min_value(255),
                      /*y=*/BigInt::min_value(255),
                      /*z=*/BigInt::min_value(255),
                      /*w=*/BigInt::min_value(255));

  HomoPoint3 reduced = original;
  reduced.Reduce();
  EXPECT_EQ(reduced, original);
  EXPECT_EQ(reduced.w(), 1);
}

TEST(HomoPoint3, ReduceDifferentFactors) {
  HomoPoint3 original(11 * 2 * 3, 13 * 2 * 5, 17 * 2 * 7, 2 * 3 * 5 * 7);

  HomoPoint3 reduced = original;
  reduced.Reduce();
  EXPECT_EQ(reduced, original);
  EXPECT_EQ(reduced.w(), 3 * 5 * 7);
}

TEST(HomoPoint3, FromDoublesExact2) {
  HomoPoint3 p = HomoPoint3::FromDoublesExact(/*x=*/2, /*y=*/2, /*z=*/2);

  EXPECT_EQ(p.x(), 2);
  EXPECT_EQ(p.y(), 2);
  EXPECT_EQ(p.z(), 2);
  EXPECT_EQ(p.w(), 1);
}

TEST(HomoPoint3, FromDoublesExactOneHalf) {
  HomoPoint3 p = HomoPoint3::FromDoublesExact(/*x=*/0.5, /*y=*/0.5, /*z=*/0.5);

  EXPECT_EQ(p.x(), 1);
  EXPECT_EQ(p.y(), 1);
  EXPECT_EQ(p.z(), 1);
  EXPECT_EQ(p.w(), 2);
}

TEST(HomoPoint3, FromDoublesExact2AndOneHalf) {
  HomoPoint3 p = HomoPoint3::FromDoublesExact(/*x=*/2, /*y=*/2, /*z=*/0.5);

  EXPECT_EQ(p.x(), 4);
  EXPECT_EQ(p.y(), 4);
  EXPECT_EQ(p.z(), 1);
  EXPECT_EQ(p.w(), 2);
}

TEST(HomoPoint3, FromDoublesExactVeryBig) {
  double d = std::ldexp(1, 1000);
  HomoPoint3 p = HomoPoint3::FromDoublesExact(/*x=*/d, /*y=*/d, /*z=*/d);

  EXPECT_EQ(p.x(), BigInt(1) << 1000);
  EXPECT_EQ(p.y(), BigInt(1) << 1000);
  EXPECT_EQ(p.z(), BigInt(1) << 1000);
  EXPECT_EQ(p.w(), 1);
}

TEST(HomoPoint3, FromDoublesExactVerySmall) {
  double d = std::ldexp(1, -1000);
  HomoPoint3 p = HomoPoint3::FromDoublesExact(/*x=*/d, /*y=*/d, /*z=*/d);

  EXPECT_EQ(p.x(), 1);
  EXPECT_EQ(p.y(), 1);
  EXPECT_EQ(p.z(), 1);
  EXPECT_EQ(p.w(), BigInt(1) << 1000);
}

TEST(HomoPoint3, FromDoublesExactZero) {
  HomoPoint3 p = HomoPoint3::FromDoublesExact(/*x=*/0, /*y=*/0, /*z=*/0);

  EXPECT_EQ(p.x(), 0);
  EXPECT_EQ(p.y(), 0);
  EXPECT_EQ(p.z(), 0);
  EXPECT_EQ(p.w(), 1);
}

TEST(HomoPoint3, FromDoublesExactNextAfterf0) {
  if (std::numeric_limits<long double>::min_exponent ==
      std::numeric_limits<float>::min_exponent) {
    GTEST_SKIP();
  }
  double d = std::nextafterf(0.0, 1.0);
  HomoPoint3 p = HomoPoint3::FromDoublesExact(/*x=*/d, /*y=*/d, /*z=*/d);

  EXPECT_EQ(p.x(), 1);
  EXPECT_EQ(p.y(), 1);
  EXPECT_EQ(p.z(), 1);
  EXPECT_EQ((long double)p.x() / (long double)p.w(), d)
      << "p.x()=" << p.x() << " p.w()=" << p.w();
  EXPECT_EQ((long double)p.y() / (long double)p.w(), d);
  EXPECT_EQ((long double)p.z() / (long double)p.w(), d);

  EXPECT_EQ(p.GetDoublePoint3(), DoublePoint3(d, d, d));
}

TEST(HomoPoint3, FromDoublesExactNextAfter0) {
  if (std::numeric_limits<long double>::min_exponent ==
      std::numeric_limits<double>::min_exponent) {
    GTEST_SKIP();
  }
  double d = std::nextafter(0.0, 1.0);
  HomoPoint3 p = HomoPoint3::FromDoublesExact(/*x=*/d, /*y=*/d, /*z=*/d);

  EXPECT_EQ(p.x(), 1);
  EXPECT_EQ(p.y(), 1);
  EXPECT_EQ(p.z(), 1);
  EXPECT_EQ((long double)p.x() / (long double)p.w(), d)
      << "p.x()=" << p.x() << " p.w()=" << p.w();
  EXPECT_EQ((long double)p.y() / (long double)p.w(), d);
  EXPECT_EQ((long double)p.z() / (long double)p.w(), d);

  EXPECT_EQ(p.GetDoublePoint3(), DoublePoint3(d, d, d));
}

TEST(HomoPoint3, FromDoublesRoundPosToward0) {
  HomoPoint3 p = HomoPoint3::FromDoubles(/*min_exponent=*/-1,
                                         /*x=*/0.001,
                                         /*y=*/0.001,
                                         /*z=*/0.001);

  EXPECT_EQ(p.x(), 0);
  EXPECT_EQ(p.y(), 0);
  EXPECT_EQ(p.z(), 0);
  EXPECT_NE(p.w(), 0);
}

TEST(HomoPoint3, FromDoublesRoundNegToward0) {
  HomoPoint3 p = HomoPoint3::FromDoubles(/*min_exponent=*/-1,
                                         /*x=*/-0.001,
                                         /*y=*/-0.001,
                                         /*z=*/-0.001);

  EXPECT_EQ(p.x(), 0);
  EXPECT_EQ(p.y(), 0);
  EXPECT_EQ(p.z(), 0);
  EXPECT_NE(p.w(), 0);
}

TEST(HomoPoint3, GetDoublePoint3) {
  DoublePoint3 input{10, 0.1, 0.01};
  HomoPoint3 p = HomoPoint3::FromDoublesExact(input.x, input.y, input.z);

  EXPECT_EQ(p.GetDoublePoint3(), input);
}

TEST(HomoPoint3, Get2DTwistDirCollinear) {
  HomoPoint3 p1(/*x=*/1, /*y=*/2, /*z=*/3, /*w=*/5);
  HomoPoint3 p2(/*x=*/-2, /*y=*/-4, /*z=*/-6, /*w=*/-5);
  HomoPoint3 p3(/*x=*/3, /*y=*/6, /*z=*/9, /*w=*/6);

  EXPECT_EQ(p2.Get2DTwistDir(/*drop_dimension=*/0, p1, p3), 0);
  EXPECT_EQ(p2.Get2DTwistDir(/*drop_dimension=*/1, p1, p3), 0);
  EXPECT_EQ(p2.Get2DTwistDir(/*drop_dimension=*/2, p1, p3), 0);
}

TEST(HomoPoint3, Get2DTwistDirPosDenom) {
  //
  //  p1               |
  //  ^                |
  //   \               |
  //   p2 -> p3        |
  //
  HomoPoint3 p1(/*x=*/1, /*y=*/2, /*z=*/1, /*w=*/3);
  HomoPoint3 p2(/*x=*/1, /*y=*/1, /*z=*/1, /*w=*/2);
  HomoPoint3 p3(/*x=*/3, /*y=*/2, /*z=*/1, /*w=*/4);

  // p3 is clockwise from p1.
  EXPECT_LT(p2.Get2DTwistDir(/*drop_dimension=*/2, p1, p3), 0);
  // p1 is counter-clockwise from p3.
  EXPECT_GT(p2.Get2DTwistDir(/*drop_dimension=*/2, p3, p1), 0);
}

TEST(HomoPoint3, Get2DTwistDirPosPosNegDenom) {
  //
  //  p1               |
  //  ^                |
  //   \               |
  //   p2 -> p3        |
  //
  HomoPoint3 p1(/*x=*/1, /*y=*/2, /*z=*/1, /*w=*/3);
  HomoPoint3 p2(/*x=*/1, /*y=*/1, /*z=*/1, /*w=*/2);
  HomoPoint3 p3(/*x=*/-3, /*y=*/-2, /*z=*/-1, /*w=*/-4);

  // p3 is clockwise from p1.
  EXPECT_LT(p2.Get2DTwistDir(/*drop_dimension=*/2, p1, p3), 0);
  // p1 is counter-clockwise from p3.
  EXPECT_GT(p2.Get2DTwistDir(/*drop_dimension=*/2, p3, p1), 0);
}

TEST(HomoPoint3, Get2DTwistDirPosNegNegDenom) {
  //
  //  p1               |
  //  ^                |
  //   \               |
  //   p2 -> p3        |
  //
  HomoPoint3 p1(/*x=*/1, /*y=*/2, /*z=*/1, /*w=*/3);
  HomoPoint3 p2(/*x=*/-1, /*y=*/-1, /*z=*/-1, /*w=*/-2);
  HomoPoint3 p3(/*x=*/-3, /*y=*/-2, /*z=*/-1, /*w=*/-4);

  // p3 is clockwise from p1.
  EXPECT_LT(p2.Get2DTwistDir(/*drop_dimension=*/2, p1, p3), 0);
  // p1 is counter-clockwise from p3.
  EXPECT_GT(p2.Get2DTwistDir(/*drop_dimension=*/2, p3, p1), 0);
}

TEST(HomoPoint3, Get2DTwistDirNegNegNegDenom) {
  //
  //  p1               |
  //  ^                |
  //   \               |
  //   p2 -> p3        |
  //
  HomoPoint3 p1(/*x=*/-1, /*y=*/-2, /*z=*/-1, /*w=*/-3);
  HomoPoint3 p2(/*x=*/-1, /*y=*/-1, /*z=*/-1, /*w=*/-2);
  HomoPoint3 p3(/*x=*/-3, /*y=*/-2, /*z=*/-1, /*w=*/-4);

  // p3 is clockwise from p1.
  EXPECT_LT(p2.Get2DTwistDir(/*drop_dimension=*/2, p1, p3), 0);
  // p1 is counter-clockwise from p3.
  EXPECT_GT(p2.Get2DTwistDir(/*drop_dimension=*/2, p3, p1), 0);
}

TEST(HomoPoint3, DifferenceBothPosDenom) {
  HomoPoint3 p1(/*x=*/1, /*y=*/1, /*z=*/1, /*w=*/2);
  HomoPoint3 p2(/*x=*/2, /*y=*/2, /*z=*/2, /*w=*/3);

  BigInt denom;
  Vector3 diff = p2.Difference(p1, denom);
  if (denom >= 0) {
    EXPECT_TRUE(diff.IsSameDir(Vector3(1, 1, 1)));
  } else {
    EXPECT_TRUE(diff.IsSameDir(-Vector3(1, 1, 1)));
  }
  // 2/3 - 1/2 = 4/6 - 3/6 = 1/6
  EXPECT_EQ(diff.x() * 6, 1 * denom);
}

TEST(HomoPoint3, DifferenceSecondNegDenom) {
  HomoPoint3 p1(/*x=*/1, /*y=*/1, /*z=*/1, /*w=*/2);
  HomoPoint3 p2(/*x=*/2, /*y=*/2, /*z=*/2, /*w=*/-3);

  BigInt denom;
  Vector3 diff = p2.Difference(p1, denom);
  if (denom >= 0) {
    EXPECT_TRUE(diff.IsSameDir(Vector3(-1, -1, -1)));
  } else {
    EXPECT_TRUE(diff.IsSameDir(-Vector3(-1, -1, -1)));
  }
  // 2/-3 - 1/2 = -4/6 - 3/6 = -7/6
  EXPECT_EQ(diff.x() * 6, -7 * denom);
}

TEST(ReducedHomoPoint3Hasher, UseInUnorderedMap) {
  std::vector<HomoPoint3> points{
    HomoPoint3(/*x=*/1, /*y=*/1, /*z=*/1, /*w=*/2),
    HomoPoint3(/*x=*/1, /*y=*/1, /*z=*/1, /*w=*/3),
    HomoPoint3(/*x=*/1, /*y=*/1, /*z=*/1, /*w=*/4),
  };
  for (long i = 1; i < 5; ++i) {
    points.emplace_back(/*x=*/i, /*y=*/0, /*z=*/0, /*w=*/1);
    points.emplace_back(/*x=*/0, /*y=*/i, /*z=*/0, /*w=*/1);
    points.emplace_back(/*x=*/0, /*y=*/0, /*z=*/i, /*w=*/1);
    points.emplace_back(/*x=*/i, /*y=*/i, /*z=*/0, /*w=*/1);
  }

  std::unordered_map<HomoPoint3, size_t, ReducedHomoPoint3Hasher> map;
  for (size_t i = 0; i < points.size(); ++i) {
    map[points[i]] = i;
  }
  for (size_t i = 0; i < points.size(); ++i) {
    EXPECT_EQ(map[points[i]], i);
  }
}

}  // walnut
