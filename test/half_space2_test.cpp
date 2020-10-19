#include "walnut/half_space2.h"

#include "gtest/gtest.h"

#include "walnut/point2.h"

namespace walnut {

TEST(HalfSpace2, ComparePoint2) {
  // Anything with x>5 is included in the half space.
  HalfSpace2<> half_space(/*normal=*/Vector2<>(/*x=*/2, /*y=*/0), /*dist=*/BigInt<32>(10));

  EXPECT_TRUE(half_space.normal().IsSameDir(Vector2<>(1, 0)));

  // included
  EXPECT_GT(half_space.Compare(Point2<>(/*x=*/1, /*y=*/100)), 0);
  // coincident
  EXPECT_EQ(half_space.Compare(Point2<>(/*x=*/5, /*y=*/100)), 0);
  // excluded
  EXPECT_LT(half_space.Compare(Point2<>(/*x=*/6, /*y=*/100)), 0);
}

TEST(HalfSpace3, CompareHomoPoint2) {
  // Anything with x>5 is included in the half space.
  HalfSpace2<> half_space(/*normal=*/Vector2<>(/*x=*/2, /*y=*/0), /*dist=*/BigInt<32>(10));

  // included
  EXPECT_GT(half_space.Compare(HomoPoint2<>(/*x=*/1, /*y=*/100, /*w=*/1)), 0);
  EXPECT_GT(half_space.Compare(HomoPoint2<>(/*x=*/9, /*y=*/100, /*w=*/2)), 0);
  // coincident
  EXPECT_EQ(half_space.Compare(HomoPoint2<>(/*x=*/5, /*y=*/100, /*w=*/1)), 0);
  EXPECT_EQ(half_space.Compare(HomoPoint2<>(/*x=*/10, /*y=*/100, /*w=*/2)), 0);
  // excluded
  EXPECT_LT(half_space.Compare(HomoPoint2<>(/*x=*/6, /*y=*/100, /*w=*/1)), 0);
  EXPECT_LT(half_space.Compare(HomoPoint2<>(/*x=*/11, /*y=*/100, /*w=*/2)), 0);
}

TEST(HalfSpace2, BuildFromPoints) {
  // Build from the line:
  // [5, 0], [6, -1]
  //
  // Anything with z<5 is included in the half space.
  HalfSpace2<> half_space(/*p1=*/Point2<>(5, 0),
                          /*p2=*/Point2<>(6, -1));

  EXPECT_TRUE(half_space.normal().IsSameDir(Vector2<>(-1, -1)));

  // included
  EXPECT_GT(half_space.Compare(Point2<>(/*x=*/3, /*y=*/3)), 0);
  // coincident
  EXPECT_EQ(half_space.Compare(Point2<>(/*x=*/2, /*y=*/3)), 0);
  // excluded
  EXPECT_LT(half_space.Compare(Point2<>(/*x=*/1, /*y=*/1)), 0);
}

TEST(HalfSpace2, HalfSpacesDistinct) {
  EXPECT_NE(HalfSpace2<>(/*x=*/0, /*y=*/0, /*dist=*/10),
            HalfSpace2<>(/*x=*/0, /*y=*/0, /*dist=*/-10));
}

template <int point2_bits>
void TestCorrectOutputBits() {
  using Builder = HalfSpace2FromPoint2Builder<point2_bits>;
  using HalfSpace2Rep = typename Builder::HalfSpace2Rep;
  // A half-space type with double the required bits.
  using HalfSpace2ExtraBits =
    HalfSpace2<HalfSpace2Rep::VectorInt::bits*2, HalfSpace2Rep::DistInt::bits*2>;
  using Point2Rep = typename Builder::Point2Rep;
  using VectorInt = typename HalfSpace2Rep::VectorInt;
  using DistInt = typename HalfSpace2Rep::DistInt;
  using BigIntRep = typename Point2Rep::BigIntRep;
  using NextSmallerVectorInt = BigInt<VectorInt::bits - 1>;
  using NextSmallerDistInt = BigInt<DistInt::bits - 1>;
  int up_to = 1;
  for (int i = 0; i < 4; ++i) {
    up_to *= 2;
  }
  EXPECT_GT(up_to, 1);
  VectorInt smallest_normal_coord[2] = {VectorInt(0),
                                        VectorInt(0)};
  VectorInt largest_normal_coord[2] = {VectorInt(0),
                                       VectorInt(0)};
  DistInt smallest_dist(0);
  DistInt largest_dist(0);
  for (int i = 0; i < up_to; ++i) {
    int remaining = i;
    Point2Rep p[2];
    for (int j = 0; j < 2; ++j) {
      BigIntRep coords[2];
      for (int k = 0; k < 2; ++k) {
        switch (remaining % 2) {
          case 0:
            coords[k] = BigIntRep::min_value();
            break;
          case 1:
            coords[k] = BigIntRep::max_value();
            break;
        }
        remaining /= 2;
      }
      p[j] = Point2Rep(coords[0], coords[1]);
    }
    HalfSpace2Rep half_space = Builder::Build(p[0], p[1]);
    HalfSpace2ExtraBits half_space_extra(p[0], p[1]);
    EXPECT_EQ(half_space.IsValid(), half_space_extra.IsValid());
    EXPECT_EQ(half_space, half_space_extra);
    EXPECT_TRUE(half_space.IsValidState());
    for (int j = 0; j < 2; ++j) {
      smallest_normal_coord[j] = std::min(smallest_normal_coord[j],
                                          half_space.normal().coords()[j]);
      largest_normal_coord[j] = std::max(largest_normal_coord[j],
                                         half_space.normal().coords()[j]);
    }
    smallest_dist = std::min(smallest_dist, half_space.d());
    largest_dist = std::max(largest_dist, half_space.d());
    ASSERT_GE(largest_dist, DistInt(0));
  }
  for (int j = 0; j < 2; ++j) {
    EXPECT_EQ(smallest_normal_coord[j], Builder::normal_component_min());
    EXPECT_EQ(largest_normal_coord[j], Builder::normal_component_max());

    EXPECT_LT(smallest_normal_coord[j], NextSmallerVectorInt::min_value());
    EXPECT_GT(largest_normal_coord[j], NextSmallerVectorInt::max_value());
  }
  EXPECT_LT(smallest_dist, DistInt(NextSmallerDistInt::min_value()));
  EXPECT_GT(largest_dist, DistInt(NextSmallerDistInt::max_value()));
  EXPECT_EQ(smallest_dist, Builder::dist_min());
  EXPECT_EQ(largest_dist, Builder::dist_max());
}

TEST(HalfSpace2FromPoint2Builder, CorrectOutputBits2) {
  TestCorrectOutputBits<2>();
}

TEST(HalfSpace2FromPoint2Builder, CorrectOutputBits32) {
  TestCorrectOutputBits<32>();
}

TEST(HalfSpace2FromPoint2Builder, CorrectOutputBits64) {
  TestCorrectOutputBits<64>();
}

}  // walnut
