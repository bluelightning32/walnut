#include "walnut/half_space3.h"

#include "gtest/gtest.h"

namespace walnut {

TEST(HalfSpace3, ComparePoint3) {
  // Anything with x>5 is included in the half space.
  HalfSpace3<> plane(/*normal=*/Vector3<>(/*x=*/2, /*y=*/0, /*z=*/0),
                     /*dist=*/BigInt<32>(10));

  EXPECT_TRUE(plane.normal().IsSameDir(Vector3<>(1, 0, 0)));

  // excluded
  EXPECT_LT(plane.Compare(Point3<>(/*x=*/1, /*y=*/100, /*z=*/100)), 0);
  // coincident
  EXPECT_EQ(plane.Compare(Point3<>(/*x=*/5, /*y=*/100, /*z=*/100)), 0);
  // included
  EXPECT_GT(plane.Compare(Point3<>(/*x=*/6, /*y=*/100, /*z=*/100)), 0);
}

TEST(HalfSpace3, CompareHomoPoint3) {
  // Anything with x>5 is included in the half space.
  HalfSpace3<> plane(/*normal=*/Vector3<>(/*x=*/2, /*y=*/0, /*z=*/0),
                     /*dist=*/BigInt<32>(10));

  // excluded
  EXPECT_LT(plane.Compare(HomoPoint3<>(/*x=*/1, /*y=*/100, /*z=*/100, /*w=*/1)), 0);
  EXPECT_LT(plane.Compare(HomoPoint3<>(/*x=*/9, /*y=*/100, /*z=*/100, /*w=*/2)), 0);
  // coincident
  EXPECT_EQ(plane.Compare(HomoPoint3<>(/*x=*/5, /*y=*/100, /*z=*/100, /*w=*/1)), 0);
  EXPECT_EQ(plane.Compare(HomoPoint3<>(/*x=*/10, /*y=*/100, /*z=*/100, /*w=*/2)), 0);
  // included
  EXPECT_GT(plane.Compare(HomoPoint3<>(/*x=*/6, /*y=*/100, /*z=*/100, /*w=*/1)), 0);
  EXPECT_GT(plane.Compare(HomoPoint3<>(/*x=*/11, /*y=*/100, /*z=*/100, /*w=*/2)), 0);
}

TEST(HalfSpace3, ComparePosHomoPoint3NegDist) {
  // Anything with x<5 is included in the half space.
  HalfSpace3<> plane(/*normal=*/Vector3<>(/*x=*/-2, /*y=*/0, /*z=*/0),
                     /*dist=*/BigInt<32>(-10));

  // included
  EXPECT_GT(plane.Compare(HomoPoint3<>(/*x=*/1, /*y=*/100, /*z=*/100, /*w=*/1)), 0);
  // excluded
  EXPECT_LT(plane.Compare(HomoPoint3<>(/*x=*/6, /*y=*/100, /*z=*/100, /*w=*/1)), 0);
}

TEST(HalfSpace3, CompareNegHomoPoint3PosDist) {
  // Anything with x>5 is included in the half space.
  HalfSpace3<> plane(/*normal=*/Vector3<>(/*x=*/2, /*y=*/0, /*z=*/0),
                     /*dist=*/BigInt<32>(10));

  // excluded
  EXPECT_LT(plane.Compare(HomoPoint3<>(/*x=*/1, /*y=*/100, /*z=*/100, /*w=*/-1)), 0);
  // included
  EXPECT_GT(plane.Compare(HomoPoint3<>(/*x=*/-6, /*y=*/100, /*z=*/100, /*w=*/-1)), 0);
}

TEST(HalfSpace3, CompareNegHomoPoint3NegDist) {
  // Anything with x<5 is included in the half space.
  HalfSpace3<> plane(/*normal=*/Vector3<>(/*x=*/-2, /*y=*/0, /*z=*/0),
                     /*dist=*/BigInt<32>(-10));

  // included
  EXPECT_GT(plane.Compare(HomoPoint3<>(/*x=*/1, /*y=*/100, /*z=*/100, /*w=*/-1)), 0);
  // excluded
  EXPECT_LT(plane.Compare(HomoPoint3<>(/*x=*/-6, /*y=*/100, /*z=*/100, /*w=*/-1)), 0);
}

TEST(HalfSpace3, BuildFromPoints) {
  // Build from the triangle:
  // [0, 0, 5], [1, 0, 5], [0, 1, 5]
  //
  // Anything with z<5 is included in the half space.
  HalfSpace3<> plane(/*p1=*/Point3<>(0, 0, 5),
                /*p2=*/Point3<>(1, 0, 5),
                /*p3=*/Point3<>(0, 1, 5));

  EXPECT_TRUE(plane.normal().IsSameDir(Vector3<>(0, 0, 1)));

  // excluded
  EXPECT_LT(plane.Compare(Point3<>(/*x=*/100, /*y=*/100, /*z=*/1)), 0);
  // coincident
  EXPECT_EQ(plane.Compare(Point3<>(/*x=*/500, /*y=*/100, /*z=*/5)), 0);
  // included
  EXPECT_GT(plane.Compare(Point3<>(/*x=*/600, /*y=*/100, /*z=*/6)), 0);
}

TEST(HalfSpace3, HalfSpacesDistinct) {
  EXPECT_NE(HalfSpace3<>(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10),
            HalfSpace3<>(/*x=*/0, /*y=*/0, /*z=*/-1, /*dist=*/-10));
}

template <int point3_bits>
void TestCorrectOutputBits() {
  using Builder = HalfSpace3FromPoint3Builder<point3_bits>;
  using HalfSpace3Rep = typename Builder::HalfSpace3Rep;
  // A plane type with double the required bits.
  using HalfSpace3ExtraBits =
    HalfSpace3<HalfSpace3Rep::VectorInt::bits*2, HalfSpace3Rep::DistInt::bits*2>;
  using Point3Rep = typename Builder::Point3Rep;
  using VectorInt = typename HalfSpace3Rep::VectorInt;
  using DistInt = typename HalfSpace3Rep::DistInt;
  using BigIntRep = typename Point3Rep::BigIntRep;
  using NextSmallerVectorInt = BigInt<VectorInt::bits - 1>;
  using NextSmallerDistInt = BigInt<DistInt::bits - 1>;
  int up_to = 1;
  for (int i = 0; i < 9; ++i) {
    up_to *= 2;
  }
  EXPECT_GT(up_to, 1);
  VectorInt smallest_normal_coord[3] = {VectorInt(0),
                                        VectorInt(0),
                                        VectorInt(0)};
  VectorInt largest_normal_coord[3] = {VectorInt(0),
                                       VectorInt(0),
                                       VectorInt(0)};
  DistInt smallest_dist(0);
  DistInt largest_dist(0);
  for (int i = 0; i < up_to; ++i) {
    int remaining = i;
    Point3Rep p[3];
    for (int j = 0; j < 3; ++j) {
      BigIntRep coords[3];
      for (int k = 0; k < 3; ++k) {
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
      p[j] = Point3Rep(coords[0], coords[1], coords[2]);
    }
    HalfSpace3Rep plane = Builder::Build(p[0], p[1], p[2]);
    HalfSpace3ExtraBits plane_extra(p[0], p[1], p[2]);
    EXPECT_EQ(plane.IsValid(), plane_extra.IsValid());
    EXPECT_EQ(plane, plane_extra);
    EXPECT_TRUE(plane.IsValidState());
    for (int j = 0; j < 3; ++j) {
      smallest_normal_coord[j] = std::min(smallest_normal_coord[j],
                                          plane.normal().coords()[j]);
      largest_normal_coord[j] = std::max(largest_normal_coord[j],
                                         plane.normal().coords()[j]);
    }
    smallest_dist = std::min(smallest_dist, plane.d());
    largest_dist = std::max(largest_dist, plane.d());
    ASSERT_GE(largest_dist, DistInt(0));
  }
  for (int j = 0; j < 3; ++j) {
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

TEST(HalfSpace3FromPoint3Builder, CorrectOutputBits2) {
  TestCorrectOutputBits<2>();
}

TEST(HalfSpace3FromPoint3Builder, CorrectOutputBits32) {
  TestCorrectOutputBits<32>();
}

TEST(HalfSpace3FromPoint3Builder, CorrectOutputBits64) {
  TestCorrectOutputBits<64>();
}

}  // walnut
