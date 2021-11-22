#include "walnut/half_space3.h"

#include <random>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

using testing::UnorderedElementsAreArray;

TEST(HalfSpace3, ComparePoint3) {
  // Anything with x>5 is included in the half space.
  HalfSpace3 plane(/*normal=*/Vector3(/*x=*/2, /*y=*/0, /*z=*/0),
                     /*dist=*/BigInt(10));

  EXPECT_TRUE(plane.normal().IsSameDir(Vector3(1, 0, 0)));

  // excluded
  EXPECT_LT(plane.Compare(Point3(/*x=*/1, /*y=*/100, /*z=*/100)), 0);
  // coincident
  EXPECT_EQ(plane.Compare(Point3(/*x=*/5, /*y=*/100, /*z=*/100)), 0);
  // included
  EXPECT_GT(plane.Compare(Point3(/*x=*/6, /*y=*/100, /*z=*/100)), 0);
}

TEST(HalfSpace3, CompareHomoPoint3) {
  // Anything with x>5 is included in the half space.
  HalfSpace3 plane(/*normal=*/Vector3(/*x=*/2, /*y=*/0, /*z=*/0),
                     /*dist=*/BigInt(10));

  // excluded
  EXPECT_LT(plane.Compare(HomoPoint3(/*x=*/1, /*y=*/100, /*z=*/100,
                                       /*w=*/1)), 0);
  EXPECT_LT(plane.Compare(HomoPoint3(/*x=*/9, /*y=*/100, /*z=*/100,
                                       /*w=*/2)), 0);
  // coincident
  EXPECT_EQ(plane.Compare(HomoPoint3(/*x=*/5, /*y=*/100, /*z=*/100,
                                       /*w=*/1)), 0);
  EXPECT_EQ(plane.Compare(HomoPoint3(/*x=*/10, /*y=*/100, /*z=*/100,
                                       /*w=*/2)), 0);
  // included
  EXPECT_GT(plane.Compare(HomoPoint3(/*x=*/6, /*y=*/100, /*z=*/100,
                                       /*w=*/1)), 0);
  EXPECT_GT(plane.Compare(HomoPoint3(/*x=*/11, /*y=*/100, /*z=*/100,
                                       /*w=*/2)), 0);
}

TEST(HalfSpace3, ComparePosHomoPoint3NegDist) {
  // Anything with x<5 is included in the half space.
  HalfSpace3 plane(/*normal=*/Vector3(/*x=*/-2, /*y=*/0, /*z=*/0),
                     /*dist=*/BigInt(-10));

  // included
  EXPECT_GT(plane.Compare(HomoPoint3(/*x=*/1, /*y=*/100, /*z=*/100,
                                       /*w=*/1)), 0);
  // excluded
  EXPECT_LT(plane.Compare(HomoPoint3(/*x=*/6, /*y=*/100, /*z=*/100,
                                       /*w=*/1)), 0);
}

TEST(HalfSpace3, CompareNegHomoPoint3PosDist) {
  // Anything with x>5 is included in the half space.
  HalfSpace3 plane(/*normal=*/Vector3(/*x=*/2, /*y=*/0, /*z=*/0),
                     /*dist=*/BigInt(10));

  // excluded
  EXPECT_LT(plane.Compare(HomoPoint3(/*x=*/1, /*y=*/100, /*z=*/100,
                                       /*w=*/-1)), 0);
  // included
  EXPECT_GT(plane.Compare(HomoPoint3(/*x=*/-6, /*y=*/100, /*z=*/100,
                                       /*w=*/-1)), 0);
}

TEST(HalfSpace3, CompareNegHomoPoint3NegDist) {
  // Anything with x<5 is included in the half space.
  HalfSpace3 plane(/*normal=*/Vector3(/*x=*/-2, /*y=*/0, /*z=*/0),
                     /*dist=*/BigInt(-10));

  // included
  EXPECT_GT(plane.Compare(HomoPoint3(/*x=*/1, /*y=*/100, /*z=*/100,
                                       /*w=*/-1)), 0);
  // excluded
  EXPECT_LT(plane.Compare(HomoPoint3(/*x=*/-6, /*y=*/100, /*z=*/100,
                                       /*w=*/-1)), 0);
}

TEST(HalfSpace3, CompareParallelPlane) {
  // These are in sorted order.
  std::vector<HalfSpace3> positive_half_spaces = {
    HalfSpace3(/*normal=*/Vector3(/*x=*/1, /*y=*/0, /*z=*/0),
                 /*dist=*/BigInt(-2)),
    HalfSpace3(/*normal=*/Vector3(/*x=*/1, /*y=*/0, /*z=*/0),
                 /*dist=*/BigInt(-1)),
    HalfSpace3(/*normal=*/Vector3(/*x=*/2, /*y=*/0, /*z=*/0),
                 /*dist=*/BigInt(-1)),
    HalfSpace3(/*normal=*/Vector3(/*x=*/1, /*y=*/0, /*z=*/0),
                 /*dist=*/BigInt(0)),
    HalfSpace3(/*normal=*/Vector3(/*x=*/2, /*y=*/0, /*z=*/0),
                 /*dist=*/BigInt(1)),
    HalfSpace3(/*normal=*/Vector3(/*x=*/1, /*y=*/0, /*z=*/0),
                 /*dist=*/BigInt(1)),
    HalfSpace3(/*normal=*/Vector3(/*x=*/1, /*y=*/0, /*z=*/0),
                 /*dist=*/BigInt(2)),
  };
  std::vector<HalfSpace3> negative_half_spaces;
  for (const HalfSpace3& h : positive_half_spaces) {
    negative_half_spaces.push_back(-h);
  }

  for (size_t i = 0; i < positive_half_spaces.size(); ++i) {
    for (size_t j = 0; j < positive_half_spaces.size(); ++j) {
      if (i < j) {
        EXPECT_LT(positive_half_spaces[i].Compare(positive_half_spaces[j],
                                                  /*nonzero_dimension=*/0), 0)
          << "i=" << i << " j=" << j;
        EXPECT_LT(positive_half_spaces[i].Compare(negative_half_spaces[j],
                                                  /*nonzero_dimension=*/0), 0)
          << "i=" << i << " j=" << j;
        EXPECT_GT(negative_half_spaces[i].Compare(positive_half_spaces[j],
                                                  /*nonzero_dimension=*/0), 0)
          << "i=" << i << " j=" << j;
        EXPECT_GT(negative_half_spaces[i].Compare(negative_half_spaces[j],
                                                  /*nonzero_dimension=*/0), 0)
          << "i=" << i << " j=" << j;
      } else if (i == j) {
        EXPECT_EQ(positive_half_spaces[i].Compare(positive_half_spaces[j],
                                                  /*nonzero_dimension=*/0), 0);
        EXPECT_EQ(positive_half_spaces[i].Compare(negative_half_spaces[j],
                                                  /*nonzero_dimension=*/0), 0);
        EXPECT_EQ(negative_half_spaces[i].Compare(positive_half_spaces[j],
                                                  /*nonzero_dimension=*/0), 0);
        EXPECT_EQ(negative_half_spaces[i].Compare(negative_half_spaces[j],
                                                  /*nonzero_dimension=*/0), 0);
      } else {
        EXPECT_GT(positive_half_spaces[i].Compare(positive_half_spaces[j],
                                                  /*nonzero_dimension=*/0), 0);
        EXPECT_GT(positive_half_spaces[i].Compare(negative_half_spaces[j],
                                                  /*nonzero_dimension=*/0), 0);
        EXPECT_LT(negative_half_spaces[i].Compare(positive_half_spaces[j],
                                                  /*nonzero_dimension=*/0), 0);
        EXPECT_LT(negative_half_spaces[i].Compare(negative_half_spaces[j],
                                                  /*nonzero_dimension=*/0), 0);
      }
    }
  }
}

TEST(HalfSpace3, BuildFromPoints) {
  // Build from the triangle:
  // [0, 0, 5], [1, 0, 5], [0, 1, 5]
  //
  // Anything with z<5 is included in the half space.
  HalfSpace3 plane(/*p1=*/Point3(0, 0, 5),
                /*p2=*/Point3(1, 0, 5),
                /*p3=*/Point3(0, 1, 5));

  EXPECT_TRUE(plane.normal().IsSameDir(Vector3(0, 0, 1)));

  // excluded
  EXPECT_LT(plane.Compare(Point3(/*x=*/100, /*y=*/100, /*z=*/1)), 0);
  // coincident
  EXPECT_EQ(plane.Compare(Point3(/*x=*/500, /*y=*/100, /*z=*/5)), 0);
  // included
  EXPECT_GT(plane.Compare(Point3(/*x=*/600, /*y=*/100, /*z=*/6)), 0);
}

TEST(HalfSpace3, BuildFromHomoPoints) {
  Point3 p[3] = {
    Point3(1, 2, 3),
    Point3(5, 7, 11),
    Point3(13, 17, 19),
  };
  HalfSpace3 from_point3(p[0], p[1], p[2]);

  for (const int scale : {-2, -1, 1, 2}) {
    for (int i = 0; i < 3; ++i) {
      HomoPoint3 scaled_p[3] = {p[0], p[1], p[2]};
      scaled_p[i] = HomoPoint3(p[i].x() * scale,
                                 p[i].y() * scale,
                                 p[i].z() * scale,
                                 BigInt(scale));
      HalfSpace3 from_homo_point3(scaled_p[0], scaled_p[1], scaled_p[2]);
      EXPECT_EQ(from_homo_point3, from_point3)
        << "scale=" << scale << ", scaled_index=" << i;
      for (int j = 0; j < 3; ++j) {
        EXPECT_EQ(from_point3.Compare(scaled_p[j]), 0);
        EXPECT_EQ(from_homo_point3.Compare(scaled_p[j]), 0);
      }
    }
  }
}

TEST(HalfSpace3, HalfSpacesDistinct) {
  EXPECT_NE(HalfSpace3(/*x=*/0, /*y=*/0, /*z=*/1, /*dist=*/10),
            HalfSpace3(/*x=*/0, /*y=*/0, /*z=*/-1, /*dist=*/-10));
}

TEST(HalfSpace3, ReduceAllPos) {
  HalfSpace3 plane(/*normal=*/Vector3(/*x=*/10, /*y=*/5, /*z=*/15),
                     /*dist=*/BigInt(20));

  HalfSpace3 reduced(plane);
  reduced.Reduce();
  EXPECT_EQ(plane, reduced);
  EXPECT_EQ(reduced.x(), 2);
  EXPECT_EQ(reduced.y(), 1);
  EXPECT_EQ(reduced.z(), 3);
  EXPECT_EQ(reduced.d(), 4);
}

TEST(HalfSpace3, ReduceAllNeg) {
  HalfSpace3 plane(/*normal=*/Vector3(/*x=*/-10, /*y=*/-5, /*z=*/-15),
                     /*dist=*/BigInt(-20));

  HalfSpace3 reduced(plane);
  reduced.Reduce();
  EXPECT_EQ(plane, reduced);
  EXPECT_EQ(reduced.x(), -2);
  EXPECT_EQ(reduced.y(), -1);
  EXPECT_EQ(reduced.z(), -3);
  EXPECT_EQ(reduced.d(), -4);
}

TEST(HalfSpace3, ExtrudeHalfSpace2ZPosDist) {
  HalfSpace2 plane2d(Point2(1, 5), Point2(2, 7));
  HalfSpace3 plane(plane2d, /*add_dimension=*/2);
  HalfSpace3 expected(Point3(1, 5, 0), Point3(2, 7, 0), Point3(1, 5, -1));

  EXPECT_EQ(plane, expected);
  EXPECT_GT(plane2d.d(), 0);
  EXPECT_GT(plane.d(), 0);
}

TEST(HalfSpace3, ExtrudeHalfSpace2ZNegDist) {
  HalfSpace2 plane2d(Point2(2, 7), Point2(1, 5));
  HalfSpace3 plane(plane2d, /*add_dimension=*/2);
  HalfSpace3 expected(Point3(2, 7, 0), Point3(1, 5, 0), Point3(1, 5, -1));

  EXPECT_EQ(plane, expected);
  EXPECT_LT(plane2d.d(), 0);
  EXPECT_LT(plane.d(), 0);
}

TEST(HalfSpace3, ExtrudeHalfSpace2Y) {
  HalfSpace2 plane2d(Point2(1, 5), Point2(2, 7));
  HalfSpace3 plane(plane2d, /*add_dimension=*/1);
  HalfSpace3 expected(Point3(5, 0, 1), Point3(7, 0, 2), Point3(5, -1, 1));

  EXPECT_EQ(plane, expected);
  EXPECT_GT(plane2d.d(), 0);
  EXPECT_GT(plane.d(), 0);
}

TEST(HalfSpace3, ExtrudeHalfSpace2X) {
  HalfSpace2 plane2d(Point2(1, 5), Point2(2, 7));
  HalfSpace3 plane(plane2d, /*add_dimension=*/0);
  HalfSpace3 expected(Point3(0, 1, 5), Point3(0, 2, 7), Point3(-1, 1, 5));

  EXPECT_EQ(plane, expected);
  EXPECT_GT(plane2d.d(), 0);
  EXPECT_GT(plane.d(), 0);
}

TEST(HalfSpace3, ProjectOntoPlaneAddX) {
  HalfSpace3 plane(/*x=*/1, /*y=*/2, /*z=*/3, /*dist=*/5);

  const HomoPoint2 original = HomoPoint2(7, 11, 13);
  const HomoPoint3 projected =
    plane.ProjectOntoPlane(original, /*add_dimension=*/0);
  EXPECT_EQ(projected.DropDimension(0), original);
  EXPECT_TRUE(plane.IsCoincident(projected));
}

TEST(HalfSpace3, ProjectOntoPlaneAddZ) {
  HalfSpace3 plane(/*x=*/1, /*y=*/2, /*z=*/3, /*dist=*/5);

  const HomoPoint2 original = HomoPoint2(7, 11, 13);
  const HomoPoint3 projected =
    plane.ProjectOntoPlane(original, /*add_dimension=*/2);
  EXPECT_EQ(projected.DropDimension(2), original);
  EXPECT_TRUE(plane.IsCoincident(projected));
}

TEST(HalfSpace3, ConstructFromVectorAndHomoPoint3) {
  Vector3 v(/*x=*/2, /*y=*/3, /*z=*/5);
  HomoPoint3 p(/*x=*/7, /*y=*/11, /*z=*/13, /*w=*/17);
  HalfSpace3 plane(v, p);
  EXPECT_TRUE(plane.normal().IsSameDir(v));
  EXPECT_TRUE(plane.IsCoincident(p));
}

TEST(HalfSpace3, ConstructFromVectorAndHomoPoint3NegDenom) {
  Vector3 v(/*x=*/2, /*y=*/3, /*z=*/5);
  HomoPoint3 p(/*x=*/7, /*y=*/11, /*z=*/13, /*w=*/-17);
  HalfSpace3 plane(v, p);
  EXPECT_TRUE(plane.normal().IsSameDir(v));
  EXPECT_TRUE(plane.IsCoincident(p));
}

TEST(HalfSpace3, HalfSpace3CompareStoreInSet) {
  std::vector<HalfSpace3> unique_halfspaces{
    HalfSpace3(1, 1, 1, 0),
    HalfSpace3(1, 1, 1, 1),
    HalfSpace3(1, 1, 1, -1),
    HalfSpace3(-1, -1, -1, 0),
    HalfSpace3(-1, -1, -1, 1),
    HalfSpace3(-1, -1, -1, -1),
    HalfSpace3(1, 0, 0, -1),
    HalfSpace3(1, 0, 0, 0),
    HalfSpace3(1, 0, 0, 1),
    HalfSpace3(0, 0, 1, -1),
    HalfSpace3(0, 0, 1, 0),
    HalfSpace3(0, 0, 1, 1),
    HalfSpace3(1, 2, 3, 5),
    HalfSpace3(-1, -2, -3, -5),
    HalfSpace3(0, 1, 0, 0),
    HalfSpace3(0, -1, 0, 0),
  };

  std::mt19937 gen;
  for (int i = 0; i < 100; ++i) {
    using Set = std::set<HalfSpace3, HalfSpace3Compare>;
    Set added;
    std::vector<HalfSpace3> to_add = unique_halfspaces;
    while (!to_add.empty()) {
      size_t selected = gen() % to_add.size();
      int multiple = (gen() % 3) + 1;
      HalfSpace3 multiplied(to_add[selected].x() * multiple,
                            to_add[selected].y() * multiple,
                            to_add[selected].z() * multiple,
                            to_add[selected].d() * multiple);
      EXPECT_EQ(multiplied, to_add[selected]);
      EXPECT_TRUE(added.insert(multiplied).second);
      to_add.erase(to_add.begin() + selected);
    }

    EXPECT_THAT(added, UnorderedElementsAreArray(unique_halfspaces));
  }
}

TEST(HalfSpace3, GetAxisAlignedPosDenom) {
  HalfSpace3 half_space = HalfSpace3::GetAxisAligned(/*dimension=*/0,
                                                     /*numerator=*/BigInt(2),
                                                     /*denominator=*/BigInt(3));
  EXPECT_LT(half_space.Compare(Point3(0, 0, 0)), 0);
  EXPECT_EQ(half_space.Compare(HomoPoint3(/*x=*/2, /*y=*/100, /*z=*/100,
                                          /*w=*/3)), 0);
  EXPECT_GT(half_space.Compare(Point3(1, 0, 0)), 0);
}

TEST(HalfSpace3, GetAxisAlignedNegDenom) {
  HalfSpace3 half_space = HalfSpace3::GetAxisAligned(/*dimension=*/0,
                                                     /*numerator=*/BigInt(-2),
                                                     /*denominator=*/BigInt(-3));
  EXPECT_GT(half_space.Compare(Point3(0, 0, 0)), 0);
  EXPECT_EQ(half_space.Compare(HomoPoint3(/*x=*/2, /*y=*/100, /*z=*/100,
                                          /*w=*/3)), 0);
  EXPECT_LT(half_space.Compare(Point3(1, 0, 0)), 0);
}

}  // walnut
