#include "walnut/plucker_line.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

TEST(PluckerLine, FromPointsDirection) {
  const Point3 p1(1, 2, 3);
  const Point3 p2(5, 7, 11);
  PluckerLine line(p1, p2);

  EXPECT_GT((p2 - p1).Dot(line.d()), 0);
  EXPECT_LT((p1 - p2).Dot(line.d()), 0);
  EXPECT_EQ((p2 - p1).Cross(line.d()), Vector3::Zero());
}

TEST(PluckerLine, IsCoincidentPoint3) {
  const Point3 p1(1, 2, 3);
  const Point3 p2(5, 7, 11);
  PluckerLine line(p1, p2);

  EXPECT_TRUE(line.IsCoincident(p1));
  EXPECT_TRUE(line.IsCoincident(p2));

  const Point3 p3(p2 + (p2 - p1));
  EXPECT_TRUE(line.IsCoincident(p3));

  const HomoPoint3 doubled_p3(p3.vector_from_origin().Scale(2), BigInt(2));
  EXPECT_TRUE(line.IsCoincident(doubled_p3));

  const Point3 p4(p1 - (p2 - p1));
  EXPECT_TRUE(line.IsCoincident(p4));

  const Point3 p5(17, 23, 31);
  EXPECT_FALSE(line.IsCoincident(p5));

  const HomoPoint3 doubled_p5(p5.vector_from_origin().Scale(2), BigInt(2));
  EXPECT_FALSE(line.IsCoincident(doubled_p5));
}

TEST(PluckerLine, IsCoincidentThroughOrigin) {
  const Point3 p1(1, 2, 3);
  const Point3 p2(0, 0, 0);
  PluckerLine line(p1, p2);

  EXPECT_TRUE(line.IsCoincident(p1));
  EXPECT_TRUE(line.IsCoincident(p2));

  const Point3 p3(p2 + (p2 - p1));
  EXPECT_TRUE(line.IsCoincident(p3));
}

TEST(PluckerLine, IsCoincidentHalfSpace3) {
  const Point3 p1(1, 2, 3);
  const Point3 p2(5, 7, 11);
  PluckerLine line(p1, p2);

  const Point3 p3(6, 8, 14);
  HalfSpace3 a(p1, p2, p3);
  const Point3 p4(7, 9, 15);
  HalfSpace3 b(p1, p3, p4);

  EXPECT_TRUE(line.IsCoincident(a));
  EXPECT_TRUE(line.IsCoincident(-a));
  EXPECT_FALSE(line.IsCoincident(b));
}

TEST(PluckerLine, IsCoincidentOffsetParallelHalfSpace3) {
  const Point3 p1(0, 0, 0);
  const Point3 p2(1, 0, 0);
  PluckerLine line(p1, p2);

  HalfSpace3 y_perp = HalfSpace3::GetAxisAligned(/*dimension=*/1,
                                                 /*numerator=*/BigInt(1),
                                                 /*denominator=*/BigInt(1));
  EXPECT_FALSE(line.IsCoincident(y_perp));
}

TEST(PluckerLine, IsCoincidentHalfSpace3ThroughOrigin) {
  const Point3 p1(2, 2, 2);
  const Point3 p2(0, 0, 0);
  PluckerLine line(p1, p2);

  HalfSpace3 yz = HalfSpace3::GetAxisAligned(/*dimension=*/0,
                                             /*numerator=*/BigInt(0),
                                             /*denominator=*/BigInt(1));
  EXPECT_FALSE(line.IsCoincident(yz));
}

TEST(PluckerLine, EqualSameDir) {
  const Point3 p1(1, 2, 3);
  const Vector3 d(5, 7, 11);
  const PluckerLine line(p1, Point3(p1 + d.Scale(2)));

  EXPECT_EQ(line, line);

  const PluckerLine line2(p1 + d.Scale(3),
                            p1 + d.Scale(5));
  EXPECT_EQ(line, line2);
}

TEST(PluckerLine, NotEqualOppositeDir) {
  const Point3 p1(1, 2, 3);
  const Vector3 d(5, 7, 11);
  const PluckerLine line(p1, Point3(p1 + d.Scale(2)));

  const PluckerLine line2(p1 + d.Scale(5),
                            p1 + d.Scale(3));
  EXPECT_NE(line, line2);

  const PluckerLine line3(p1,
                            Point3(p1 + d.Scale(-3)));
  EXPECT_NE(line, line3);

  const PluckerLine line4(Point3(p1 + d.Scale(2)), p1);
  EXPECT_NE(line, line4);
  EXPECT_NE(line4, line);
}

TEST(PluckerLine, NotEqualWithZeroMY) {
  const Point3 p1(0, 0, 0);
  const Point3 p2(-2, -2, 0);
  const Point3 p3(-2, 0, 0);

  // { d={ -2, -2, 0 } m={ 0, 0, 0 } }
  const PluckerLine line(p1, p2);
  // { d={ 0, 2, 0 } m={ 0, 0, -4 } }
  const PluckerLine line2(p2, p3);
  EXPECT_NE(line, line2);
  EXPECT_NE(line2, line);
}

TEST(PluckerLine, ConstructFromPlanes) {
  const Point3 p1(1, 2, 3);
  const Point3 p2(5, 7, 11);
  const PluckerLine line_from_points(p1, p2);

  const Point3 p3(5, 7, 12);
  const Point3 p4(6, 7, 11);

  HalfSpace3 a(p1, p2, p3);
  HalfSpace3 b(p1, p2, p4);
  const PluckerLine line_from_planes(a, b);

  EXPECT_EQ(line_from_planes, line_from_points);

  EXPECT_TRUE(line_from_planes.IsCoincident(p1));
  EXPECT_TRUE(line_from_planes.IsCoincident(p2));
}

TEST(PluckerLine, ConstructFromPlanesOrientation) {
  const Point3 p1(1, 2, 3);
  const Point3 p2(5, 7, 11);
  const Point3 p3(5, 7, 12);
  const Point3 p4(6, 7, 11);

  HalfSpace3 a(p1, p2, p3);
  HalfSpace3 b(p1, p2, p4);
  HalfSpace3 neg_a(a);
  HalfSpace3 neg_b(b);
  neg_a.Negate();
  neg_b.Negate();

  const PluckerLine line_a_b(a, b);
  const PluckerLine line_neg_a_b(neg_a, b);
  const PluckerLine line_a_neg_b(a, neg_b);
  const PluckerLine line_neg_a_neg_b(neg_a, neg_b);
  const PluckerLine line_b_a(b, a);

  auto d = line_a_b.d();
  auto neg_d = line_a_b.d();
  neg_d.Negate();

  // Lines with opposite directions are not considered equal.
  EXPECT_NE(line_a_b, line_neg_a_b);

  EXPECT_EQ(line_a_b.d(), d);
  EXPECT_EQ(line_neg_a_b.d(), neg_d);
  EXPECT_EQ(line_a_neg_b.d(), neg_d);
  EXPECT_EQ(line_neg_a_neg_b.d(), d);
  EXPECT_EQ(line_b_a.d(), neg_d);
}

TEST(PluckerLine, ConstructFromHomoPoint3) {
  const Point3 p1(1, 2, 3);
  const Point3 p2(5, 7, 11);
  PluckerLine line(p1, p2);

  const HomoPoint3 homo_p1(1 * 11, 2 * 11, 3 * 11, 11);
  const HomoPoint3 homo_p2(5 * 13, 7 * 13, 11 * 13, 13);
  PluckerLine from_homo_points(homo_p1, homo_p2);

  const HomoPoint3 homo_p1_neg(1 * -11, 2 * -11, 3 * -11, -11);
  const HomoPoint3 homo_p2_neg(5 * -13, 7 * -13, 11 * -13, -13);
  PluckerLine from_homo_points2(homo_p1_neg, homo_p2);
  PluckerLine from_homo_points3(homo_p1_neg, homo_p2_neg);

  EXPECT_EQ(from_homo_points, line);
  EXPECT_TRUE(from_homo_points.d().IsSameDir(line.d()));
  EXPECT_EQ(from_homo_points2, line);
  EXPECT_TRUE(from_homo_points2.d().IsSameDir(line.d()));
  EXPECT_EQ(from_homo_points3, line);
  EXPECT_TRUE(from_homo_points3.d().IsSameDir(line.d()));
}

TEST(PluckerLine, FromPlanesDirection) {
  const Point3 p1(1, 2, 3);
  const Point3 p2(5, 7, 11);
  const PluckerLine line_from_points(p1, p2);

  const Point3 p3(5, 7, 12);
  const Point3 p4(6, 7, 11);

  HalfSpace3 a(p1, p2, p3);
  HalfSpace3 b(p1, p2, p4);
  const PluckerLine line(a, b);

  EXPECT_GT((p2 - p1).Dot(line.d()), 0);
  EXPECT_LT((p1 - p2).Dot(line.d()), 0);
  EXPECT_EQ((p2 - p1).Cross(line.d()), Vector3::Zero());
}

TEST(PluckerLine, IntersectPlane) {
  const Point3 p1(1, 2, 3);
  const Point3 p2(5, 7, 11);
  const PluckerLine line(p1, p2);

  const Point3 p3(6, 8, 12);
  const Point3 p4(6, 9, 11);

  EXPECT_FALSE(line.IsCoincident(p3));
  EXPECT_FALSE(line.IsCoincident(p4));

  HalfSpace3 plane(p2, p3, p4);

  auto intersect_point = line.Intersect(plane);
  EXPECT_EQ(intersect_point, HomoPoint3(p2));
  EXPECT_TRUE(line.IsCoincident(intersect_point));
}

template <int point3_bits>
void TestIntersectPlanes() {
  const BigInt min_int = BigInt::min_value(point3_bits - 1);
  const BigInt min_plus_1 = min_int + BigInt(1);
  const BigInt max_int = BigInt::max_value(point3_bits - 1);

  Point3 plane1_points[3] = {
    Point3{min_plus_1, min_int, min_int},
    Point3{min_int, max_int, min_int},
    Point3{min_int, min_int, max_int},
  };

  HalfSpace3 plane1(plane1_points[0], plane1_points[1], plane1_points[2]);

  HalfSpace3 plane2(Vector3(0, 1, 0), max_int);

  HalfSpace3 plane3(Vector3(0, 0, 1), max_int);

  const PluckerLine line(plane1, plane2);

  auto intersection = line.Intersect(plane3);
  EXPECT_TRUE(line.IsCoincident(intersection));
  EXPECT_TRUE(plane1.IsCoincident(intersection));
  EXPECT_TRUE(plane2.IsCoincident(intersection));
  EXPECT_TRUE(plane3.IsCoincident(intersection));
}

TEST(PluckerLine, IntersectPlanes64) {
  TestIntersectPlanes<64>();
}

TEST(PluckerLine, IntersectPlanes128) {
  TestIntersectPlanes<128>();
}

TEST(PluckerLine, Project2D) {
  const Point3 p1(17, 23, 31);
  const Point3 p2(131, 163, 197);
  const PluckerLine line(p1, p2);

  const Point3 p3(41, 103, 73);

  EXPECT_TRUE(line.IsCoincident(p1));
  EXPECT_TRUE(line.IsCoincident(p2));
  EXPECT_FALSE(line.IsCoincident(p3));

  for (int dimension = 0; dimension < 3; ++dimension) {
    auto line2d = line.Project2D(dimension);
    EXPECT_TRUE(line2d.IsCoincident(p1.DropDimension(dimension)))
      << "dimension=" << dimension;
    EXPECT_TRUE(line2d.IsCoincident(p2.DropDimension(dimension)))
      << "dimension=" << dimension;
    EXPECT_FALSE(line2d.IsCoincident(p3.DropDimension(dimension)))
      << "dimension=" << dimension;
  }
}

TEST(PluckerLine, Project2DDropYSideness) {
  const Point3 p1(0, 1, 2);
  const Point3 p2(1, 1, 2);

  const Point3 origin(0, 0, 0);

  {
    const PluckerLine line(p1, p2);
    auto line2d = line.Project2D(/*dimension=*/1);
    // The `line` goes from (0, 1, 2) to (1, 1, 2). After dropping the y
    // coordinate and swapping x and z, `line2d` goes from (2, 0) to (2, 1). A
    // clockwise quarter turn from that line points from (2, 0) towards the
    // origin.
    EXPECT_GT(line2d.Compare(origin.DropDimension(1)), 0);
  }

  {
    const PluckerLine line(p2, p1);
    auto line2d = line.Project2D(/*dimension=*/1);
    // The `line` goes from (1, 1, 2) to (0, 1, 2). After dropping the y
    // coordinate and swapping x and z, `line2d` goes from (2, 1) to (2, 0). A
    // clockwise quarter turn from that line points from (2, 0) away from the
    // origin.
    EXPECT_LT(line2d.Compare(origin.DropDimension(1)), 0);
  }
}

TEST(PluckerLine, Project2DDropZSideness) {
  const Point3 p1(0, 1, 1);
  const Point3 p2(1, 1, 1);

  const Point3 origin(0, 0, 0);

  {
    const PluckerLine line(p1, p2);
    auto line2d = line.Project2D(/*dimension=*/2);
    EXPECT_LT(line2d.Compare(origin.DropDimension(2)), 0);
  }

  {
    const PluckerLine line(p2, p1);
    auto line2d = line.Project2D(/*dimension=*/2);
    EXPECT_GT(line2d.Compare(origin.DropDimension(2)), 0);
  }
}

TEST(PluckerLine, ReduceAllIntMin) {
  PluckerLine original(
      /*d=*/Vector3(/*x=*/BigInt::min_value(255),
                    /*y=*/BigInt::min_value(255),
                    /*z=*/BigInt::min_value(255)),
      /*m=*/Vector3(/*x=*/BigInt::min_value(255),
                    /*y=*/BigInt::min_value(255),
                    /*z=*/BigInt::min_value(255)));

  PluckerLine reduced = original;
  reduced.Reduce();
  EXPECT_EQ(reduced, original);
  EXPECT_EQ(reduced.d().x(), -1);
}

TEST(PluckerLine, Project2DThenExtrude) {
  const Point3 p1(17, 23, 31);
  const Point3 p2(131, 163, 197);
  const PluckerLine line(p1, p2);

  const Point3 p3(41, 103, 73);

  EXPECT_TRUE(line.IsCoincident(p1));
  EXPECT_TRUE(line.IsCoincident(p2));
  EXPECT_FALSE(line.IsCoincident(p3));

  for (int dimension = 0; dimension < 3; ++dimension) {
    const HalfSpace2 line2d = line.Project2D(dimension);
    HalfSpace3 extruded(line2d, dimension);
    EXPECT_TRUE(extruded.IsCoincident(p1))
      << "dimension=" << dimension << std::endl
      << "line2d=" << line2d << std::endl
      << "extruded=" << extruded << std::endl
      << "p1=" << p1 << std::endl;
    EXPECT_TRUE(extruded.IsCoincident(p2))
      << "dimension=" << dimension << std::endl
      << "line2d=" << line2d << std::endl
      << "extruded=" << extruded << std::endl
      << "p2=" << p2;
    EXPECT_FALSE(extruded.IsCoincident(p3))
      << "dimension=" << dimension << std::endl
      << "line2d=" << line2d << std::endl
      << "extruded=" << extruded << std::endl
      << "p3=" << p3;
  }
}

}  // walnut
