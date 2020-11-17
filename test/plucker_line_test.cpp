#include "walnut/plucker_line.h"

#include "gtest/gtest.h"

namespace walnut {

TEST(PluckerLine, FromPointsDirection) {
  const Point3<> p1(1, 2, 3);
  const Point3<> p2(5, 7, 11);
  PluckerLine<> line(p1, p2);

  EXPECT_GT((p2 - p1).Dot(line.d()), 0);
  EXPECT_LT((p1 - p2).Dot(line.d()), 0);
  EXPECT_EQ((p2 - p1).Cross(line.d()), Vector3<>::Zero());
}

TEST(PluckerLine, IsCoincidentPoint3) {
  const Point3<> p1(1, 2, 3);
  const Point3<> p2(5, 7, 11);
  PluckerLine<> line(p1, p2);

  EXPECT_TRUE(line.IsCoincident(p1));
  EXPECT_TRUE(line.IsCoincident(p2));

  const Point3<> p3(p2 + (p2 - p1));
  EXPECT_TRUE(line.IsCoincident(p3));

  const HomoPoint3<> doubled_p3(p3.vector_from_origin().Scale(2), BigInt<8>(2));
  EXPECT_TRUE(line.IsCoincident(doubled_p3));

  const Point3<> p4(p1 - (p2 - p1));
  EXPECT_TRUE(line.IsCoincident(p4));

  const Point3<> p5(17, 23, 31);
  EXPECT_FALSE(line.IsCoincident(p5));

  const HomoPoint3<> doubled_p5(p5.vector_from_origin().Scale(2), BigInt<8>(2));
  EXPECT_FALSE(line.IsCoincident(doubled_p5));
}

TEST(PluckerLine, IsCoincidentThroughOrigin) {
  const Point3<> p1(1, 2, 3);
  const Point3<> p2(0, 0, 0);
  PluckerLine<> line(p1, p2);

  EXPECT_TRUE(line.IsCoincident(p1));
  EXPECT_TRUE(line.IsCoincident(p2));

  const Point3<> p3(p2 + (p2 - p1));
  EXPECT_TRUE(line.IsCoincident(p3));
}

TEST(PluckerLine, IsCoincidentHalfSpace3) {
  const Point3<> p1(1, 2, 3);
  const Point3<> p2(5, 7, 11);
  PluckerLine<> line(p1, p2);

  const Point3<> p3(6, 8, 14);
  HalfSpace3<> a(p1, p2, p3);
  const Point3<> p4(7, 9, 15);
  HalfSpace3<> b(p1, p3, p4);

  EXPECT_TRUE(line.IsCoincident(a));
  EXPECT_FALSE(line.IsCoincident(b));
}

TEST(PluckerLine, Equality) {
  const Point3<> p1(1, 2, 3);
  const Vector3<> d(5, 7, 11);
  const PluckerLine<> line(p1, Point3<>(p1 + d.Scale(2)));

  const PluckerLine<> line2(p1 + d.Scale(5),
                            p1 + d.Scale(3));
  EXPECT_EQ(line, line2);

  const PluckerLine<> line3(p1,
                            Point3<>(p1 + d.Scale(-3)));
  EXPECT_EQ(line, line3);

  const PluckerLine<> line4(Point3<>(p1 + d.Scale(2)), p1);
  EXPECT_EQ(line, line4);
  EXPECT_EQ(line4, line);
}

TEST(PluckerLine, ConstructFromPlanes) {
  const Point3<> p1(1, 2, 3);
  const Point3<> p2(5, 7, 11);
  const PluckerLine<> line_from_points(p1, p2);

  const Point3<> p3(5, 7, 12);
  const Point3<> p4(6, 7, 11);

  HalfSpace3<> a(p1, p2, p3);
  HalfSpace3<> b(p1, p2, p4);
  const PluckerLine<> line_from_planes(a, b);

  EXPECT_EQ(line_from_planes, line_from_points);

  EXPECT_TRUE(line_from_planes.IsCoincident(p1));
  EXPECT_TRUE(line_from_planes.IsCoincident(p2));
}

TEST(PluckerLine, ConstructFromPlanesOrientation) {
  const Point3<> p1(1, 2, 3);
  const Point3<> p2(5, 7, 11);
  const Point3<> p3(5, 7, 12);
  const Point3<> p4(6, 7, 11);

  HalfSpace3<> a(p1, p2, p3);
  HalfSpace3<> b(p1, p2, p4);
  HalfSpace3<> neg_a(a);
  HalfSpace3<> neg_b(b);
  neg_a.Negate();
  neg_b.Negate();

  const PluckerLine<> line_a_b(a, b);
  const PluckerLine<> line_neg_a_b(neg_a, b);
  const PluckerLine<> line_a_neg_b(a, neg_b);
  const PluckerLine<> line_neg_a_neg_b(neg_a, neg_b);
  const PluckerLine<> line_b_a(b, a);

  auto d = line_a_b.d();
  auto neg_d = line_a_b.d();
  neg_d.Negate();

  // Even lines with opposite directions are considered equal.
  EXPECT_EQ(line_a_b, line_neg_a_b);

  EXPECT_EQ(line_a_b.d(), d);
  EXPECT_EQ(line_neg_a_b.d(), neg_d);
  EXPECT_EQ(line_a_neg_b.d(), neg_d);
  EXPECT_EQ(line_neg_a_neg_b.d(), d);
  EXPECT_EQ(line_b_a.d(), neg_d);
}

TEST(PluckerLine, ConstructFromHomoPoint3) {
  const Point3<> p1(1, 2, 3);
  const Point3<> p2(5, 7, 11);
  PluckerLine<> line(p1, p2);

  const HomoPoint3<> homo_p1(1 * 11, 2 * 11, 3 * 11, 11);
  const HomoPoint3<> homo_p2(5 * 13, 7 * 13, 11 * 13, 13);
  PluckerLine<> from_homo_points(homo_p1, homo_p2);

  const HomoPoint3<> homo_p1_neg(1 * -11, 2 * -11, 3 * -11, -11);
  const HomoPoint3<> homo_p2_neg(5 * -13, 7 * -13, 11 * -13, -13);
  PluckerLine<> from_homo_points2(homo_p1_neg, homo_p2);
  PluckerLine<> from_homo_points3(homo_p1_neg, homo_p2_neg);

  EXPECT_EQ(from_homo_points, line);
  EXPECT_TRUE(from_homo_points.d().IsSameDir(line.d()));
  EXPECT_EQ(from_homo_points2, line);
  EXPECT_TRUE(from_homo_points2.d().IsSameDir(line.d()));
  EXPECT_EQ(from_homo_points3, line);
  EXPECT_TRUE(from_homo_points3.d().IsSameDir(line.d()));
}

TEST(PluckerLine, FromPlanesDirection) {
  const Point3<> p1(1, 2, 3);
  const Point3<> p2(5, 7, 11);
  const PluckerLine<> line_from_points(p1, p2);

  const Point3<> p3(5, 7, 12);
  const Point3<> p4(6, 7, 11);

  HalfSpace3<> a(p1, p2, p3);
  HalfSpace3<> b(p1, p2, p4);
  const PluckerLine<> line(a, b);

  EXPECT_GT((p2 - p1).Dot(line.d()), 0);
  EXPECT_LT((p1 - p2).Dot(line.d()), 0);
  EXPECT_EQ((p2 - p1).Cross(line.d()), Vector3<>::Zero());
}

TEST(PluckerLine, IntersectPlane) {
  const Point3<> p1(1, 2, 3);
  const Point3<> p2(5, 7, 11);
  const PluckerLine<> line(p1, p2);

  const Point3<> p3(6, 8, 12);
  const Point3<> p4(6, 9, 11);

  EXPECT_FALSE(line.IsCoincident(p3));
  EXPECT_FALSE(line.IsCoincident(p4));

  HalfSpace3<> plane(p2, p3, p4);

  auto intersect_point = line.Intersect(plane);
  EXPECT_EQ(intersect_point, HomoPoint3<>(p2));
  EXPECT_TRUE(line.IsCoincident(intersect_point));
}

TEST(PluckerLine, Project2D) {
  const Point3<> p1(17, 23, 31);
  const Point3<> p2(131, 163, 197);
  const PluckerLine<> line(p1, p2);

  const Point3<> p3(41, 103, 73);

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
  const Point3<> p1(0, 1, 2);
  const Point3<> p2(1, 1, 2);

  const Point3<> origin(0, 0, 0);

  {
    const PluckerLine<> line(p1, p2);
    auto line2d = line.Project2D(/*dimension=*/1);
    // The `line` goes from (0, 1, 2) to (1, 1, 2). After dropping the y
    // coordinate and swapping x and z, `line2d` goes from (2, 0) to (2, 1). A
    // clockwise quarter turn from that line points from (2, 0) towards the
    // origin.
    EXPECT_GT(line2d.Compare(origin.DropDimension(1)), 0);
  }

  {
    const PluckerLine<> line(p2, p1);
    auto line2d = line.Project2D(/*dimension=*/1);
    // The `line` goes from (1, 1, 2) to (0, 1, 2). After dropping the y
    // coordinate and swapping x and z, `line2d` goes from (2, 1) to (2, 0). A
    // clockwise quarter turn from that line points from (2, 0) away from the
    // origin.
    EXPECT_LT(line2d.Compare(origin.DropDimension(1)), 0);
  }
}

TEST(PluckerLine, Project2DDropZSideness) {
  const Point3<> p1(0, 1, 1);
  const Point3<> p2(1, 1, 1);

  const Point3<> origin(0, 0, 0);

  {
    const PluckerLine<> line(p1, p2);
    auto line2d = line.Project2D(/*dimension=*/2);
    EXPECT_LT(line2d.Compare(origin.DropDimension(2)), 0);
  }

  {
    const PluckerLine<> line(p2, p1);
    auto line2d = line.Project2D(/*dimension=*/2);
    EXPECT_GT(line2d.Compare(origin.DropDimension(2)), 0);
  }
}

template <int point3_bits>
void TestCorrectOutputBitsFromVertices() {
  using Builder = PluckerLineFromPoint3sBuilder<point3_bits>;
  using PluckerLineRep = typename Builder::PluckerLineRep;
  using Point3Rep = typename Builder::Point3Rep;
  using DInt = typename PluckerLineRep::DVector::BigIntRep;
  using MInt = typename PluckerLineRep::MVector::BigIntRep;
  using BigIntRep = typename Point3Rep::BigIntRep;
  int up_to = 1;
  for (int i = 0; i < 6; ++i) {
    up_to *= 2;
  }
  EXPECT_GT(up_to, 1);
  DInt smallest_d_coord[3] = {DInt(0),
                              DInt(0),
                              DInt(0)};
  DInt largest_d_coord[3] = {DInt(0),
                             DInt(0),
                             DInt(0)};
  MInt smallest_m_coord[3] = {MInt(0),
                              MInt(0),
                              MInt(0)};
  MInt largest_m_coord[3] = {MInt(0),
                             MInt(0),
                             MInt(0)};
  for (int i = 0; i < up_to; ++i) {
    int remaining = i;
    Point3Rep p[2];
    for (int j = 0; j < 2; ++j) {
      BigIntRep components[3];
      for (int k = 0; k < 3; ++k) {
        switch (remaining % 2) {
          case 0:
            components[k] = BigIntRep::min_value();
            break;
          case 1:
            components[k] = BigIntRep::max_value();
            break;
        }
        remaining /= 2;
      }
      p[j] = Point3Rep(components[0], components[1], components[2]);
    }
    PluckerLineRep line = Builder::Build(p[0], p[1]);
    // A PluckerLine type with double the required bits.
    using PluckerLineExtraBits =
      PluckerLine<PluckerLineRep::DVector::BigIntRep::bits*2,
                  PluckerLineRep::MVector::BigIntRep::bits*2>;
    PluckerLineExtraBits line_extra(p[0], p[1]);
    EXPECT_EQ(line.IsValid(), line_extra.IsValid());
    EXPECT_EQ(line, line_extra);
    EXPECT_TRUE(line.IsValidState());
    for (int j = 0; j < 3; ++j) {
      smallest_d_coord[j] = std::min(smallest_d_coord[j],
                                     line.d().components()[j]);
      largest_d_coord[j] = std::max(largest_d_coord[j],
                                    line.d().components()[j]);

      smallest_m_coord[j] = std::min(smallest_m_coord[j],
                                     line.m().components()[j]);
      largest_m_coord[j] = std::max(largest_m_coord[j],
                                    line.m().components()[j]);
    }
  }
  for (int j = 0; j < 3; ++j) {
    EXPECT_EQ(smallest_d_coord[j], Builder::d_component_min());
    EXPECT_EQ(largest_d_coord[j], Builder::d_component_max());

    EXPECT_EQ(smallest_m_coord[j], Builder::m_component_min());
    EXPECT_EQ(largest_m_coord[j], Builder::m_component_max());
  }
  using NextSmallerDInt = BigInt<DInt::bits - 1>;
  using NextSmallerMInt = BigInt<MInt::bits - 1>;
  EXPECT_LT(Builder::d_component_min(), DInt(NextSmallerDInt::min_value()));
  EXPECT_GT(Builder::d_component_max(), DInt(NextSmallerDInt::max_value()));
  EXPECT_LT(Builder::m_component_min(), MInt(NextSmallerMInt::min_value()));
  EXPECT_GT(Builder::m_component_max(), MInt(NextSmallerMInt::max_value()));
}

TEST(PluckerLineFromPoint3sBuilder, CorrectOutputBits2) {
  TestCorrectOutputBitsFromVertices<2>();
}

TEST(PluckerLineFromPoint3sBuilder, CorrectOutputBits32) {
  TestCorrectOutputBitsFromVertices<32>();
}

TEST(PluckerLineFromPoint3sBuilder, CorrectOutputBits64) {
  TestCorrectOutputBitsFromVertices<64>();
}

template <int point3_bits>
void TestCorrectOutputBitsFromPlanesFromPoint3s() {
  using Builder = PluckerLineFromPlanesFromPoint3sBuilder<point3_bits>;
  using HalfSpace3Builder = typename Builder::HalfSpace3Builder;
  using HalfSpace3Rep = typename HalfSpace3Builder::HalfSpace3Rep;
  using VectorInt = typename HalfSpace3Rep::VectorInt;
  using DistInt = typename HalfSpace3Rep::DistInt;
  using PluckerLineRep = typename Builder::PluckerLineRep;
  using DInt = typename PluckerLineRep::DVector::BigIntRep;
  using MInt = typename PluckerLineRep::MVector::BigIntRep;
  int up_to = 1;
  for (int i = 0; i < 8; ++i) {
    up_to *= 2;
  }
  EXPECT_GT(up_to, 1);
  DInt smallest_d_coord[3] = {DInt(0),
                              DInt(0),
                              DInt(0)};
  DInt largest_d_coord[3] = {DInt(0),
                             DInt(0),
                             DInt(0)};
  MInt smallest_m_coord[3] = {MInt(0),
                              MInt(0),
                              MInt(0)};
  MInt largest_m_coord[3] = {MInt(0),
                             MInt(0),
                             MInt(0)};
  for (int i = 0; i < up_to; ++i) {
    int remaining = i;
    HalfSpace3Rep plane[2];
    for (int j = 0; j < 2; ++j) {
      VectorInt normal_components[3];
      for (int k = 0; k < 3; ++k) {
        switch (remaining % 2) {
          case 0:
            normal_components[k] = HalfSpace3Builder::normal_component_min();
            break;
          case 1:
            normal_components[k] = HalfSpace3Builder::normal_component_max();
            break;
        }
        remaining /= 2;
      }
      DistInt dist;
      switch (remaining % 2) {
        case 0:
          dist = HalfSpace3Builder::dist_min();
          break;
        case 1:
          dist = HalfSpace3Builder::dist_max();
          break;
      }
      remaining /= 2;
      typename HalfSpace3Rep::VectorRep normal(normal_components[0],
                                          normal_components[1],
                                          normal_components[2]);
      plane[j] = HalfSpace3Rep(normal, dist);
    }
    if (!plane[0].IsValid() || !plane[1].IsValid()) {
      continue;
    }
    if (plane[0] == plane[1]) {
      continue;
    }
    PluckerLineRep line = Builder::Build(plane[0], plane[1]);
    // A PluckerLine type with double the required bits.
    using PluckerLineExtraBits =
      PluckerLine<PluckerLineRep::DVector::BigIntRep::bits*2,
                  PluckerLineRep::MVector::BigIntRep::bits*2>;
    PluckerLineExtraBits line_extra(const_cast<const HalfSpace3Rep&>(plane[0]),
        const_cast<const HalfSpace3Rep&>(plane[1]));
    EXPECT_EQ(line.IsValid(), line_extra.IsValid());
    EXPECT_EQ(line, line_extra);
    EXPECT_TRUE(line.IsValidState());
    for (int j = 0; j < 3; ++j) {
      smallest_d_coord[j] = std::min(smallest_d_coord[j],
                                     line.d().components()[j]);
      largest_d_coord[j] = std::max(largest_d_coord[j],
                                    line.d().components()[j]);

      smallest_m_coord[j] = std::min(smallest_m_coord[j],
                                     line.m().components()[j]);
      largest_m_coord[j] = std::max(largest_m_coord[j],
                                    line.m().components()[j]);
    }
  }
  for (int j = 0; j < 3; ++j) {
    EXPECT_EQ(smallest_d_coord[j], Builder::d_component_min());
    EXPECT_EQ(largest_d_coord[j], Builder::d_component_max());

    EXPECT_EQ(smallest_m_coord[j], Builder::m_component_min());
    EXPECT_EQ(largest_m_coord[j], Builder::m_component_max());
  }
  using NextSmallerDInt = BigInt<DInt::bits - 1>;
  using NextSmallerMInt = BigInt<MInt::bits - 1>;
  EXPECT_LT(Builder::d_component_min(), DInt(NextSmallerDInt::min_value()));
  EXPECT_GT(Builder::d_component_max(), DInt(NextSmallerDInt::max_value()));
  EXPECT_LT(Builder::m_component_min(), MInt(NextSmallerMInt::min_value()));
  EXPECT_GT(Builder::m_component_max(), MInt(NextSmallerMInt::max_value()));
}

TEST(PluckerLineFromPlanesFromPoint3sBuilder, CorrectOutputBits3) {
  TestCorrectOutputBitsFromPlanesFromPoint3s<3>();
}

TEST(PluckerLineFromPlanesFromPoint3sBuilder, CorrectOutputBits32) {
  TestCorrectOutputBitsFromPlanesFromPoint3s<32>();
}

TEST(PluckerLineFromPlanesFromPoint3sBuilder, CorrectOutputBits64) {
  TestCorrectOutputBitsFromPlanesFromPoint3s<64>();
}

}  // walnut
