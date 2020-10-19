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

TEST(PluckerLine, IsOnLine) {
  const Point3<> p1(1, 2, 3);
  const Point3<> p2(5, 7, 11);
  PluckerLine<> line(p1, p2);

  EXPECT_TRUE(line.IsOnLine(p1));
  EXPECT_TRUE(line.IsOnLine(p2));

  const Point3<> p3(p2 + (p2 - p1));
  EXPECT_TRUE(line.IsOnLine(p3));

  const Point3<> p4(p1 - (p2 - p1));
  EXPECT_TRUE(line.IsOnLine(p4));
}

TEST(PluckerLine, IsOnLineThroughOrigin) {
  const Point3<> p1(1, 2, 3);
  const Point3<> p2(0, 0, 0);
  PluckerLine<> line(p1, p2);

  EXPECT_TRUE(line.IsOnLine(p1));
  EXPECT_TRUE(line.IsOnLine(p2));

  const Point3<> p3(p2 + (p2 - p1));
  EXPECT_TRUE(line.IsOnLine(p3));
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
}

TEST(PluckerLine, ConstructFromPlanes) {
  const Point3<> p1(1, 2, 3);
  const Point3<> p2(5, 7, 11);
  const PluckerLine<> line_from_points(p1, p2);

  const Point3<> p3(5, 7, 12);
  const Point3<> p4(6, 7, 11);

  Plane<> a(p1, p2, p3);
  Plane<> b(p1, p2, p4);
  const PluckerLine<> line_from_planes(a, b);

  EXPECT_EQ(line_from_planes, line_from_points);

  EXPECT_TRUE(line_from_planes.IsOnLine(p1));
  EXPECT_TRUE(line_from_planes.IsOnLine(p2));
}

TEST(PluckerLine, FromPlanesDirection) {
  const Point3<> p1(1, 2, 3);
  const Point3<> p2(5, 7, 11);
  const PluckerLine<> line_from_points(p1, p2);

  const Point3<> p3(5, 7, 12);
  const Point3<> p4(6, 7, 11);

  Plane<> a(p1, p2, p3);
  Plane<> b(p1, p2, p4);
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

  EXPECT_FALSE(line.IsOnLine(p3));
  EXPECT_FALSE(line.IsOnLine(p4));

  Plane<> plane(p2, p3, p4);

  EXPECT_EQ(line.Intersect(plane), Point4<>(p2));
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
                                     line.d().coords()[j]);
      largest_d_coord[j] = std::max(largest_d_coord[j],
                                    line.d().coords()[j]);

      smallest_m_coord[j] = std::min(smallest_m_coord[j],
                                     line.m().coords()[j]);
      largest_m_coord[j] = std::max(largest_m_coord[j],
                                    line.m().coords()[j]);
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
  using PlaneBuilder = typename Builder::PlaneBuilder;
  using PlaneRep = typename PlaneBuilder::PlaneRep;
  using VectorInt = typename PlaneRep::VectorInt;
  using DistInt = typename PlaneRep::DistInt;
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
    PlaneRep plane[2];
    for (int j = 0; j < 2; ++j) {
      VectorInt normal_coords[3];
      for (int k = 0; k < 3; ++k) {
        switch (remaining % 2) {
          case 0:
            normal_coords[k] = PlaneBuilder::normal_component_min();
            break;
          case 1:
            normal_coords[k] = PlaneBuilder::normal_component_max();
            break;
        }
        remaining /= 2;
      }
      DistInt dist;
      switch (remaining % 2) {
        case 0:
          dist = PlaneBuilder::dist_min();
          break;
        case 1:
          dist = PlaneBuilder::dist_max();
          break;
      }
      remaining /= 2;
      typename PlaneRep::VectorRep normal(normal_coords[0],
                                          normal_coords[1],
                                          normal_coords[2]);
      plane[j] = PlaneRep(normal, dist);
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
    PluckerLineExtraBits line_extra(const_cast<const PlaneRep&>(plane[0]),
        const_cast<const PlaneRep&>(plane[1]));
    EXPECT_EQ(line.IsValid(), line_extra.IsValid());
    EXPECT_EQ(line, line_extra);
    EXPECT_TRUE(line.IsValidState());
    for (int j = 0; j < 3; ++j) {
      smallest_d_coord[j] = std::min(smallest_d_coord[j],
                                     line.d().coords()[j]);
      largest_d_coord[j] = std::max(largest_d_coord[j],
                                    line.d().coords()[j]);

      smallest_m_coord[j] = std::min(smallest_m_coord[j],
                                     line.m().coords()[j]);
      largest_m_coord[j] = std::max(largest_m_coord[j],
                                    line.m().coords()[j]);
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
