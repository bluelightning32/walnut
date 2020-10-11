#include "walnut/plucker_line.h"

#include "gtest/gtest.h"

namespace walnut {

TEST(PluckerLine, IsOnLine) {
  const Vertex3<> p1(1, 2, 3);
  const Vertex3<> p2(5, 7, 11);
  PluckerLine<> line(p1, p2);

  EXPECT_TRUE(line.IsOnLine(p1));
  EXPECT_TRUE(line.IsOnLine(p2));

  const Vertex3<> p3(p2 + (p2 - p1));
  EXPECT_TRUE(line.IsOnLine(p3));

  const Vertex3<> p4(p1 - (p2 - p1));
  EXPECT_TRUE(line.IsOnLine(p4));
}

TEST(PluckerLine, IsOnLineThroughOrigin) {
  const Vertex3<> p1(1, 2, 3);
  const Vertex3<> p2(0, 0, 0);
  PluckerLine<> line(p1, p2);

  EXPECT_TRUE(line.IsOnLine(p1));
  EXPECT_TRUE(line.IsOnLine(p2));

  const Vertex3<> p3(p2 + (p2 - p1));
  EXPECT_TRUE(line.IsOnLine(p3));
}

TEST(PluckerLine, Equality) {
  const Vertex3<> p1(1, 2, 3);
  const Vector3<> d(5, 7, 11);
  const PluckerLine<> line(p1, Vertex3<>(p1 + d.Scale(2)));

  const PluckerLine<> line2(p1 + d.Scale(5),
                            p1 + d.Scale(3));

  EXPECT_EQ(line, line2);

  const PluckerLine<> line3(p1,
                            Vertex3<>(p1 + d.Scale(-3)));

  EXPECT_EQ(line, line3);
}

template <int vertex3_bits>
void TestCorrectOutputBits() {
  using Builder = PluckerLineFromVertex3Builder<vertex3_bits>;
  using PluckerLineRep = typename Builder::PluckerLineRep;
  using Vertex3Rep = typename Builder::Vertex3Rep;
  using DInt = typename PluckerLineRep::DVector::BigIntRep;
  using MInt = typename PluckerLineRep::MVector::BigIntRep;
  using BigIntRep = typename Vertex3Rep::BigIntRep;
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
    Vertex3Rep p[2];
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
      p[j] = Vertex3Rep(coords[0], coords[1], coords[2]);
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

TEST(PluckerLineFromVertex3Builder, CorrectOutputBits2) {
  TestCorrectOutputBits<2>();
}

TEST(PluckerLineFromVertex3Builder, CorrectOutputBits32) {
  TestCorrectOutputBits<32>();
}

TEST(PluckerLineFromVertex3Builder, CorrectOutputBits64) {
  TestCorrectOutputBits<64>();
}

}  // walnut
