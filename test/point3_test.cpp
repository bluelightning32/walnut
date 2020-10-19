#include "walnut/point3.h"

#include "gtest/gtest.h"

namespace walnut {

TEST(Point3, ConstructorAssertsOnOverflow) {
  static constexpr int big_coord_bits = Point3<32>::BigIntRep::word_count *
                                        Point3<32>::BigIntRep::bits_per_word * 2;
  BigInt<big_coord_bits> big_coord = BigInt<big_coord_bits>::max_value();
  Point3<big_coord_bits> big_vertex(big_coord, big_coord, big_coord);

  ASSERT_DEBUG_DEATH(Point3<32> v(big_vertex), "overflow");
}

TEST(Point3, XYZIntConstructor) {
  Point3<> vertex(1, 2, 3);
  EXPECT_EQ(vertex.x(), 1);
  EXPECT_EQ(vertex.y(), 2);
  EXPECT_EQ(vertex.z(), 3);
}

TEST(Point3, Get2DTwistDirDropZ) {
  int z_values[] = {0, 10, -10};
  for (int z : z_values) {
    //
    // b
    // ^
    // |
    // c ---> a
    //
    Point3<> a(1, 0, z);
    Point3<> center(0, 0, z);
    Point3<> b(0, 1, z);

    // a to b is counter-clockwise
    EXPECT_GT(center.Get2DTwistDir(/*drop_dimension=*/2, a, b), 0);
    // b to a is clockwise
    EXPECT_LT(center.Get2DTwistDir(/*drop_dimension=*/2, b, a), 0);

    // collinear tests
    EXPECT_EQ(center.Get2DTwistDir(/*drop_dimension=*/2, a, a), 0);
    EXPECT_EQ(center.Get2DTwistDir(/*drop_dimension=*/2, b, b), 0);
    EXPECT_EQ(center.Get2DTwistDir(/*drop_dimension=*/2, a, center), 0);
    EXPECT_EQ(center.Get2DTwistDir(/*drop_dimension=*/2, center, a), 0);
  }
}

}  // walnut
