#include "walnut/vertex.h"

#include "gtest/gtest.h"

namespace walnut {

TEST(Vertex3, ConstructorAssertsOnOverflow) {
  static constexpr int big_coord_bits = Vertex3<32>::max_coord_bits*2;
  BigInt<big_coord_bits> big_coord = BigInt<big_coord_bits>::max_value();
  Vertex3<big_coord_bits> big_vertex(big_coord, big_coord, big_coord);

  ASSERT_DEBUG_DEATH(Vertex3<32> v(big_vertex), "overflow");
}

TEST(Vertex3, XYZIntConstructor) {
  Vertex3<> vertex(1, 2, 3);
  EXPECT_EQ(vertex.x(), 1);
  EXPECT_EQ(vertex.y(), 2);
  EXPECT_EQ(vertex.z(), 3);
}

}  // walnut
