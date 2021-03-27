#include "walnut/half_space2_sideless_compare.h"

#include "gtest/gtest.h"

namespace walnut {

TEST(HalfSpace2SidelessCompare, Irreflexive) {
  HalfSpace2 h1(/*x=*/1, /*y=*/0, /*dist=*/0);

  HalfSpace2SidelessCompare compare;
  EXPECT_FALSE(compare(h1, h1));
}

TEST(HalfSpace2SidelessCompare, DifferentVectors) {
  HalfSpace2 h1(/*x=*/1, /*y=*/0, /*dist=*/0);
  HalfSpace2 h2(/*x=*/0, /*y=*/1, /*dist=*/0);

  HalfSpace2SidelessCompare compare;
  EXPECT_NE(compare(h1, h2), compare(h2, h1));
}

TEST(HalfSpace2SidelessCompare, OppositeDirectionsEquivalent) {
  HalfSpace2 h1(/*x=*/1, /*y=*/2, /*dist=*/1);
  HalfSpace2 h2(/*x=*/-1, /*y=*/-2, /*dist=*/-1);

  HalfSpace2SidelessCompare compare;
  EXPECT_FALSE(compare(h1, h2));
  EXPECT_FALSE(compare(h2, h1));
}

TEST(HalfSpace2SidelessCompare, DifferentMagnitudesEquivalent) {
  HalfSpace2 h1(/*x=*/1, /*y=*/2, /*dist=*/1);
  HalfSpace2 h2(/*x=*/3, /*y=*/6, /*dist=*/3);

  HalfSpace2SidelessCompare compare;
  EXPECT_FALSE(compare(h1, h2));
  EXPECT_FALSE(compare(h2, h1));
}

TEST(HalfSpace2SidelessCompare, XAxisDifferentDists) {
  HalfSpace2 h1(/*x=*/1, /*y=*/0, /*dist=*/0);
  HalfSpace2 h2(/*x=*/1, /*y=*/0, /*dist=*/1);

  HalfSpace2SidelessCompare compare;
  EXPECT_NE(compare(h1, h2), compare(h2, h1));
}

TEST(HalfSpace2SidelessCompare, YAxisDifferentDists) {
  HalfSpace2 h1(/*x=*/0, /*y=*/1, /*dist=*/0);
  HalfSpace2 h2(/*x=*/0, /*y=*/1, /*dist=*/1);

  HalfSpace2SidelessCompare compare;
  EXPECT_NE(compare(h1, h2), compare(h2, h1));
}

TEST(HalfSpace2SidelessCompare, XAxisDifferentNormalizedDists) {
  HalfSpace2 h1(/*x=*/1, /*y=*/0, /*dist=*/1);
  HalfSpace2 h2(/*x=*/2, /*y=*/0, /*dist=*/1);

  HalfSpace2SidelessCompare compare;
  EXPECT_NE(compare(h1, h2), compare(h2, h1));
}

}  // walnut
