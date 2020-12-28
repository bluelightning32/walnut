#include "walnut/greatest_angle_tracker.h"

#include "gtest/gtest.h"

namespace walnut {

namespace {

const Vector3<> right(1, 0, 0);
const Vector3<> right_up(1, 1, 0);
const Vector3<> up(0, 1, 0);
const Vector3<> left(-1, 0, 0);
const Vector3<> down(0, -1, 0);

TEST(GreatestAngleTracker, MostCCW) {
  GreatestAngleTracker</*most_ccw=*/true> tracker;
  EXPECT_FALSE(tracker.AnyReceived());

  tracker.Receive(right_up);
  EXPECT_TRUE(tracker.AnyReceived());
  EXPECT_EQ(tracker.current(), right_up);

  // `right` is clockwise from `right_up`. So it should not change `tracker`.
  tracker.Receive(right);
  EXPECT_EQ(tracker.current(), right_up);

  tracker.Receive(left);
  EXPECT_EQ(tracker.current(), left);

  tracker.Receive(right_up);
  EXPECT_EQ(tracker.current(), left);
}

TEST(GreatestAngleTracker, MostCW) {
  GreatestAngleTracker</*most_ccw=*/false> tracker;
  EXPECT_FALSE(tracker.AnyReceived());

  tracker.Receive(right_up);
  EXPECT_TRUE(tracker.AnyReceived());
  EXPECT_EQ(tracker.current(), right_up);

  // `up` is counter-clockwise from `right_up`. So it should not change
  // `tracker`.
  tracker.Receive(up);
  EXPECT_EQ(tracker.current(), right_up);

  tracker.Receive(down);
  EXPECT_EQ(tracker.current(), down);

  tracker.Receive(right_up);
  EXPECT_EQ(tracker.current(), down);
}

}  // namespace

}  // namespace walnut
