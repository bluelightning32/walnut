#include "walnut/member_forward.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

struct TrackMoveCopy {
  TrackMoveCopy() = default;

  TrackMoveCopy(const TrackMoveCopy& parent) {
    parent.copied_count++;
  }

  TrackMoveCopy(TrackMoveCopy&& parent) {
    parent.moved_count++;
  }

  int moved_count = 0;
  mutable int copied_count = 0;
};

TEST(MemberForward, ConstRefParent) {
  std::tuple<TrackMoveCopy> t;

  const std::tuple<TrackMoveCopy>& tref = t;

  MemberForward<decltype(tref)>(std::get<0>(tref));

  EXPECT_EQ(std::get<0>(tref).moved_count, 0);
  EXPECT_EQ(std::get<0>(tref).copied_count, 0);

  TrackMoveCopy copy = MemberForward<decltype(tref)>(std::get<0>(tref));

  EXPECT_EQ(std::get<0>(tref).moved_count, 0);
  EXPECT_EQ(std::get<0>(tref).copied_count, 1);

  TrackMoveCopy moved = MemberForward<decltype(tref)>(
      std::move(std::get<0>(t)));

  EXPECT_EQ(std::get<0>(tref).moved_count, 1);
  EXPECT_EQ(std::get<0>(tref).copied_count, 1);
}

TEST(MemberForward, RefParent) {
  std::tuple<TrackMoveCopy> t;

  std::tuple<TrackMoveCopy>& tref = t;

  MemberForward<decltype(tref)>(std::get<0>(tref));

  EXPECT_EQ(std::get<0>(tref).moved_count, 0);
  EXPECT_EQ(std::get<0>(tref).copied_count, 0);

  TrackMoveCopy copy = MemberForward<decltype(tref)>(std::get<0>(tref));

  EXPECT_EQ(std::get<0>(tref).moved_count, 0);
  EXPECT_EQ(std::get<0>(tref).copied_count, 1);

  TrackMoveCopy moved = MemberForward<decltype(tref)>(
      std::move(std::get<0>(t)));

  EXPECT_EQ(std::get<0>(tref).moved_count, 1);
  EXPECT_EQ(std::get<0>(tref).copied_count, 1);
}

TEST(MemberForward, RValueRefParent) {
  std::tuple<TrackMoveCopy> t;

  std::tuple<TrackMoveCopy>&& tref = std::move(t);

  MemberForward<decltype(tref)>(std::get<0>(tref));

  EXPECT_EQ(std::get<0>(tref).moved_count, 0);
  EXPECT_EQ(std::get<0>(tref).copied_count, 0);

  TrackMoveCopy moved1 = MemberForward<decltype(tref)>(std::get<0>(tref));

  EXPECT_EQ(std::get<0>(tref).moved_count, 1);
  EXPECT_EQ(std::get<0>(tref).copied_count, 0);

  TrackMoveCopy moved2 = MemberForward<decltype(tref)>(
      std::move(std::get<0>(t)));

  EXPECT_EQ(std::get<0>(tref).moved_count, 2);
  EXPECT_EQ(std::get<0>(tref).copied_count, 0);
}

}  // walnut
