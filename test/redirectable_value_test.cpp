#include "walnut/redirectable_value.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

struct HasDefaultConstructor {
  int value = 5;
};

TEST(RedirectableValue, DefaultConstruct) {
  RedirectableValue<HasDefaultConstructor> value;

  EXPECT_EQ(value->value, HasDefaultConstructor().value);
}

struct CopyOnly {
  explicit CopyOnly(int value) : value(value) { }
  CopyOnly(const CopyOnly&) = default;

  int value;
};

TEST(RedirectableValue, CopyOnly) {
  RedirectableValue<CopyOnly> value(1);

  EXPECT_EQ(value->value, 1);
}

struct TrackDestruction {
  TrackDestruction(int& destruct_counter) :
    destruct_counter(destruct_counter) { }

  ~TrackDestruction() {
    ++destruct_counter;
  }

  int& destruct_counter;
};

TEST(RedirectableValue, Redirect) {
  int v1_destructed = 0;
  int v2_destructed = 0;
  int v3_destructed = 0;

  {
    RedirectableValue<TrackDestruction> v1(v1_destructed);
    RedirectableValue<TrackDestruction> v2(v2_destructed);
    RedirectableValue<TrackDestruction> v3(v3_destructed);

    EXPECT_EQ(&v1->destruct_counter, &v1_destructed);
    EXPECT_EQ(&v2->destruct_counter, &v2_destructed);
    EXPECT_EQ(&v3->destruct_counter, &v3_destructed);

    EXPECT_TRUE(v1.IsPrimary());
    EXPECT_TRUE(v2.IsPrimary());
    EXPECT_TRUE(v3.IsPrimary());

    v1.Redirect(v2);
    EXPECT_EQ(&v1->destruct_counter, &v2_destructed);
    EXPECT_EQ(&v2->destruct_counter, &v2_destructed);
    EXPECT_EQ(v1_destructed, 1);
    EXPECT_EQ(v2_destructed, 0);

    v2.Redirect(v3);
    EXPECT_EQ(&v1->destruct_counter, &v3_destructed);
    EXPECT_EQ(&v2->destruct_counter, &v3_destructed);
    EXPECT_EQ(&v3->destruct_counter, &v3_destructed);
    EXPECT_EQ(v1_destructed, 1);
    EXPECT_EQ(v2_destructed, 1);
    EXPECT_EQ(v3_destructed, 0);

    EXPECT_FALSE(v1.IsPrimary());
    EXPECT_FALSE(v2.IsPrimary());
    EXPECT_TRUE(v3.IsPrimary());

    EXPECT_EQ(&v1.primary(), &v3);
    EXPECT_EQ(&v2.primary(), &v3);
    EXPECT_EQ(&v3.primary(), &v3);
  }
  EXPECT_EQ(v1_destructed, 1);
  EXPECT_EQ(v2_destructed, 1);
  EXPECT_EQ(v3_destructed, 1);
}

TEST(RedirectableValue, ConstPrimary) {
  int v1_destructed = 0;
  int v2_destructed = 0;
  int v3_destructed = 0;

  using Value = RedirectableValue<TrackDestruction>;
  Value v1(v1_destructed);
  Value v2(v2_destructed);
  Value v3(v3_destructed);

  v1.Redirect(v2);
  v2.Redirect(v3);

  EXPECT_EQ(&const_cast<const Value&>(v1).primary(), &v3);
  EXPECT_EQ(&const_cast<const Value&>(v2).primary(), &v3);
  EXPECT_EQ(&const_cast<const Value&>(v3).primary(), &v3);
}

TEST(RedirectableValue, Equality) {
  int v1_destructed = 0;
  int v2_destructed = 0;
  int v3_destructed = 0;

  using Value = RedirectableValue<TrackDestruction>;
  Value v1(v1_destructed);
  Value v2(v2_destructed);
  Value v3(v3_destructed);

  EXPECT_NE(v1, v2);
  EXPECT_NE(v2, v3);
  EXPECT_NE(v3, v1);

  v1.Redirect(v2);
  v2.Redirect(v3);

  EXPECT_EQ(v1, v2);
  EXPECT_EQ(v2, v3);
  EXPECT_EQ(v3, v1);
}

}  // walnut
