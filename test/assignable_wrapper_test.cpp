#include "walnut/assignable_wrapper.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

struct IntWrapper {
  IntWrapper(int value) : value(value) { }

  int value;
};

struct CopyOnly : public IntWrapper {
  using IntWrapper::IntWrapper;

  CopyOnly(const CopyOnly& other) : IntWrapper(other.value + 1) { }

  CopyOnly(CopyOnly&&) = delete;

  CopyOnly& operator=(const CopyOnly& other) {
    value = other.value + 10;
    return *this;
  }

  CopyOnly& operator=(CopyOnly&&) = delete;
};

TEST(AssignableWrapper, CopyOnly) {
  CopyOnly a(1);

  AssignableWrapper<CopyOnly> b(a);
  AssignableWrapper<CopyOnly> c(a);

  EXPECT_EQ(b.value, 2);
  EXPECT_EQ(c.value, 2);
  EXPECT_TRUE((std::is_constructible<AssignableWrapper<CopyOnly>,
                                     const CopyOnly&>::value));
  EXPECT_FALSE((std::is_nothrow_constructible<AssignableWrapper<CopyOnly>,
                                              const CopyOnly&>::value));
}

TEST(AssignableWrapper, CopyAssign) {
  CopyOnly a(1);

  AssignableWrapper<CopyOnly> b(a);
  AssignableWrapper<CopyOnly> c(a);

  EXPECT_EQ(b.value, 2);
  EXPECT_EQ(c.value, 2);

  c = b;

  EXPECT_EQ(b.value, 2);
  EXPECT_EQ(c.value, 12);
}

struct MoveOnly : public IntWrapper {
  using IntWrapper::IntWrapper;

  MoveOnly(RValueKey<MoveOnly> other) : IntWrapper(other.get().value + 1) { }

  MoveOnly(const MoveOnly&) = delete;

  MoveOnly& operator=(const MoveOnly&) = delete;

 protected:
  RValueKey<MoveOnly> GetRValueKey() && {
    return RValueKey<MoveOnly>(std::move(*this));
  }

  MoveOnly& operator=(RValueKey<MoveOnly> other) noexcept {
    value = other.get().value + 10;
    ++other.get().value;
    return *this;
  }
};

TEST(AssignableWrapper, MoveOnly) {
  AssignableWrapper<MoveOnly> a(1);

  AssignableWrapper<MoveOnly> b(std::move(a));

  EXPECT_EQ(b.value, 2);
  EXPECT_FALSE((std::is_constructible<AssignableWrapper<MoveOnly>,
                                      MoveOnly&&>::value));
  EXPECT_TRUE(std::is_move_constructible<AssignableWrapper<MoveOnly>>::value);
  EXPECT_FALSE(
      std::is_nothrow_move_constructible<AssignableWrapper<MoveOnly>>::value);
}

TEST(AssignableWrapper, MoveAssign) {
  AssignableWrapper<MoveOnly> a(1);

  AssignableWrapper<MoveOnly> b(0);

  EXPECT_EQ(a.value, 1);
  EXPECT_EQ(b.value, 0);

  b = std::move(a);

  EXPECT_EQ(a.value, 2);
  EXPECT_EQ(b.value, 11);

  EXPECT_TRUE(std::is_move_assignable<AssignableWrapper<MoveOnly>>::value);
  EXPECT_TRUE(
      std::is_nothrow_move_assignable<AssignableWrapper<MoveOnly>>::value);
}

struct NoExcept : public IntWrapper {
  using IntWrapper::IntWrapper;

  NoExcept(const NoExcept& other) noexcept : IntWrapper(other.value + 1) { }

  NoExcept(RValueKey<NoExcept> other) noexcept
   : IntWrapper(other.get().value + 2) { }

 protected:
  RValueKey<NoExcept> GetRValueKey() && {
    return RValueKey<NoExcept>(std::move(*this));
  }
};

TEST(AssignableWrapper, NoExceptConstruct) {
  AssignableWrapper<NoExcept> a(1);

  AssignableWrapper<NoExcept> b(a);
  AssignableWrapper<NoExcept> c(std::move(a));

  EXPECT_EQ(b.value, 2);
  EXPECT_TRUE((std::is_nothrow_constructible<AssignableWrapper<NoExcept>,
                                             const NoExcept&>::value));
  EXPECT_EQ(c.value, 3);
  EXPECT_TRUE((std::is_nothrow_constructible<
                 AssignableWrapper<NoExcept>,
                 AssignableWrapper<NoExcept>&&>::value));
}

struct MoveOnlyDerived : public MoveOnly {
  MoveOnlyDerived(int value) : MoveOnly(value) { }

  MoveOnlyDerived(RValueKey<MoveOnlyDerived> other) : MoveOnly(other) { }

 protected:
  RValueKey<MoveOnlyDerived> GetRValueKey() && {
    return RValueKey<MoveOnlyDerived>(std::move(*this));
  }

  MoveOnlyDerived& operator=(RValueKey<MoveOnlyDerived> other) noexcept {
    MoveOnly::operator=(other);
    return *this;
  }
};

TEST(AssignableWrapper, MoveOnlyDerived) {
  AssignableWrapper<MoveOnlyDerived> a(1);

  AssignableWrapper<MoveOnlyDerived> b(std::move(a));

  EXPECT_EQ(b.value, 2);
  EXPECT_FALSE((std::is_constructible<AssignableWrapper<MoveOnlyDerived>,
                                      MoveOnlyDerived&&>::value));
  EXPECT_TRUE(
      std::is_move_constructible<AssignableWrapper<MoveOnlyDerived>>::value);
}

}  // walnut
