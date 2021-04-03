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
};

TEST(AssignableWrapper, CopyOnly) {
  CopyOnly a(1);

  AssignableWrapper<CopyOnly> b(a);

  EXPECT_EQ(b.value, 2);
  EXPECT_TRUE((std::is_constructible<AssignableWrapper<CopyOnly>,
                                     const CopyOnly&>::value));
  EXPECT_FALSE((std::is_nothrow_constructible<AssignableWrapper<CopyOnly>,
                                              const CopyOnly&>::value));
}

struct MoveOnly : public IntWrapper {
  using IntWrapper::IntWrapper;

  MoveOnly(MoveOnly&& other) : IntWrapper(other.value + 1) { }

  MoveOnly(const MoveOnly&) = delete;
};

TEST(AssignableWrapper, MoveOnly) {
  MoveOnly a(1);

  AssignableWrapper<MoveOnly> b(std::move(a));

  EXPECT_EQ(b.value, 2);
  EXPECT_TRUE((std::is_constructible<AssignableWrapper<CopyOnly>,
                                     CopyOnly&&>::value));
  EXPECT_FALSE((std::is_nothrow_constructible<AssignableWrapper<CopyOnly>,
                                              CopyOnly&&>::value));
}

struct ProtectedMove : public IntWrapper {
  using IntWrapper::IntWrapper;

  ProtectedMove(const ProtectedMove& other) : IntWrapper(other.value + 1) { }

 protected:
  ProtectedMove(ProtectedMove&& other) : IntWrapper(other.value - 1) { }
};

TEST(AssignableWrapper, ProtectedMove) {
  ProtectedMove a(1);

  // Even though an rvalue reference is passed, AssignableWrapper should fall
  // back to using ProtectedMove's copy constructor, because the
  // ProtectedMove's move constructor is protected.
  AssignableWrapper<ProtectedMove> b(std::move(a));

  EXPECT_EQ(b.value, 2);
}

struct NoExcept : public IntWrapper {
  using IntWrapper::IntWrapper;

  NoExcept(const NoExcept& other) noexcept : IntWrapper(other.value + 1) { }

  NoExcept(NoExcept&& other) noexcept : IntWrapper(other.value + 2) { }
};

TEST(AssignableWrapper, NoExceptConstruct) {
  NoExcept a(1);

  AssignableWrapper<NoExcept> b(a);
  AssignableWrapper<NoExcept> c(std::move(a));

  EXPECT_EQ(b.value, 2);
  EXPECT_TRUE((std::is_nothrow_constructible<AssignableWrapper<NoExcept>,
                                             const NoExcept&>::value));
  EXPECT_EQ(c.value, 3);
  EXPECT_TRUE((std::is_nothrow_constructible<AssignableWrapper<NoExcept>,
                                             NoExcept&&>::value));
}

}  // walnut
