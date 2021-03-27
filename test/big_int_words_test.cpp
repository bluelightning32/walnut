#include "walnut/big_int_words.h"

#include "gtest/gtest.h"

namespace walnut {

TEST(BigIntWords, ConstructWithSize) {
  BigIntWords a(2);
  EXPECT_EQ(a.size(), 2);

  a[0] = 1;
  a[1] = 2;
  EXPECT_EQ(a[0], 1);
  EXPECT_EQ(a[1], 2);
}

TEST(BigIntWords, CopyConstructSize1) {
  BigIntWords a(1);
  a[0] = 1;

  BigIntWords b(a);
  EXPECT_EQ(b.size(), 1);
  EXPECT_EQ(b[0], 1);
}

TEST(BigIntWords, CopyConstructSize2) {
  BigIntWords a(2);
  a[0] = 1;
  a[1] = 2;

  BigIntWords b(a);
  EXPECT_EQ(b.size(), 2);
  EXPECT_EQ(b[0], 1);
  EXPECT_EQ(b[1], 2);
}

TEST(BigIntWords, MoveConstructSize1) {
  BigIntWords a(1);
  a[0] = 1;

  BigIntWords b(std::move(a));
  EXPECT_EQ(b.size(), 1);
  EXPECT_EQ(b[0], 1);
}

TEST(BigIntWords, MoveConstructSize2) {
  BigIntWords a(2);
  a[0] = 1;
  a[1] = 2;

  BigIntWords b(std::move(a));
  EXPECT_EQ(b.size(), 2);
  EXPECT_EQ(b[0], 1);
  EXPECT_EQ(b[1], 2);
}

TEST(BigIntWords, AssignSize1) {
  BigIntWords a(1);
  a[0] = 1;

  BigIntWords b(1);
  b = a;
  EXPECT_EQ(b.size(), 1);
  EXPECT_EQ(b[0], 1);
}

TEST(BigIntWords, AssignSize2) {
  BigIntWords a(2);
  a[0] = 1;
  a[1] = 2;

  BigIntWords b(1);
  b = a;
  EXPECT_EQ(b.size(), 2);
  EXPECT_EQ(b[0], 1);
  EXPECT_EQ(b[1], 2);
}

TEST(BigIntWords, MoveAssignSize1) {
  BigIntWords a(1);
  a[0] = 1;

  BigIntWords b(1);
  b = std::move(a);
  EXPECT_EQ(b.size(), 1);
  EXPECT_EQ(b[0], 1);
}

TEST(BigIntWords, MoveAssignSize2) {
  BigIntWords a(2);
  a[0] = 1;
  a[1] = 2;

  BigIntWords b(1);
  b = std::move(a);
  EXPECT_EQ(b.size(), 2);
  EXPECT_EQ(b[0], 1);
  EXPECT_EQ(b[1], 2);
}

TEST(BigIntWords, PushBack) {
  BigIntWords a(1);
  a[0] = 1;

  size_t i = 1;
  while (a.size() < 32) {
    ++i;
    a.push_back(BigUIntWord{i});
  }

  for (size_t i = 0; i < 32; ++i) {
    EXPECT_EQ(a[i], i + 1);
  }
}

TEST(BigIntWords, Resize) {
  BigIntWords a(2);
  a[0] = 1;
  a[1] = 2;

  a.resize(4);
  a[2] = 3;
  a[3] = 4;

  EXPECT_EQ(a.size(), 4);
  EXPECT_EQ(a[0], 1);
  EXPECT_EQ(a[1], 2);
  EXPECT_EQ(a[2], 3);
  EXPECT_EQ(a[3], 4);
}

}  // walnut
