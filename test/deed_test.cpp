#include "walnut/deed.h"

#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace walnut {

struct TestObject : public DeedObject {
  TestObject(int value) : value(value) { }

  TestObject(const TestObject& other) = default;

  TestObject(TestObject&&) noexcept = default;

  TestObject& operator=(const TestObject&) = default;

  TestObject& operator=(TestObject&&) = default;

  int value = 0;
};

struct NoCopy : public DeedObject {
  NoCopy(int value) : value(value) { }

  NoCopy(NoCopy&& other) : DeedObject(std::move(other)), value(other.value) { }

  NoCopy& operator=(NoCopy&& other) {
    value = other.value;
    DeedObject::operator=(std::move(other));
    return *this;
  }

  int value = 0;
};

TEST(Deed, CreateDeed) {
  TestObject o(1);
  Deed<TestObject> deed(&o);
  EXPECT_EQ(deed.get(), &o);
}

TEST(Deed, CopyingObjectDoesNotTransferDeed) {
  TestObject o1(1);
  Deed<TestObject> deed(&o1);
  TestObject o2(o1);
  EXPECT_EQ(deed.get(), &o1);
}

TEST(Deed, CopyAssigningObjectDoesNotTransferDeed) {
  TestObject o1(1);
  Deed<TestObject> deed(&o1);
  TestObject o2(2);
  o2 = o1;
  EXPECT_EQ(deed.get(), &o1);
}

TEST(Deed, MovingObjectTransfersDeed) {
  TestObject o1(1);
  Deed<TestObject> deed(&o1);
  TestObject o2(std::move(o1));
  EXPECT_EQ(deed.get(), &o2);
  deed.Return();
}

TEST(Deed, MoveAssigningObjectTransfersDeed) {
  TestObject o1(1);
  Deed<TestObject> deed1(&o1);
  TestObject o2(2);
  Deed<TestObject> deed2(&o2);
  o1 = std::move(o2);
  EXPECT_EQ(deed2.get(), &o1);
  deed1.Return();
  deed2.Return();
}

TEST(Deed, BoolCast) {
  TestObject o1(1);
  Deed<TestObject> deed1;
  EXPECT_FALSE(deed1);
  EXPECT_EQ(deed1, nullptr);

  deed1 = Deed<TestObject>(&o1);
  EXPECT_TRUE(deed1);
  EXPECT_NE(deed1, nullptr);
}

TEST(Deed, Move) {
  TestObject o1(1);
  Deed<TestObject> deed1(&o1);
  Deed<TestObject> deed2(std::move(deed1));
  EXPECT_EQ(deed2.get(), &o1);
  TestObject o2(std::move(o1));
  EXPECT_EQ(deed2.get(), &o2);
  deed2.Return();
}

TEST(Deed, MoveNull) {
  Deed<TestObject> deed1;
  Deed<TestObject> deed2(std::move(deed1));
  EXPECT_EQ(deed2.get(), nullptr);
}

TEST(Deed, Lend) {
  TestObject o1(1);
  Deed<TestObject> deed1(&o1);
  {
    Deed<TestObject> deed2 = deed1.Lend();
    EXPECT_EQ(deed2.get(), &o1);
    EXPECT_TRUE(deed1.is_lender());
    EXPECT_FALSE(deed2.is_lender());
  }
  // deed1 is restored when deed2 is destructed.
  EXPECT_EQ(deed1.get(), &o1);
}

TEST(Deed, Return) {
  TestObject o1(1);
  Deed<TestObject> deed1(&o1);
  Deed<TestObject> deed2 = deed1.Lend();
  EXPECT_EQ(deed2.get(), &o1);
  deed2.Return();
  EXPECT_EQ(deed2.get(), nullptr);
  EXPECT_EQ(deed1.get(), &o1);
}

TEST(Deed, Dereference) {
  TestObject o1(1);
  Deed<TestObject> deed1(&o1);
  EXPECT_EQ((*deed1).value, 1);
}

TEST(Deed, PointsTo) {
  TestObject o1(1);
  Deed<TestObject> deed1(&o1);
  EXPECT_EQ(deed1->value, 1);
}

// Move-only DeedObjects can be put a std::vector, and so can their deeds.
TEST(Deed, MoveOnlyVectorCompatible) {
  std::vector<NoCopy> objects;
  objects.emplace_back(0);
  std::vector<Deed<NoCopy>> deeds;
  deeds.emplace_back(&objects[0]);

  size_t initial_capacity = objects.capacity();
  size_t i = 1;
  while (objects.capacity() <= initial_capacity) {
    objects.emplace_back(i++);
    deeds.emplace_back(&objects.back());
  }

  for (i = 0; i < objects.size(); ++i) {
    EXPECT_EQ(deeds[i].get(), &objects[i]);
    EXPECT_EQ(deeds[i]->value, i);
  }
}

// Types that are noexcept movable and copyable can be put into a std::vector.
// Note that types that are copyable and movable with exception cannot be put
// in a std::vector.
TEST(Deed, NoExceptMoveVectorCompatible) {
  std::vector<TestObject> objects;
  objects.emplace_back(0);
  std::vector<Deed<TestObject>> deeds;
  deeds.emplace_back(&objects[0]);

  size_t initial_capacity = objects.capacity();
  size_t i = 1;
  while (objects.capacity() <= initial_capacity) {
    objects.emplace_back(i++);
    deeds.emplace_back(&objects.back());
  }

  for (i = 0; i < objects.size(); ++i) {
    EXPECT_EQ(deeds[i].get(), &objects[i]);
    EXPECT_EQ(deeds[i]->value, i);
  }
}

}  // walnut
