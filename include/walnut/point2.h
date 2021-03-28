#ifndef WALNUT_POINT2_H__
#define WALNUT_POINT2_H__

#include "walnut/vector2.h"

namespace walnut {

class Point2 {
 public:
  // Leaves the coordinates in an undefined state
  Point2() = default;

  Point2(const Point2& other) :
    Point2(other.coords()[0], other.coords()[1]) { }

  Point2(const BigInt& x, const BigInt& y) :
    vector_from_origin_(x, y) { }

  Point2(int x, int y) : vector_from_origin_(x, y) { }

  std::array<BigInt, 2>& coords() { return vector_from_origin_.coords(); }

  const std::array<BigInt, 2>& coords() const { return vector_from_origin_.coords(); }

  const Vector2& vector_from_origin() const {
    return vector_from_origin_;
  }

  BigInt& x() {
    return vector_from_origin_.x();
  }

  const BigInt& x() const {
    return vector_from_origin_.x();
  }

  BigInt& y() {
    return vector_from_origin_.y();
  }

  const BigInt& y() const {
    return vector_from_origin_.y();
  }

  bool operator == (const Point2& other) const {
    return vector_from_origin() == other.vector_from_origin();
  }

  Vector2 operator-(const Point2& other) const {
    return vector_from_origin() - other.vector_from_origin();
  }

  // Returns 0 if (p1, `this`, p3) are collinear.
  // Returns >0 if p3 is counter-clockwise from p1, with `this` as the center
  // point.
  // Returns <0 if p3 is clockwise from p1, with `this` as the center point.
  BigIntWord GetTwistDir(const Point2& p1,
                         const Point2& p3) {
    return (p1 - *this).Cross(p3 - *this).GetSign();
  }

 private:
  Vector2 vector_from_origin_;
};

}  // walnut

#endif // WALNUT_POINT2_H__
