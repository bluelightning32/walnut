#ifndef WALNUT_POINT3_H__
#define WALNUT_POINT3_H__

#include "walnut/point2.h"
#include "walnut/vector3.h"

namespace walnut {

class Point3 {
 public:
  // Leaves the components in an undefined state
  Point3() = default;

  explicit Point3(const Vector3& other) :
    Point3(other.components()[0], other.components()[1],
           other.components()[2]) { }

  Point3(const Point3& other) : Point3(other.vector_from_origin()) { }

  Point3(const BigInt& x, const BigInt& y, const BigInt& z) :
    vector_from_origin_(x, y, z) { }

  Point3(int x, int y, int z) : vector_from_origin_(x, y, z) { }

  std::array<BigInt, 3>& components() {
    return vector_from_origin_.components();
  }

  const std::array<BigInt, 3>& components() const {
    return vector_from_origin_.components();
  }

  const Vector3& vector_from_origin() const {
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

  BigInt& z() {
    return vector_from_origin_.z();
  }

  const BigInt& z() const {
    return vector_from_origin_.z();
  }

  bool operator == (const Point3& other) const {
    return vector_from_origin() == other.vector_from_origin();
  }

  bool operator != (const Point3& other) const {
    return vector_from_origin() != other.vector_from_origin();
  }

  Vector3 operator-(const Point3& other) const {
    return vector_from_origin() - other.vector_from_origin();
  }

  Point3 operator+(const Vector3& other) const {
    return Point3(vector_from_origin() + other);
  }

  Point3 operator-(const Vector3& other) const {
    return Point3(vector_from_origin() - other);
  }

  Point2 DropDimension(int drop_dimension) const {
    Vector2 v = vector_from_origin().DropDimension(drop_dimension);
    return Point2(v.x(), v.y());
  }

  // Returns 0 if (p1, `this`, p3) are collinear.
  // Returns >0 if p3 is counter-clockwise from p1, with `this` as the center
  // point.
  // Returns <0 if p3 is clockwise from p1, with `this` as the center point.
  //
  // The calculations are done in 2D by removing (treating it as 0)
  // `drop_dimension` from the point.
  BigIntWord Get2DTwistDir(int drop_dimension, const Point3& p1,
                           const Point3& p3) const {
    return DropDimension(drop_dimension).GetTwistDir(
        p1.DropDimension(drop_dimension), p3.DropDimension(drop_dimension));
  }

  // Returns 0 if (p1, `this`, p3) are collinear.
  // Returns 1 if p3 is counter-clockwise from p1, with `this` as the center
  // point.
  // Returns -1 if p3 is clockwise from p1, with `this` as the center point.
  //
  // The calculations are done in 2D by removing (treating it as 0)
  // `drop_dimension` from the point.
  int Get2DTwistDirReduced(int drop_dimension, const Point3& p1,
                           const Point3& p3) const {
    const BigIntWord twist = Get2DTwistDir(drop_dimension, p1, p3);
    if (twist > 0) {
      return 1;
    } else if (twist < 0) {
      return -1;
    } else {
      return 0;
    }
  }

  void Negate() {
    vector_from_origin_.Negate();
  }

  // Compares `component` from this and `other`.
  //
  // Returns <0 if the component is less in this than other.
  // Returns 0 if the is equal in the two points.
  // Returns >0 if the component is larger in this than other.
  int CompareComponent(size_t component, const Point3& other) const {
    return vector_from_origin().components()[component].Compare(
        other.vector_from_origin().components()[component]);
  }

  // Returns true if `component` from this and `other` are equal.
  bool IsEquivalentComponent(size_t component, const Point3& other) const {
    return vector_from_origin().components()[component] ==
        other.vector_from_origin().components()[component];
  }

 private:
  Vector3 vector_from_origin_;
};

inline std::ostream& operator<<(std::ostream& out, const Point3& v) {
  return out << v.vector_from_origin();
}

}  // walnut

#endif // WALNUT_POINT3_H__
