#ifndef WALNUT_POINT3_H__
#define WALNUT_POINT3_H__

#include "walnut/point2.h"
#include "walnut/vector3.h"

namespace walnut {

template <size_t component_bits_template = 32>
class Point3 {
 public:
  using VectorRep = Vector3;
  using BigIntRep = BigIntImpl;

  // The minimum number of bits to support for each component.
  //
  // Note that the BigInt may round up the requested number of bits and end up
  // supporting more bits. Also note that the BigInt is faster when not all of
  // the requested bits are used.
  static constexpr size_t component_bits = component_bits_template;

  // Leaves the components in an undefined state
  Point3() = default;

  explicit Point3(const Vector3& other) :
    Point3(other.components()[0], other.components()[1],
           other.components()[2]) { }

  template <size_t other_component_bits>
  Point3(const Point3<other_component_bits>& other) :
    Point3(other.vector_from_origin()) { }

  Point3(const BigIntImpl& x, const BigIntImpl& y, const BigIntImpl& z) :
    vector_from_origin_(x, y, z) { }

  Point3(int x, int y, int z) : vector_from_origin_(x, y, z) { }

  std::array<BigIntRep, 3>& components() {
    return vector_from_origin_.components();
  }

  const std::array<BigIntRep, 3>& components() const {
    return vector_from_origin_.components();
  }

  const VectorRep& vector_from_origin() const {
    return vector_from_origin_;
  }

  BigIntRep& x() {
    return vector_from_origin_.x();
  }

  const BigIntRep& x() const {
    return vector_from_origin_.x();
  }

  BigIntRep& y() {
    return vector_from_origin_.y();
  }

  const BigIntRep& y() const {
    return vector_from_origin_.y();
  }

  BigIntRep& z() {
    return vector_from_origin_.z();
  }

  const BigIntRep& z() const {
    return vector_from_origin_.z();
  }

  template <size_t other_component_bits>
  bool operator == (const Point3<other_component_bits>& other) const {
    return vector_from_origin() == other.vector_from_origin();
  }

  template <size_t other_component_bits>
  bool operator != (const Point3<other_component_bits>& other) const {
    return vector_from_origin() != other.vector_from_origin();
  }

  template <size_t other_component_bits>
  Vector3 operator-(const Point3<other_component_bits>& other) const {
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
  template <size_t other_component_bits>
  BigIntWord Get2DTwistDir(int drop_dimension,
                           const Point3<other_component_bits>& p1,
                           const Point3<other_component_bits>& p3) const {
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
  template <size_t other_component_bits>
  int Get2DTwistDirReduced(int drop_dimension,
                           const Point3<other_component_bits>& p1,
                           const Point3<other_component_bits>& p3) const {
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

 private:
  VectorRep vector_from_origin_;
};

template <size_t component_bits>
std::ostream& operator<<(std::ostream& out, const Point3<component_bits>& v) {
  return out << v.vector_from_origin();
}

}  // walnut

#endif // WALNUT_POINT3_H__
