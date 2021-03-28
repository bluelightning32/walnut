#ifndef WALNUT_HOMO_POINT3_H__
#define WALNUT_HOMO_POINT3_H__

#include <sstream>

#include "walnut/homo_point2.h"
#include "walnut/point3.h"
#include "walnut/vector3.h"

namespace walnut {

// 3D point represented with homogeneous coordinates. The w component acts
// like a divisor for the x, y, and z component.
class HomoPoint3 {
 public:
  // Leaves the components in an undefined state
  HomoPoint3() = default;

  HomoPoint3(const HomoPoint3& other) :
    vector_from_origin_(other.vector_from_origin()),
    dist_denom_(other.dist_denom()) { }

  HomoPoint3(const BigInt& x, const BigInt& y, const BigInt& z,
             const BigInt& w) :
    vector_from_origin_(x, y, z), dist_denom_(w) { }

  HomoPoint3(long x, long y, long z, long w) :
    vector_from_origin_(x, y, z), dist_denom_(w) { }

  HomoPoint3(const Point3& other) :
    vector_from_origin_(other.vector_from_origin()), dist_denom_(1) { }

  HomoPoint3(const Vector3& v, const BigInt& w) :
    vector_from_origin_(v), dist_denom_(w) { }

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

  BigInt& w() {
    return dist_denom_;
  }

  const BigInt& w() const {
    return dist_denom_;
  }

  Vector3& vector_from_origin() {
    return vector_from_origin_;
  }

  const Vector3& vector_from_origin() const {
    return vector_from_origin_;
  }

  BigInt& dist_denom() {
    return dist_denom_;
  }

  const BigInt& dist_denom() const {
    return dist_denom_;
  }

  static bool LexicographicallyLt(const HomoPoint3& a, const HomoPoint3& b);

  static bool TopnessLt(const HomoPoint3& a, const HomoPoint3& b);

  // Returns 0 if (p1, `this`, p3) are collinear.
  // Returns >0 if p3 is counter-clockwise from p1, with `this` as the center
  // point.
  // Returns <0 if p3 is clockwise from p1, with `this` as the center point.
  //
  // The calculations are done in 2D by removing (treating it as 0)
  // `drop_dimension` from the point.
  BigIntWord Get2DTwistDir(int drop_dimension, const HomoPoint3& p1,
                           const HomoPoint3& p3) const;

  HomoPoint2 DropDimension(int drop_dimension) const {
    Vector2 v = vector_from_origin().DropDimension(drop_dimension);
    return HomoPoint2(v, w());
  }

  // Note that everything equals the 0 point with a 0 denominator.
  bool operator==(const HomoPoint3& other) const {
    return vector_from_origin().Scale(other.w()) ==
      other.vector_from_origin().Scale(w());
  }

  // Note that everything equals the 0 point with a 0 denominator.
  bool operator==(const Point3& other) const {
    return vector_from_origin() == other.vector_from_origin().Scale(w());
  }

  // Note that everything equals the 0 point with a 0 denominator.
  bool operator!=(const HomoPoint3& other) const {
    return !(*this == other);
  }

  // Note that everything equals the 0 point with a 0 denominator.
  bool operator!=(const Point3& other) const {
    return !(*this == other);
  }

  // Compares `component` from this and `other`, taking w and other.w into
  // account.
  //
  // Returns <0 if the component is less in this than other.
  // Returns 0 if the is equal in the two points.
  // Returns >0 if the component is larger in this than other.
  int CompareComponent(size_t component, const HomoPoint3& other) const;

  bool IsValid() const {
    return !dist_denom_.IsZero();
  }

  // Removes the common factor.
  //
  // After this function returns, the point may be stored in a more efficient
  // format, but the value of the point will be equivalent to the value from
  // before.
  void Reduce();

  // Return a string representation of the edge that uses decimal points to
  // approximate the vertex coordinates.
  std::string Approximate() const;

 private:
  Vector3 vector_from_origin_;
  BigInt dist_denom_;
};

inline int HomoPoint3::CompareComponent(size_t component,
                                        const HomoPoint3& other) const {
  // Check:
  // new_min/new_denom < old_min/old_denom
  // new_min*old_denom * sgn(new_denom*old_denom) <
  //   old_min*new_denom * sgn(new_denom*old_denom)
  auto scaled_this =
    vector_from_origin().components()[component] * other.w();
  auto scaled_other =
    other.vector_from_origin().components()[component] * w();
  return scaled_this.Compare(scaled_other) * other.w().GetAbsMult(w());
}

inline bool operator==(const Point3& a, const HomoPoint3& b) {
  return b == a;
}

std::ostream& operator<<(std::ostream& out, const HomoPoint3& p);

}  // walnut

#endif // WALNUT_HOMO_POINT3_H__
