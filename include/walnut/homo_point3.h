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

  HomoPoint3(const BigIntImpl& x, const BigIntImpl& y, const BigIntImpl& z,
             const BigIntImpl& w) :
    vector_from_origin_(x, y, z), dist_denom_(w) { }

  HomoPoint3(long x, long y, long z, long w) :
    vector_from_origin_(x, y, z), dist_denom_(w) { }

  HomoPoint3(const Point3& other) :
    vector_from_origin_(other.vector_from_origin()), dist_denom_(1) { }

  HomoPoint3(const Vector3& v, const BigIntImpl& w) :
    vector_from_origin_(v), dist_denom_(w) { }

  BigIntImpl& x() {
    return vector_from_origin_.x();
  }

  const BigIntImpl& x() const {
    return vector_from_origin_.x();
  }

  BigIntImpl& y() {
    return vector_from_origin_.y();
  }

  const BigIntImpl& y() const {
    return vector_from_origin_.y();
  }

  BigIntImpl& z() {
    return vector_from_origin_.z();
  }

  const BigIntImpl& z() const {
    return vector_from_origin_.z();
  }

  BigIntImpl& w() {
    return dist_denom_;
  }

  const BigIntImpl& w() const {
    return dist_denom_;
  }

  Vector3& vector_from_origin() {
    return vector_from_origin_;
  }

  const Vector3& vector_from_origin() const {
    return vector_from_origin_;
  }

  BigIntImpl& dist_denom() {
    return dist_denom_;
  }

  const BigIntImpl& dist_denom() const {
    return dist_denom_;
  }

  static bool LexicographicallyLt(const HomoPoint3& a, const HomoPoint3& b) {
    // a.v / a.w <?> b.v / b.w
    // a.v <?> b.v * a.w / b.w (maybe flip sign)
    // a.v * b.w <?> b.v * a.w (maybe flip sign)
    int sign_flip = b.dist_denom().GetAbsMult(a.dist_denom());
    auto a_scaled = a.vector_from_origin() * (b.dist_denom() * sign_flip);
    auto b_scaled = b.vector_from_origin() * (a.dist_denom() * sign_flip);
    return std::lexicographical_compare(a_scaled.components().begin(),
                                        a_scaled.components().end(),
                                        b_scaled.components().begin(),
                                        b_scaled.components().end());
  }

  static bool TopnessLt(const HomoPoint3& a, const HomoPoint3& b) {
    // a.v / a.w <?> b.v / b.w
    // a.v <?> b.v * a.w / b.w (maybe flip sign)
    // a.v * b.w <?> b.v * a.w (maybe flip sign)
    auto a_scaled = a.vector_from_origin() * b.dist_denom();
    auto b_scaled = b.vector_from_origin() * a.dist_denom();
    BigIntImpl::FlippableCompare compare(
        /*flip=*/b.dist_denom().HasDifferentSign(a.dist_denom()));
    return std::lexicographical_compare(a_scaled.components().rbegin(),
                                        a_scaled.components().rend(),
                                        b_scaled.components().rbegin(),
                                        b_scaled.components().rend(),
                                        compare);
  }

  // Returns 0 if (p1, `this`, p3) are collinear.
  // Returns >0 if p3 is counter-clockwise from p1, with `this` as the center
  // point.
  // Returns <0 if p3 is clockwise from p1, with `this` as the center point.
  //
  // The calculations are done in 2D by removing (treating it as 0)
  // `drop_dimension` from the point.
  BigIntWord Get2DTwistDir(int drop_dimension, const HomoPoint3& p1,
                           const HomoPoint3& p3) const {
    // We roughly want to calculate:
    //   sign( (p1 - *this) x (p3 - *this) )
    //
    // However, each HomoPoint3 is like a fraction. The fractions must have
    // common bases to perform subtraction. However, only the pairs that will
    // be subtracted need to have common bases. The trick is to make two copies
    // of *this, one for p1's denominator and one for p3's denominator.
    //
    // The vectors (p1 - *this) and (p3 - *this) will have different scales,
    // but that's okay since we're only looking at the sign of the cross
    // product.
    Vector2 p1_from_origin =
      p1.vector_from_origin().DropDimension(drop_dimension) * dist_denom();
    Vector2 p2_from_origin1 =
      vector_from_origin().DropDimension(drop_dimension) * p1.dist_denom();
    Vector2 p2_from_origin3 =
      vector_from_origin().DropDimension(drop_dimension) * p3.dist_denom();
    Vector2 p3_from_origin =
      p3.vector_from_origin().DropDimension(drop_dimension) * dist_denom();
    return (p1_from_origin - p2_from_origin1).Cross(
        p3_from_origin - p2_from_origin3).GetSign();
  }

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
  BigIntImpl dist_denom_;
};

inline void HomoPoint3::Reduce() {
  auto common_factor = dist_denom_.GetGreatestCommonDivisor(
      vector_from_origin_.x());
  common_factor = common_factor.GetGreatestCommonDivisor(
      vector_from_origin_.y());
  common_factor = common_factor.GetGreatestCommonDivisor(
      vector_from_origin_.z());

  // Prioritize making the dist positive.
  if ((common_factor.GetSign() < 0) != (dist_denom_.GetSign() < 0)) {
    bool overflowed = common_factor.Negate();
    assert(!overflowed);
  }

  dist_denom_ /= common_factor;
  vector_from_origin_.x() /= common_factor;
  vector_from_origin_.y() /= common_factor;
  vector_from_origin_.z() /= common_factor;
}

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

inline std::string HomoPoint3::Approximate() const {
  std::ostringstream out;
  double w_double(w());
  out << "{ "
      << (double)x()/w_double << ", "
      << (double)y()/w_double << ", "
      << (double)z()/w_double
      << " }";
  return out.str();
}

inline std::ostream& operator<<(std::ostream& out, const HomoPoint3& p) {
  return out << "{ ["
             << p.x() << ", "
             << p.y() << ", "
             << p.z() << "] / "
             << p.w()
             << " }";
}

}  // walnut

#endif // WALNUT_HOMO_POINT3_H__
