#ifndef WALNUT_HOMO_POINT3_H__
#define WALNUT_HOMO_POINT3_H__

#include <sstream>

#include "walnut/double_point3.h"
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

  HomoPoint3(const HomoPoint3&) = default;
  HomoPoint3(HomoPoint3&&) = default;

  HomoPoint3(const BigInt& x, const BigInt& y, const BigInt& z,
             const BigInt& w) :
    vector_from_origin_(x, y, z), dist_denom_(w) { }

  HomoPoint3(BigInt&& x, BigInt&& y, BigInt&& z, BigInt&& w) :
    vector_from_origin_(std::move(x), std::move(y), std::move(z)),
    dist_denom_(std::move(w)) { }

  HomoPoint3(long x, long y, long z, long w) :
    vector_from_origin_(x, y, z), dist_denom_(w) { }

  HomoPoint3(const Point3& other) :
    vector_from_origin_(other.vector_from_origin()), dist_denom_(1) { }

  HomoPoint3(const Vector3& v, const BigInt& w) :
    vector_from_origin_(v), dist_denom_(w) { }

  HomoPoint3& operator=(const HomoPoint3&) = default;
  HomoPoint3& operator=(HomoPoint3&&) = default;

  // Returns a HomoPoint3 that is the exact value of the double point.
  //
  // The returned point can take a lot of memory (about 600 bytes in the worst
  // case) if the exponents of any of the doubles are very high or low.
  //
  // The return value is unspecified if any of the input coordinates are not
  // finite (such as infinite or NaN).
  static HomoPoint3 FromDoublesExact(double x, double y, double z);

  // Returns a HomoPoint3 that is the value of the double point after rounding
  // to the grain size, where the grain size is 2^min_exponent.
  //
  // The return value is unspecified if any of the input coordinates are not
  // finite (such as infinite or NaN).
  static HomoPoint3 FromDoubles(int min_exponent, double x, double y,
                                double z);

  static HomoPoint3 FromDoublePoint3Exact(const DoublePoint3& p) {
    return FromDoublesExact(p.x, p.y, p.z);
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

  BigInt& w() {
    return dist_denom_;
  }

  const BigInt& w() const {
    return dist_denom_;
  }

  Vector3 Difference(const HomoPoint3& other, BigInt& denom) const {
    denom = other.dist_denom() * dist_denom();
    return vector_from_origin() * other.dist_denom() -
           other.vector_from_origin() * dist_denom();
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

  bool LexicographicallyLt(const HomoPoint3& other) const {
    return LexicographicallyLt(*this, other);
  }

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

  // Returns 0 if (p1, `this`, p3) are collinear.
  // Returns 1 if p3 is counter-clockwise from p1, with `this` as the center
  // point.
  // Returns -1 if p3 is clockwise from p1, with `this` as the center point.
  //
  // The calculations are done in 2D by removing (treating it as 0)
  // `drop_dimension` from the point.
  int Get2DTwistDirReduced(int drop_dimension, const HomoPoint3& p1,
                           const HomoPoint3& p3) const {
    const BigIntWord twist = Get2DTwistDir(drop_dimension, p1, p3);
    if (twist > 0) {
      return 1;
    } else if (twist < 0) {
      return -1;
    } else {
      return 0;
    }
  }

  HomoPoint2 DropDimension(int drop_dimension) const {
    Vector2 v = vector_from_origin().DropDimension(drop_dimension);
    return HomoPoint2(v, w());
  }

  BigInt& GetComponentAfterDrop(int component, int drop_dimension) {
    return vector_from_origin().GetComponentAfterDrop(component,
                                                      drop_dimension);
  }

  const BigInt& GetComponentAfterDrop(int component,
                                      int drop_dimension) const {
    return vector_from_origin().GetComponentAfterDrop(component,
                                                      drop_dimension);
  }

  DoublePoint3 GetDoublePoint3() const {
    long double w_d = (long double)w();
    long double x_d = (long double)x();
    long double y_d = (long double)y();
    long double z_d = (long double)z();
    return DoublePoint3(static_cast<double>(x_d / w_d),
                        static_cast<double>(y_d / w_d),
                        static_cast<double>(z_d / w_d));
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

  // Returns true if `component` from this and `other` are equivalent, taking w
  // and other.w into account.
  bool IsEquivalentComponent(size_t component, const HomoPoint3& other) const;

  bool IsValid() const {
    return !dist_denom_.IsZero();
  }

  HomoPoint3 AddOffset(const Vector3& direction,
                       const BigInt& direction_denom) const;

  // Removes the common factor.
  //
  // After this function returns, the point may be stored in a more efficient
  // format, but the value of the point will be equivalent to the value from
  // before.
  void Reduce();

  // Return a string representation of the point that uses decimal points to
  // approximate the coordinates.
  std::string Approximate() const;

  // Print a string representation of the point that uses decimal points to
  // approximate the coordinates.
  std::ostream& Approximate(std::ostream& out) const;

 private:
  Vector3 vector_from_origin_;
  BigInt dist_denom_;
};

// This hasher only works for points that have been reduced.
//
// If the points have not been reduced, two equal points could end up with
// different hashes.
struct ReducedHomoPoint3Hasher {
  size_t operator()(const HomoPoint3& p) const {
#ifndef NDEBUG
    HomoPoint3 reduced(p);
    reduced.Reduce();
    assert(p.w() == reduced.w());
#endif
    size_t raw_hashes[4] {
      p.x().GetHash(),
      p.y().GetHash(),
      p.z().GetHash(),
      p.w().GetHash(),
    };
    constexpr int size_t_bits = sizeof(size_t)*8;
    return raw_hashes[0] ^
           ((raw_hashes[1] << 1) | (raw_hashes[1] >> (size_t_bits - 1))) ^
           ((raw_hashes[2] << 2) | (raw_hashes[2] >> (size_t_bits - 2))) ^
           ((raw_hashes[3] << 3) | (raw_hashes[3] >> (size_t_bits - 3)));
  }
};

inline HomoPoint3 HomoPoint3::AddOffset(const Vector3& direction,
                                        const BigInt& direction_denom) const {
  return HomoPoint3(/*x=*/x()*direction_denom + direction.x()*w(),
                    /*y=*/y()*direction_denom + direction.y()*w(),
                    /*z=*/z()*direction_denom + direction.z()*w(),
                    /*w=*/direction_denom*w());
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

inline bool HomoPoint3::IsEquivalentComponent(size_t component,
                                             const HomoPoint3& other) const {
  // Check:
  // new_min/new_denom == old_min/old_denom
  // new_min*old_denom == old_min*new_denom
  return vector_from_origin().components()[component] * other.w() ==
         other.vector_from_origin().components()[component] * w();
}

inline bool operator==(const Point3& a, const HomoPoint3& b) {
  return b == a;
}

std::ostream& operator<<(std::ostream& out, const HomoPoint3& p);

}  // walnut

#endif // WALNUT_HOMO_POINT3_H__
