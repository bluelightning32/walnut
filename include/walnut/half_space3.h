#ifndef WALNUT_HALF_SPACE3_H__
#define WALNUT_HALF_SPACE3_H__

#include <tuple>

#include "walnut/half_space2.h"
#include "walnut/homo_point3.h"
#include "walnut/point3.h"
#include "walnut/vector3.h"

namespace walnut {

// Sometimes this class is used just to represent a plane in R^3. Other times
// it is used to represent the positive half-space created by dividing R^3 in
// half at that plane.
class HalfSpace3 {
 public:
  // Leaves the components in an undefined state
  HalfSpace3() = default;

  // Constructs a half-space from a normal and distance from the origin along
  // that normal.
  //
  // normal * dist a vector that goes from the origin to a point on the plane.
  // From that point, everything in the normal direction is considered in the
  // positive half-space and everything opposite to the normal direction is
  // considered in the negative half-space.
  HalfSpace3(const Vector3& normal, const BigInt& dist) :
    normal_(normal), dist_(dist) { }

  HalfSpace3(Vector3&& normal, BigInt&& dist) :
    normal_(std::move(normal)), dist_(std::move(dist)) { }

  // Constructs a half-space with the given normal and a point that should be
  // coincident to the plane.
  HalfSpace3(const Vector3& normal, const HomoPoint3& coincident) {
    bool was_signed;
    normal_ = normal * coincident.dist_denom().GetAbs(was_signed);
    dist_ = normal.Dot(coincident.vector_from_origin());
    if (was_signed) {
      dist_ = -dist_;
    }
  }

  // Constructs a half-space from a normal in component form and distance from
  // the origin along that normal.
  //
  // normal * dist a vector that goes from the origin to a point on the plane.
  // From that point, everything in the normal direction is considered in the
  // positive half-space and everything opposite to the normal direction is
  // considered in the negative half-space.
  HalfSpace3(const BigInt& x, const BigInt& y, const BigInt& z,
             const BigInt& dist) :
    normal_(x, y, z), dist_(dist) { }

  HalfSpace3(int x, int y, int z, int dist) : normal_(x, y, z), dist_(dist) { }

  HalfSpace3(const HalfSpace3&) = default;
  HalfSpace3(HalfSpace3&&) = default;

  HalfSpace3(const Point3& p1, const Point3& p2, const Point3& p3) :
    // Use p2 as the center point, because if p1, p2, and p3 are from a polygon
    // with more than 3 points, (p3 - p2) and (p1 - p2) are likely to be
    // shorter than (p2 - p1) and (p3 - p1).
    normal_((p3 - p2).Cross(p1 - p2)),
    dist_(normal_.Dot(p2.vector_from_origin())) { }

  HalfSpace3(const HomoPoint3& p1, const HomoPoint3& p2,
             const HomoPoint3& p3) {
    // Use p2 as the center point, because if p1, p2, and p3 are from a polygon
    // with more than 3 points, (p3 - p2) and (p1 - p2) are likely to be
    // shorter than (p2 - p1) and (p3 - p1).
    Vector3 unscaled_normal((p3.vector_from_origin()*p2.w() -
                             p2.vector_from_origin()*p3.w())
                            .Cross((p1.vector_from_origin()*p2.w() -
                                    p2.vector_from_origin()*p1.w()))
                            .Scale(p1.w().GetAbsMult(p3.w())));
    dist_ = unscaled_normal.Dot(p2.vector_from_origin()) * p2.w().GetAbsMult();
    normal_ = unscaled_normal.Scale(p2.w().abs());
  }

  // Extrudes a 2D half-space to form a 3D half-space.
  //
  // `projection` is a 2D projection of the final result, when `add_dimension`
  // is removed.
  //
  // Note that when dimension 1 is added, the 2d vector components are flipped,
  // that is the vector normal goes from <x, y> to <y, 0, x>.
  HalfSpace3(const HalfSpace2& projection, int add_dimension)
      : dist_(projection.d()) {
    assert(0 <= add_dimension);
    assert(add_dimension < 3);
    normal_.components()[(add_dimension + 1)%3] = projection.normal().x();
    normal_.components()[(add_dimension + 2)%3] = projection.normal().y();
  }

  // Creates a HalfSpace3 that is perpendicular to `dimension`.
  //
  // If `denominator` is positive, then the created positive half-space
  // includes all points that are further than numerator/denominator in
  // the `dimension` axis. Otherwise, the negative half-space includes all
  // points that are further than numerator/denominator in the `dimension`
  // axis.
  static HalfSpace3 GetAxisAligned(int dimension, BigInt numerator,
                                   BigInt denominator) {
    Vector3 normal = Vector3::Zero();
    normal.components()[dimension] = std::move(denominator);
    return HalfSpace3(std::move(normal), std::move(numerator));
  }

  HalfSpace3& operator=(const HalfSpace3&) = default;
  HalfSpace3& operator=(HalfSpace3&&) = default;

  const BigInt& x() const {
    return normal_.x();
  }

  const BigInt& y() const {
    return normal_.y();
  }

  const BigInt& z() const {
    return normal_.z();
  }

  const Vector3& normal() const {
    return normal_;
  }

  BigInt& d() {
    return dist_;
  }

  const BigInt& d() const {
    return dist_;
  }

  bool IsValid() const {
    return !normal_.IsZero();
  }

  // Returns >0 if `v` is in the positive half-space, 0 if `v` is coincident
  // with the plane, or <0 if `v` is in the negative half-space.
  int Compare(const Point3& v) const {
    return normal_.Dot(v.vector_from_origin()).Compare(dist_);
  }

  // Returns true if the point is on the plane
  bool IsCoincident(const Point3& v) const {
    return Compare(v) == 0;
  }

  // Returns >0 if `v` is in the positive half-space, 0 if `v` is coincident
  // with the plane, or <0 if `v` is in the negative half-space.
  int Compare(const HomoPoint3& v) const {
    return normal_.Dot(v.vector_from_origin()).Compare(
        v.dist_denom() * dist_) * v.dist_denom().GetAbsMult();
  }

  // Returns true if the point is on the plane
  bool IsCoincident(const HomoPoint3& v) const {
    return Compare(v) == 0;
  }

  // Determines which side of this half-space a parallel plane is on.
  //
  // Returns:
  //   >0:  if `p` is parallel to this half-space's plane and is fully
  //         contained within the positive half-space
  //    0:  if `p` is in the same plane as this half-space
  //   <0:  if `p` is parallel to this half-space's plane and is fully
  //        contained within the negative half-space.
  //
  // The return value is undefined if `p` is not parallel to this half-space,
  // or if the normal component with index `nonzero_dimension` is zero.
  int Compare(const HalfSpace3& p, int nonzero_dimension) const {
    return (d() * p.normal().components()[nonzero_dimension]).Compare(
        p.d() * normal_.components()[nonzero_dimension]) *
      p.normal().components()[nonzero_dimension].GetAbsMult();
  }

  // Returns a plane with an invalid 0 normal vector and a 0 distance.
  //
  // All vertices are coincident with the returned plane. `IsValid` will report
  // false for the returned plane.
  static HalfSpace3 Zero() {
    return HalfSpace3(/*normal=*/Vector3::Zero(), /*dist=*/BigInt(0));
  }

  // Note that everything equals the zero plane.
  //
  // Two HalfSpace3s are not equal if they refer to different half-spaces.
  bool operator==(const HalfSpace3& other) const {
    BigInt scale_other;
    BigInt scale_mine;
    if (d() != 0) {
      scale_other = d().abs();
      scale_mine = other.d().abs();
    } else if (x() != 0) {
      scale_other = x().abs();
      scale_mine = other.x().abs();
    } else if (y() != 0) {
      scale_other = y().abs();
      scale_mine = other.y().abs();
    } else {
      scale_other = z().abs();
      scale_mine = other.z().abs();
    }

    return x().Multiply(scale_mine) == other.x().Multiply(scale_other) &&
           y().Multiply(scale_mine) == other.y().Multiply(scale_other) &&
           z().Multiply(scale_mine) == other.z().Multiply(scale_other) &&
           d().Multiply(scale_mine) == other.d().Multiply(scale_other);
  }

  bool IsSameOrOpposite(const HalfSpace3& other) const {
    BigInt scale_other;
    BigInt scale_mine;
    if (d() != 0) {
      scale_other = d();
      scale_mine = other.d();
    } else if (x() != 0) {
      scale_other = x();
      scale_mine = other.x();
    } else if (y() != 0) {
      scale_other = y();
      scale_mine = other.y();
    } else {
      scale_other = z();
      scale_mine = other.z();
    }

    return x().Multiply(scale_mine) == other.x().Multiply(scale_other) &&
           y().Multiply(scale_mine) == other.y().Multiply(scale_other) &&
           z().Multiply(scale_mine) == other.z().Multiply(scale_other) &&
           d().Multiply(scale_mine) == other.d().Multiply(scale_other);
  }

  // Note that everything equals the zero vector.
  bool operator!=(const HalfSpace3& other) const {
    return !(*this == other);
  }

  // This function could potentially overflow. The caller must ensure there is
  // sufficient bitspace.
  void Negate() {
    normal_.Negate();
    dist_.Negate();
  }

  // This could overflow. It is the caller's responsibility to ensure that none
  // of the normal components are equal to their min_value and that dist is not
  // equal to its min_value.
  HalfSpace3 operator-() const {
    return HalfSpace3(-normal(), -d());
  }

  // Removes the common factor.
  //
  // After this function returns, the point may be stored in a more efficient
  // format, but the value of the point will be equivalent to the value from
  // before.
  void Reduce();

  // Projects a 2D point onto the plane by adding `add_dimension` to the point.
  HomoPoint3 ProjectOntoPlane(const HomoPoint2& p, int add_dimension) const;

 private:
  Vector3 normal_;
  BigInt dist_;
};

std::ostream& operator<<(std::ostream& out, const HalfSpace3& p);

// A comparison object for HalfSpace3 that uses an implementation defined
// order.
//
// This is useful for putting `HalfSpace3`s in a std::map or std::set.
struct HalfSpace3Compare {
  bool operator()(const HalfSpace3& a, const HalfSpace3& b) const {
    BigInt scale_a = (b.x() + b.y() + b.z()).abs();
    BigInt scale_b = (a.x() + a.y() + a.z()).abs();

    return std::make_tuple(a.x().Multiply(scale_a),
                           a.y().Multiply(scale_a),
                           a.z().Multiply(scale_a),
                           a.d().Multiply(scale_a)) <
           std::make_tuple(b.x().Multiply(scale_b),
                           b.y().Multiply(scale_b),
                           b.z().Multiply(scale_b),
                           b.d().Multiply(scale_b));
  }
};

}  // walnut

#endif // WALNUT_HALF_SPACE3_H__
