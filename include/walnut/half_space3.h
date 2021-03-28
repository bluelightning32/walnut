#ifndef WALNUT_HALF_SPACE3_H__
#define WALNUT_HALF_SPACE3_H__

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

  // Constructs a half-space a the normal and distance from the origin along
  // that normal.
  //
  // normal * dist a vector that goes from the origin to a point on the plane.
  // From that point, everything in the normal direction is considered in the
  // positive half-space and everything opposite to the normal direction is
  // considered in the negative half-space.
  HalfSpace3(const Vector3& normal, const BigInt& dist) :
    normal_(normal), dist_(dist) { }

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

  HalfSpace3(const HalfSpace3& other) :
    HalfSpace3(other.normal(), other.d()) { }

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

  // Determines which side of this half-space a parallel plane is one.
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

  // Note that everything equals the zero vector.
  bool operator!=(const HalfSpace3& other) const {
    return !(*this == other);
  }

  // This function could potentially overflow. The caller must ensure there is
  // sufficient bitspace.
  void Negate() {
    normal_.Negate();
    bool overflowed = dist_.Negate();
    assert(!overflowed);
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

 private:
  Vector3 normal_;
  BigInt dist_;
};

inline void HalfSpace3::Reduce() {
  auto common_factor = normal_.components()[0];
  common_factor = common_factor.GetGreatestCommonDivisor(
      normal_.components()[1]);
  common_factor = common_factor.GetGreatestCommonDivisor(
      normal_.components()[2]);

  common_factor = common_factor.GetGreatestCommonDivisor(dist_);

  bool unused;
  auto abs_factor = common_factor.GetAbs(unused);

  dist_ /= abs_factor;

  normal_.components()[0] /= abs_factor;
  normal_.components()[1] /= abs_factor;
  normal_.components()[2] /= abs_factor;
}

inline std::ostream& operator<<(std::ostream& out, const HalfSpace3& p) {
  return out << "{ x*" << p.x()
             << " + y*" << p.y()
             << " + z*" << p.z()
             << " = " << p.d()
             << " }";
}

}  // walnut

#endif // WALNUT_HALF_SPACE3_H__
