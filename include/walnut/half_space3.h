#ifndef WALNUT_HALF_SPACE3_H__
#define WALNUT_HALF_SPACE3_H__

#include "walnut/homo_point3.h"
#include "walnut/point3.h"
#include "walnut/vector3.h"

namespace walnut {

// Sometimes this class is used just to represent a plane in R^3. Other times
// it is used to represent the positive half-space created by dividing R^3 in
// half at that plane.
template <int vector_bits_template = 31*2 + 3, int dist_bits_template = 31*3 + 3>
class HalfSpace3 {
 public:
  using VectorRep = Vector3<vector_bits_template>;
  using VectorInt = typename VectorRep::BigIntRep;
  using DistInt = BigInt<dist_bits_template>;

  // The minimum number of bits to support for each of the x, y, and z coordinates.
  static constexpr int vector_bits = vector_bits_template;
  // The minimum number of bits to support for the d coordinate.
  static constexpr int dist_bits = dist_bits_template;

  const VectorInt& x() const {
    return normal_.x();
  }

  const VectorInt& y() const {
    return normal_.y();
  }

  const VectorInt& z() const {
    return normal_.z();
  }

  const VectorRep& normal() const {
    return normal_;
  }

  DistInt& d() {
    return dist_;
  }

  const DistInt& d() const {
    return dist_;
  }

  bool IsValid() const {
    return !normal_.IsZero();
  }

  // Leaves the coordinates in an undefined state
  HalfSpace3() = default;

  template <int other_vector_bits>
  HalfSpace3(const Vector3<other_vector_bits>& normal, const DistInt& dist) :
    normal_(normal), dist_(dist) { }

  HalfSpace3(const VectorInt& x, const VectorInt& y, const VectorInt& z,
        const DistInt& dist) :
    normal_(x, y, z), dist_(dist) { }

  HalfSpace3(int x, int y, int z, int dist) : normal_(x, y, z), dist_(dist) { }

  template <int other_vector_bits, int other_dist_bits>
  HalfSpace3(const HalfSpace3<other_vector_bits, other_dist_bits>& other) :
    HalfSpace3(other.normal(), other.d()) { }

  template <int point_bits>
  HalfSpace3(const Point3<point_bits>& p1,
        const Point3<point_bits>& p2,
        const Point3<point_bits>& p3) :
    // Use p2 as the center point, because if p1, p2, and p3 are from a polygon
    // with more than 3 points, (p3 - p2) and (p1 - p2) are likely to be
    // shorter than (p2 - p1) and (p3 - p1).
    normal_((p3 - p2).Cross(p1 - p2)),
    dist_(normal_.Dot(p2.vector_from_origin())) { }

  // Returns >0 if `v` is in the half-space, 0 if `v` is coincident with the
  // plane, or <0 if `v` is outside of the half-space.
  template <int v_bits>
  int Compare(const Point3<v_bits>& v) const {
    return dist_.Compare(normal_.Dot(v.vector_from_origin()));
  }

  // Returns true if the point is on the plane
  template <int v_bits>
  bool IsCoincident(const Point3<v_bits>& v) const {
    return Compare(v) == 0;
  }

  // Returns >0 if `v` is in the half-space, 0 if `v` is coincident with the
  // plane, or <0 if `v` is outside of the half-space.
  template <int v_num_bits, int v_denom_bits>
  int Compare(const HomoPoint3<v_num_bits, v_denom_bits>& v) {
    return (v.dist_denom() * dist_).Compare(normal_.Dot(v.vector_from_origin()));
  }

  // Returns true if the point is on the plane
  template <int v_num_bits, int v_denom_bits>
  bool IsCoincident(const HomoPoint3<v_num_bits, v_denom_bits>& v) const {
    return Compare(v) == 0;
  }

  // Returns a plane with an invalid 0 normal vector and a 0 distance.
  //
  // All vertices are coincident with the returned plane. `IsValid` will report
  // false for the returned plane.
  static HalfSpace3 Zero() {
    return HalfSpace3(/*normal=*/VectorRep::Zero(), /*dist=*/DistInt(0));
  }

  // Note that everything equals the zero plane.
  //
  // Two HalfSpace3s are not equal if they refer to different half-spaces.
  template <int other_vector_bits, int other_dist_bits>
  bool operator==(
      const HalfSpace3<other_vector_bits, other_dist_bits>& other) const {
    BigInt<std::max(vector_bits, dist_bits)> scale_other;
    BigInt<std::max(other_vector_bits, other_dist_bits)> scale_mine;
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
  template <int other_vector_bits, int other_dist_bits>
  bool operator!=(
      const HalfSpace3<other_vector_bits, other_dist_bits>& other) const {
    return !(*this == other);
  }

  // Verifies the fields are in their supported ranges.
  //
  // The BigInts can sometimes internally support a larger range than what is
  // requested in the template parameters. This function returns true if all of
  // the fields are in their supported range.
  //
  // This function exists for testing purposes. It should always return true.
  bool IsValidState() const {
    return normal_.IsValidState() && dist_.IsValidState();
  }

  // This function could potentially overflow. The caller must ensure there is
  // sufficient bitspace.
  void Negate() {
    normal_.Negate();
    dist_.Negate();
  }

 private:
  VectorRep normal_;
  DistInt dist_;
};

// This is a wrapper around the HalfSpace3 constructor that takes 3 Point3's. The
// only reason to use this wrapper is that it figures out how many bits are
// necessary in the worst case for the plane numerator and denominator, given
// the number of bits in each Point3.
template <int point3_bits_template = 32>
class HalfSpace3FromPoint3Builder {
 public:
  using Point3Rep = Point3<point3_bits_template>;
  using HalfSpace3Rep = HalfSpace3<(point3_bits_template - 1)*2 + 3,
                         (point3_bits_template - 1)*3 + 3>;
  using VectorInt = typename HalfSpace3Rep::VectorInt;
  using DistInt = typename HalfSpace3Rep::DistInt;

  static constexpr VectorInt normal_component_min() {
    VectorInt n = Point3Rep::BigIntRep::max_value() + VectorInt(1);
    VectorInt two_n_1 = n + n - BigInt<2>(1);
    return -two_n_1 * two_n_1;
  }
  static constexpr VectorInt normal_component_max() {
    return -normal_component_min();
  }
  static constexpr DistInt dist_min() {
    DistInt n = Point3Rep::BigIntRep::max_value() + DistInt(1);
    return normal_component_min() * (n + DistInt(1));
  }
  static constexpr DistInt dist_max() {
    return -dist_min();
  }

  static HalfSpace3Rep Build(const Point3Rep& p1,
                        const Point3Rep& p2,
                        const Point3Rep& p3) {
    return HalfSpace3Rep(p1, p2, p3);
  }
};

template <int vector_bits, int dist_bits>
std::ostream& operator<<(std::ostream& out,
                         const HalfSpace3<vector_bits, dist_bits>& p) {
  return out << "{ x*" << p.x()
             << " + y*" << p.y()
             << " + z*" << p.z()
             << " = " << p.d()
             << " }";
}

}  // walnut

#endif // WALNUT_HALF_SPACE3_H__