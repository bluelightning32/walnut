#ifndef WALNUT_HALF_SPACE2_H__
#define WALNUT_HALF_SPACE2_H__

#include "walnut/homo_point2.h"
#include "walnut/point2.h"
#include "walnut/vector2.h"

namespace walnut {

// Represents part of R^2 on one side of a line.
template <size_t vector_bits_template = 31*2 + 3,
          size_t dist_bits_template = 31*3 + 3>
class HalfSpace2 {
 public:
  using VectorRep = Vector2;
  using VectorInt = BigIntImpl;
  using DistInt = BigInt<dist_bits_template>;

  // The minimum number of bits to support for each of the x and y coordinates.
  static constexpr size_t vector_bits = vector_bits_template;
  // The minimum number of bits to support for the d coordinate.
  static constexpr size_t dist_bits = dist_bits_template;

  const VectorInt& x() const {
    return normal_.x();
  }

  const VectorInt& y() const {
    return normal_.y();
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
  HalfSpace2() = default;

  HalfSpace2(const Vector2& normal, const DistInt& dist) :
    normal_(normal), dist_(dist) { }

  HalfSpace2(const VectorInt& x, const VectorInt& y, const DistInt& dist) :
    normal_(x, y), dist_(dist) { }

  HalfSpace2(int x, int y, int dist) : normal_(x, y), dist_(dist) { }

  template <size_t other_vector_bits, size_t other_dist_bits>
  HalfSpace2(const HalfSpace2<other_vector_bits, other_dist_bits>& other) :
    HalfSpace2(other.normal(), other.d()) { }

  HalfSpace2(const Point2& p1, const Point2& p2) :
    normal_((p2 - p1).GetPerpendicular()),
    dist_(normal_.Dot(p1.vector_from_origin())) { }

  // Returns >0 if `v` is in the half-space, 0 if `v` is coincident with the
  // line, or <0 if `v` is outside of the half-space.
  int Compare(const Point2& v) const {
    return normal_.Dot(v.vector_from_origin()).Compare(dist_);
  }

  // Returns true if the point is on the line
  bool IsCoincident(const Point2& v) const {
    return Compare(v) == 0;
  }

  // Returns >0 if `v` is in the half-space, 0 if `v` is coincident with the
  // line, or <0 if `v` is outside of the half-space.
  template <size_t v_num_bits, size_t v_denom_bits>
  int Compare(const HomoPoint2<v_num_bits, v_denom_bits>& v) const {
    return normal_.Dot(v.vector_from_origin()).Compare(
        v.dist_denom() * dist_) * v.dist_denom().GetAbsMult();
  }

  // Returns true if the point is on the line
  template <size_t v_num_bits, size_t v_denom_bits>
  bool IsCoincident(const HomoPoint2<v_num_bits, v_denom_bits>& v) const {
    return Compare(v) == 0;
  }

  // Returns a HalfSpace2 with an invalid 0 normal vector and a 0 distance.
  //
  // All vertices are coincident with the returned half-space. `IsValid` will
  // report false for the returned value.
  static HalfSpace2 Zero() {
    return HalfSpace2(/*normal=*/VectorRep::Zero(), /*dist=*/DistInt(0));
  }

  // Note that everything equals the zero half-space.
  //
  // Two HalfSpace2s are not equal if they refer to different sides of the same
  // line.
  template <size_t other_vector_bits, size_t other_dist_bits>
  bool operator==(
      const HalfSpace2<other_vector_bits, other_dist_bits>& other) const {
    BigInt<std::max(vector_bits, dist_bits)> scale_other;
    BigInt<std::max(other_vector_bits, other_dist_bits)> scale_mine;
    if (d() != 0) {
      scale_other = d().abs();
      scale_mine = other.d().abs();
    } else if (x() != 0) {
      scale_other = x().abs();
      scale_mine = other.x().abs();
    } else {
      scale_other = y().abs();
      scale_mine = other.y().abs();
    }

    return x().Multiply(scale_mine) == other.x().Multiply(scale_other) &&
           y().Multiply(scale_mine) == other.y().Multiply(scale_other) &&
           d().Multiply(scale_mine) == other.d().Multiply(scale_other);
  }

  // Note that everything equals the zero vector.
  template <size_t other_vector_bits, size_t other_dist_bits>
  bool operator!=(
      const HalfSpace2<other_vector_bits, other_dist_bits>& other) const {
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
    return dist_.IsValidState();
  }

  // This function could potentially overflow. The caller must ensure there is
  // sufficient bitspace.
  void Negate() {
    normal_.Negate();
    bool overflowed = dist_.Negate();
    assert(!overflowed);
  }

  // This function could potentially overflow. The caller must ensure there is
  // sufficient bitspace.
  HalfSpace2 operator-() const {
    return HalfSpace2(-normal_, -dist_);
  }

 private:
  VectorRep normal_;
  DistInt dist_;
};

// This is a wrapper around the HalfSpace2 constructor that takes 2 Point2's.
// The only reason to use this wrapper is that it figures out how many bits are
// necessary in the worst case for the numerator and denominator, given the
// number of bits in each Point2.
template <size_t point3_bits_template = 32>
class HalfSpace2FromPoint2Builder {
 public:
  using HalfSpace2Rep = HalfSpace2<(point3_bits_template - 1)*1 + 2,
                                   (point3_bits_template - 1)*2 + 2>;
  using VectorInt = typename HalfSpace2Rep::VectorInt;
  using DistInt = typename HalfSpace2Rep::DistInt;

  static HalfSpace2Rep Build(const Point2& p1,
                             const Point2& p2) {
    return HalfSpace2Rep(p1, p2);
  }
};

template <size_t vector_bits, size_t dist_bits>
std::ostream& operator<<(std::ostream& out,
                         const HalfSpace2<vector_bits, dist_bits>& p) {
  return out << "{ x*" << p.x()
             << " + y*" << p.y()
             << " = " << p.d()
             << " }";
}

}  // walnut

#endif // WALNUT_HALF_SPACE2_H__
