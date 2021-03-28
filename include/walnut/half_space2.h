#ifndef WALNUT_HALF_SPACE2_H__
#define WALNUT_HALF_SPACE2_H__

#include "walnut/homo_point2.h"
#include "walnut/point2.h"
#include "walnut/vector2.h"

namespace walnut {

// Represents part of R^2 on one side of a line.
class HalfSpace2 {
 public:
  const BigInt& x() const {
    return normal_.x();
  }

  const BigInt& y() const {
    return normal_.y();
  }

  const Vector2& normal() const {
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

  // Leaves the coordinates in an undefined state
  HalfSpace2() = default;

  HalfSpace2(const Vector2& normal, const BigInt& dist) :
    normal_(normal), dist_(dist) { }

  HalfSpace2(const BigInt& x, const BigInt& y, const BigInt& dist) :
    normal_(x, y), dist_(dist) { }

  HalfSpace2(int x, int y, int dist) : normal_(x, y), dist_(dist) { }

  HalfSpace2(const HalfSpace2& other) :
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
  int Compare(const HomoPoint2& v) const {
    return normal_.Dot(v.vector_from_origin()).Compare(
        v.dist_denom() * dist_) * v.dist_denom().GetAbsMult();
  }

  // Returns true if the point is on the line
  bool IsCoincident(const HomoPoint2& v) const {
    return Compare(v) == 0;
  }

  // Returns a HalfSpace2 with an invalid 0 normal vector and a 0 distance.
  //
  // All vertices are coincident with the returned half-space. `IsValid` will
  // report false for the returned value.
  static HalfSpace2 Zero() {
    return HalfSpace2(/*normal=*/Vector2::Zero(), /*dist=*/BigInt(0));
  }

  // Note that everything equals the zero half-space.
  //
  // Two HalfSpace2s are not equal if they refer to different sides of the same
  // line.
  bool operator==(const HalfSpace2& other) const {
    BigInt scale_other;
    BigInt scale_mine;
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

  // Returns true if `other` shares the same line with `this.
  //
  // `other` will share the same line with `this` if it is the same half-space
  // or the opposite half-space.
  //
  // Lines with the same direction but different distances from the origin are
  // considered distinct.
  bool HasSameLine(const HalfSpace2& other) const {
    BigInt const *scale_other;
    BigInt const *scale_mine;
    if (!d().IsZero()) {
      scale_other = &d();
      scale_mine = &other.d();
    } else if (x().IsZero()) {
      scale_other = &x();
      scale_mine = &other.x();
    } else {
      scale_other = &y();
      scale_mine = &other.y();
    }

    return x().Multiply(*scale_mine) == other.x().Multiply(*scale_other) &&
           y().Multiply(*scale_mine) == other.y().Multiply(*scale_other) &&
           d().Multiply(*scale_mine) == other.d().Multiply(*scale_other);
  }

  // Note that everything equals the zero vector.
  bool operator!=(const HalfSpace2& other) const {
    return !(*this == other);
  }

  void Negate() {
    normal_.Negate();
    dist_.Negate();
  }

  HalfSpace2 operator-() const {
    return HalfSpace2(-normal_, -dist_);
  }

 private:
  Vector2 normal_;
  BigInt dist_;
};

std::ostream& operator<<(std::ostream& out, const HalfSpace2& p);

}  // walnut

#endif // WALNUT_HALF_SPACE2_H__
