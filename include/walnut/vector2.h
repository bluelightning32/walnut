#ifndef WALNUT_VECTOR2_H__
#define WALNUT_VECTOR2_H__

#include <array>

#include "walnut/big_int.h"

namespace walnut {

template <size_t coord_bits_template = 33>
class Vector2 {
  template <size_t other_coord_bits>
  friend class Vector2;

 public:
  using BigIntRep = BigInt<coord_bits_template>;

  // Compares two Vector2 by their rotation from the x axis through the y axis
  // and up to (but not including) the negative axis.
  //
  // More specifically for two vectors v and u, where the counter-clockwise
  // rotation from the x axis to each vector is in the range [0, 0.5), then
  // compare(v, u) returns true if the v's rotation is strictly less than u's.
  // That is, compare(v, u) acts like v < u. If v or u's rotation is outside of
  // that range, then the rotation of its negation is compared instead.
  //
  // This class meets the std requirements for a Compare. Notably it meets the
  // transitivity requirements.
  struct HalfRotationCompare {
   public:
    HalfRotationCompare() = default;

    bool operator()(const Vector2& v, const Vector2& u) const {
      return v.IsHalfRotationLessThan(u);
    }
  };

  // The minimum number of bits to support for each coordinate.
  //
  // Note that the BigInt may round up the requested number of bits and end up
  // supporting more bits. Also note that BigInts are faster when fewer bits
  // are requested, and of BigInts that have the same requested bits, instances
  // with fewer used bits are faster.
  static constexpr size_t coord_bits = coord_bits_template;

  // Leaves the coordinates in an undefined state
  Vector2() = default;

  template <size_t other_coord_bits>
  Vector2(const Vector2<other_coord_bits>& other) :
    coords_{other.coords()[0], other.coords()[1]} { }

  Vector2(const BigIntRep& x, const BigIntRep& y) :
    coords_{x, y} { }

  Vector2(int x, int y) :
    coords_{BigIntRep(x), BigIntRep(y)} { }

  BigIntRep& x() {
    return coords_[0];
  }

  const BigIntRep& x() const {
    return coords_[0];
  }

  BigIntRep& y() {
    return coords_[1];
  }

  const BigIntRep& y() const {
    return coords_[1];
  }

  std::array<BigIntRep, 2>& coords() {
    return coords_;
  }

  const std::array<BigIntRep, 2>& coords() const {
    return coords_;
  }

  template <size_t other_coord_bits>
  bool operator ==(const Vector2<other_coord_bits>& other) const {
    return x() == other.x() &&
           y() == other.y();
  }

  template <size_t other_coord_bits>
  Vector2<std::max(other_coord_bits, coord_bits) + 1> operator-(
      const Vector2<other_coord_bits>& other) const {
    return Vector2<std::max(other_coord_bits, coord_bits) + 1>(
        /*x=*/x() - other.x(),
        /*y=*/y() - other.y());
  }

  static Vector2 Zero() {
    return Vector2(/*x=*/0, /*y=*/0);
  }

  // Return true if the vectors have the same direction and only differ in
  // magnitude.
  //
  // Vectors with opposite magnitude are considered to have opposite
  // directions, and this function returns false for such vectors.
  template <size_t other_coord_bits>
  bool IsSameDir(const Vector2<other_coord_bits>& other) const;

  // Return true if the vectors have the same direction or opposite directions
  // and only differ in magnitude.
  template <size_t other_coord_bits>
  bool IsSameOrOppositeDir(const Vector2<other_coord_bits>& other) const;

  // Get the square of the scale of this vector
  BigInt<coord_bits*2 + 2> GetScaleSquared() const {
    return Dot(*this);
  }

  // Compute the dot product
  template <size_t other_coord_bits>
  BigInt<coord_bits + other_coord_bits + 1> Dot(const Vector2<other_coord_bits>& other) const {
    BigInt<coord_bits + other_coord_bits + 1> result = x() * other.x();
    result += y() * other.y();
    return result;
  }

  // Compute the cross product
  template <size_t other_coord_bits>
  BigInt<coord_bits + other_coord_bits> Cross(const Vector2<other_coord_bits>& other) const {
    return x()*other.y() - y()*other.x();
  }

  template <size_t other_bits>
  Vector2<coord_bits + other_bits> Scale(const BigInt<other_bits>& scale) const {
    return Vector2<coord_bits + other_bits>(x() * scale,
                                            y() * scale);
  }

  Vector2<coord_bits + sizeof(int)*8> Scale(int scale) const {
    return Scale<sizeof(int)*8>(BigInt<sizeof(int)*8>(scale));
  }

  template <size_t other_bits>
  Vector2<coord_bits + other_bits> operator*(
      const BigInt<other_bits>& scale) const {
    return Scale(scale);
  }

  Vector2<coord_bits + sizeof(int)*8> operator*(int scale) const {
    return Scale(scale);
  }

  bool IsZero() const {
    return coords_[0] == 0 && coords_[1] == 0;
  }

  // Get a vector that is a 1/4 turn counter-clockwise.
  //
  // This can overflow. It is up to the caller to ensure there is enough
  // bitspace.
  Vector2 GetPerpendicular() const {
    return Vector2(-y(), x());
  }

  // Swap the x and y coordinates
  //
  // Swapping the x and y coordinates is like reflecting the
  // vector across the x=y line.
  Vector2 SwapXY() const {
    return Vector2(y(), x());
  }

  // This function could potentially overflow. The caller must ensure there is
  // sufficient bitspace.
  void Negate() {
    for (BigIntRep& coord : coords_) {
      bool overflowed = coord.Negate();
      assert(!overflowed);
    }
  }

  // This function could potentially overflow. The caller must ensure there is
  // sufficient bitspace.
  Vector2 operator-() const {
    return Vector2(-x(), -y());
  }

  // Verifies the fields are in their supported ranges.
  //
  // The BigInts can sometimes internally support a larger range than what is
  // requested in the template parameters. This function returns true if all of
  // the fields are in their supported range.
  //
  // This function exists for testing purposes. It should always return true.
  bool IsValidState() const {
    return coords_[0].IsValidState() &&
           coords_[1].IsValidState();
  }

  // Returns true if the counter-clockwise angle from the x-axis to normalized
  // `this` is less than the angle from the x-axis to normalized `u`.
  //
  // In this context normalized means that if the counter-clockwise rotation
  // from the x-axis to the vector is half a rotation or greater, then the
  // vector is negated.
  //
  // Note that this comparison has the transitive property.
  template <size_t other_coord_bits>
  bool IsHalfRotationLessThan(const Vector2<other_coord_bits>& other) const;

 private:
  std::array<BigIntRep, 2> coords_;
};

template <size_t coord_bits>
template <size_t other_coord_bits>
inline bool Vector2<coord_bits>::IsSameDir(
    const Vector2<other_coord_bits>& other) const {
  BigInt<coord_bits> scale_other;
  BigInt<other_coord_bits> scale_mine;
  if (x() != 0) {
    scale_other = x().abs();
    scale_mine = other.x().abs();
  } else {
    scale_other = y().abs();
    scale_mine = other.y().abs();
  }

  return x().Multiply(scale_mine) == other.x().Multiply(scale_other) &&
         y().Multiply(scale_mine) == other.y().Multiply(scale_other);
}

template <size_t coord_bits>
template <size_t other_coord_bits>
inline bool Vector2<coord_bits>::IsSameOrOppositeDir(
    const Vector2<other_coord_bits>& other) const {
  BigInt<coord_bits> scale_other;
  BigInt<other_coord_bits> scale_mine;
  if (x() != 0) {
    scale_other = x();
    scale_mine = other.x();
  } else {
    scale_other = y();
    scale_mine = other.y();
  }

  return x().Multiply(scale_mine) == other.x().Multiply(scale_other) &&
         y().Multiply(scale_mine) == other.y().Multiply(scale_other);
}

template <size_t coord_bits>
template <size_t other_coord_bits>
inline bool Vector2<coord_bits>::IsHalfRotationLessThan(
    const Vector2<other_coord_bits>& other) const {
  // For determining whether to negate `this`, look at the sign of y(), when
  // y() != 0. If y() == 0, use the sign of x() instead.
  //
  // y_0_adjust will be 1 if y() is non-negative and x() is negative.
  //
  // So the sign of (y() - y_0_adjust) can be used to determine whether
  // to negate `this`.
  bool y_0_adjust = (y().GetSign() >= 0) & (x().GetSign() < 0);
  bool other_y_0_adjust = (other.y().GetSign() >= 0) &
                          (other.x().GetSign() < 0);
  const bool flip = (y() - int(y_0_adjust)).HasDifferentSign(
      other.y() - int(other_y_0_adjust));
  return (y() * other.x()).LessThan(flip, other.y() * x());
}

template <size_t coord_bits>
std::ostream& operator<<(std::ostream& out, const Vector2<coord_bits>& v) {
  return out << "{ "
             << v.coords()[0] << ", "
             << v.coords()[1]
             << " }";
}

}  // walnut

#endif // WALNUT_VECTOR2_H__
