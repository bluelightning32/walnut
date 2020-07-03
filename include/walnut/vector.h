#ifndef WALNUT_VECTOR_H__
#define WALNUT_VECTOR_H__

#include <array>

#include "walnut/big_int.h"

namespace walnut {

template <int coord_bits_template = 33>
class Vector {
  template <int other_coord_bits>
  friend class Vector;

 public:
  using BigIntRep = BigInt<coord_bits_template>;

  // The minimum number of bits to support for each coordinate.
  //
  // Note that the BigInt may round up the requested number of bits and end up
  // supporting more bits. Also note that BigInts are faster when fewer bits
  // are requested, and of BigInts that have the same requested bits, instances
  // with fewer used bits are faster.
  static constexpr int coord_bits = coord_bits_template;

  // Leaves the coordinates in an undefined state
  Vector() = default;

  template <int other_coord_bits>
  Vector(const Vector<other_coord_bits>& other) :
    coords_{other.coords()[0], other.coords()[1], other.coords()[2]} { }

  Vector(const BigIntRep& x, const BigIntRep& y, const BigIntRep& z) :
    coords_{x, y, z} { }

  Vector(int x, int y, int z) :
    coords_{BigIntRep(x), BigIntRep(y), BigIntRep(z)} { }

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

  BigIntRep& z() {
    return coords_[2];
  }

  const BigIntRep& z() const {
    return coords_[2];
  }

  std::array<BigIntRep, 3>& coords() {
    return coords_;
  }

  const std::array<BigIntRep, 3>& coords() const {
    return coords_;
  }

  template <int other_coord_bits>
  bool operator ==(const Vector<other_coord_bits>& other) const {
    return x() == other.x() &&
           y() == other.y() &&
           z() == other.z();
  }

  template <int other_coord_bits>
  Vector<std::max(other_coord_bits, coord_bits_template) + 1> operator-(
      const Vector<other_coord_bits>& other) const {
    return Vector<std::max(other_coord_bits, coord_bits_template) + 1>(
        /*x=*/x() - other.x(),
        /*y=*/y() - other.y(),
        /*z=*/z() - other.z());
  }

  // Return true if the vectors have the same direction and only differ in
  // magnitude.
  //
  // Vectors with opposite magnitude are considered to have opposite
  // directions, and this function returns false for such vectors.
  template <int other_coord_bits>
  bool IsSameDir(const Vector<other_coord_bits>& other) const;

  // Return true if the vectors have the same direction or opposite directions
  // and only differ in magnitude.
  template <int other_coord_bits>
  bool IsSameOrOppositeDir(const Vector<other_coord_bits>& other) const;

  // Get the square of the scale of this vector
  BigInt<coord_bits*2 + 2> GetScaleSquared() const {
    return Dot(*this);
  }

  // Compute the dot product
  template <int other_coord_bits>
  BigInt<coord_bits + other_coord_bits + 1> Dot(const Vector<other_coord_bits>& other) const {
    BigInt<coord_bits + other_coord_bits + 1> result = x() * other.x();
    result += y() * other.y();
    result += z() * other.z();
    return result;
  }

  // Compute the cross product
  template <int other_coord_bits>
  Vector<coord_bits + other_coord_bits> Cross(const Vector<other_coord_bits>& other) const {
    return Vector<coord_bits + other_coord_bits>(
        /*x=*/y()*other.z() - z()*other.y(),
        /*y=*/z()*other.x() - x()*other.z(),
        /*z=*/x()*other.y() - y()*other.x());
  }

  template <int other_bits>
  Vector<coord_bits + other_bits> Scale(const BigInt<other_bits>& scale) const {
    return Vector<coord_bits + other_bits>(x() * scale,
                                            y() * scale,
                                            z() * scale);
  }

  Vector<coord_bits + sizeof(int)*8> Scale(int scale) const {
    return Scale<sizeof(int)*8>(BigInt<sizeof(int)*8>(scale));
  }

  bool IsZero() const {
    return coords_[0] == 0 && coords_[1] == 0 && coords_[2] == 0;
  }

 private:
  std::array<BigIntRep, 3> coords_;
};

template <int coord_bits_template>
template <int other_coord_bits>
inline bool Vector<coord_bits_template>::IsSameDir(
    const Vector<other_coord_bits>& other) const {
  BigInt<coord_bits> scale_other;
  BigInt<other_coord_bits> scale_mine;
  if (x() != 0) {
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
         z().Multiply(scale_mine) == other.z().Multiply(scale_other);
}

template <int coord_bits_template>
template <int other_coord_bits>
inline bool Vector<coord_bits_template>::IsSameOrOppositeDir(
    const Vector<other_coord_bits>& other) const {
  BigInt<coord_bits> scale_other;
  BigInt<other_coord_bits> scale_mine;
  if (x() != 0) {
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
         z().Multiply(scale_mine) == other.z().Multiply(scale_other);
}

}  // walnut

#endif // WALNUT_VECTOR_H__
