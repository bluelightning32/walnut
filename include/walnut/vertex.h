#ifndef WALNUT_VERTEX_H__
#define WALNUT_VERTEX_H__

#include "walnut/big_int.h"

namespace walnut {

template <int coord_bits_template = 32>
struct Vector3;

template <int coord_bits_template = 32>
struct Vertex3 {
  using BigIntRep = BigInt<coord_bits_template>;

  // The minimum number of bits to support for each coordinate.
  //
  // Note that the BigInt may round up the requested number of bits and end up
  // supporting more bits. Also note that the BigInt is faster when not all of
  // the requested bits are used.
  static constexpr int coord_bits = coord_bits_template;

  // Leaves the coordinates in an undefined state
  Vertex3() = default;

  template <int other_coord_bits>
  Vertex3(const Vertex3<other_coord_bits>& other) :
    coords{other.coords[0], other.coords[1], other.coords[2]} { }

  Vertex3(const BigIntRep& x, const BigIntRep& y, const BigIntRep& z) :
    coords{x, y, z} { }

  Vertex3(int x, int y, int z) : coords{BigIntRep{x}, BigIntRep{y}, BigIntRep{z}} { }

  BigIntRep& x() {
    return coords[0];
  }

  const BigIntRep& x() const {
    return coords[0];
  }

  BigIntRep& y() {
    return coords[1];
  }

  const BigIntRep& y() const {
    return coords[1];
  }

  BigIntRep& z() {
    return coords[2];
  }

  const BigIntRep& z() const {
    return coords[2];
  }

  template <int other_coord_bits>
  bool operator == (const Vertex3<other_coord_bits>& other) {
    return
      coords[0] == other.coords[0] &&
      coords[1] == other.coords[1] &&
      coords[2] == other.coords[2];
  }

  template <int other_coord_bits>
  Vector3<std::max(other_coord_bits, coord_bits) + 1> operator+(const Vertex3<other_coord_bits>& other);

  template <int other_coord_bits>
  Vector3<std::max(other_coord_bits, coord_bits) + 1> operator-(const Vertex3<other_coord_bits>& other);

  BigIntRep coords[3];
};

template <int coord_bits_template>
struct Vector3 {
  using Vertex3Rep = Vertex3<coord_bits_template>;
  using BigIntRep = typename Vertex3Rep::BigIntRep;

  // The minimum number of bits to support for each coordinate.
  static constexpr int coord_bits = coord_bits_template;

  Vertex3Rep rep;

  template <int other_coord_bits>
  Vector3(const Vector3<other_coord_bits>& other) :
    rep(other.coords[0], other.coords[1], other.coords[2]) { }

  Vector3(const BigIntRep& x, const BigIntRep& y, const BigIntRep& z) :
    rep(x, y, z) { }

  Vector3(int x, int y, int z) : rep(x, y, z) { }

  BigIntRep& x() {
    return rep.x();
  }

  const BigIntRep& x() const {
    return rep.x();
  }

  BigIntRep& y() {
    return rep.y();
  }

  const BigIntRep& y() const {
    return rep.y();
  }

  BigIntRep& z() {
    return rep.z();
  }

  const BigIntRep& z() const {
    return rep.z();
  }

  template <int other_coord_bits>
  bool operator ==(const Vector3<other_coord_bits>& other) const {
    return x() == other.x() &&
           y() == other.y() &&
           z() == other.z();
  }

  // Return true if the vectors have the same direction and only differ in
  // magnitude.
  //
  // Vectors with opposite magnitude are considered to have opposite
  // directions, and this function returns false for such vectors.
  template <int other_coord_bits>
  bool IsSameDir(const Vector3<other_coord_bits>& other) const {
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

  // Get the square of the scale of this vector
  BigInt<coord_bits*2 + 2> GetScaleSquared() const {
    return Dot(*this);
  }

  // Compute the dot product
  template <int other_coord_bits>
  BigInt<coord_bits + other_coord_bits + 2> Dot(const Vector3<other_coord_bits>& other) const {
    BigInt<coord_bits + other_coord_bits + 2> result = x() * other.x();
    result += y() * other.y();
    result += z() * other.z();
    return result;
  }

  template <int other_bits>
  Vector3<coord_bits + other_bits> Scale(const BigInt<other_bits>& scale) const {
    return Vector3<coord_bits + other_bits>(x() * scale,
                                            y() * scale,
                                            z() * scale);
  }

  Vector3<coord_bits + sizeof(int)*8> Scale(int scale) const {
    return Scale<sizeof(int)*8>(BigInt<sizeof(int)*8>(scale));
  }
};

// 3D vertex represented with homogeneous coordinates. The w coordinate acts
// like a divisor for the x, y, and z coordinates.
template <int num_bits_template = 32*7 + 9, int denom_bits_template = 32*6 + 7>
struct Vertex4 {
  using Vertex3Rep = Vertex3<num_bits_template>;
  using NumInt = typename Vertex3Rep::BigIntRep;
  using DenomInt = BigInt<denom_bits_template>;

  // The minimum number of bits to support for each of the x, y, and z coordinates.
  static constexpr int num_bits = num_bits_template;
  // The maximum number of bits supported for the x, y, and z coordinates.
  static constexpr int max_num_bits = NumInt::max_bits;
  // The minimum number of bits to support for the w coordinate.
  static constexpr int denom_bits = denom_bits_template;
  // The maximum number of bits supported for the w coordinate.
  static constexpr int max_denom_bits = DenomInt::max_bits;

  Vertex3Rep numerator;
  DenomInt denominator;

  NumInt& x() {
    return numerator.x();
  }

  NumInt& y() {
    return numerator.y();
  }

  NumInt& z() {
    return numerator.z();
  }

  DenomInt& w() {
    return denominator;
  }

  // Leaves the coordinates in an undefined state
  Vertex4() = default;

  template <int other_num_bits, int other_denom_bits>
  Vertex4(const Vertex4<other_num_bits, other_denom_bits>& other) :
    numerator(other.numerator), denominator(other.denominator) { }
};

}  // walnut

#endif // WALNUT_VERTEX_H__
