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
  static constexpr int max_coord_bits = BigIntRep::max_bits;

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

  BigIntRep& y() {
    return coords[1];
  }

  BigIntRep& z() {
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
  static constexpr int max_coord_bits = Vertex3Rep::max_coord_bits;

  Vertex3Rep rep;

  BigIntRep& x() {
    return rep.x();
  }

  BigIntRep& y() {
    return rep.y();
  }

  BigIntRep& z() {
    return rep.z();
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
