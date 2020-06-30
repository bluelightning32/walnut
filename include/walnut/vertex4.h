#ifndef WALNUT_VERTEX4_H__
#define WALNUT_VERTEX4_H__

#include "walnut/vector.h"

namespace walnut {

// 3D vertex represented with homogeneous coordinates. The w coordinate acts
// like a divisor for the x, y, and z coordinates.
template <int num_bits_template = 32*7 + 9, int denom_bits_template = 32*6 + 7>
class Vertex4 {
 public:
  using VectorRep = Vector<num_bits_template>;
  using NumInt = typename VectorRep::BigIntRep;
  using DenomInt = BigInt<denom_bits_template>;

  // The minimum number of bits to support for each of the x, y, and z coordinates.
  static constexpr int num_bits = num_bits_template;
  // The maximum number of bits supported for the x, y, and z coordinates.
  static constexpr int max_num_bits = NumInt::max_bits;
  // The minimum number of bits to support for the w coordinate.
  static constexpr int denom_bits = denom_bits_template;
  // The maximum number of bits supported for the w coordinate.
  static constexpr int max_denom_bits = DenomInt::max_bits;

  NumInt& x() {
    return direction_.x();
  }

  NumInt& y() {
    return direction_.y();
  }

  NumInt& z() {
    return direction_.z();
  }

  DenomInt& w() {
    return denominator_;
  }

  // Leaves the coordinates in an undefined state
  Vertex4() = default;

  template <int other_num_bits, int other_denom_bits>
  Vertex4(const Vertex4<other_num_bits, other_denom_bits>& other) :
    direction_(other.direction_), denominator_(other.denominator_) { }

 private:
  VectorRep direction_;
  DenomInt denominator_;
};

}  // walnut

#endif // WALNUT_VERTEX4_H__
