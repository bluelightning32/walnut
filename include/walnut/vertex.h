#ifndef WALNUT_VERTEX3_H__
#define WALNUT_VERTEX3_H__

#include "walnut/vector.h"

namespace walnut {

template <int coord_bits_template = 32>
class Vertex3 {
 public:
  using VectorRep = Vector<coord_bits_template>;
  using BigIntRep = typename VectorRep::BigIntRep;

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
    Vertex3(other.coords()[0], other.coords()[1], other.coords()[2]) { }

  template <int other_coord_bits>
  Vertex3(const BigInt<other_coord_bits>& x,
          const BigInt<other_coord_bits>& y,
          const BigInt<other_coord_bits>& z) :
    vector_from_origin_(x, y, z) { }

  Vertex3(int x, int y, int z) : vector_from_origin_(x, y, z) { }

  std::array<BigIntRep, 3>& coords() { return vector_from_origin_.coords(); }

  const std::array<BigIntRep, 3>& coords() const { return vector_from_origin_.coords(); }

  const VectorRep& vector_from_origin() const {
    return vector_from_origin_;
  }

  BigIntRep& x() {
    return vector_from_origin_.x();
  }

  const BigIntRep& x() const {
    return vector_from_origin_.x();
  }

  BigIntRep& y() {
    return vector_from_origin_.y();
  }

  const BigIntRep& y() const {
    return vector_from_origin_.y();
  }

  BigIntRep& z() {
    return vector_from_origin_.z();
  }

  const BigIntRep& z() const {
    return vector_from_origin_.z();
  }

  template <int other_coord_bits>
  bool operator == (const Vertex3<other_coord_bits>& other) const {
    return vector_from_origin() == other.vector_from_origin();
  }

  template <int other_coord_bits>
  Vector<std::max(other_coord_bits, coord_bits_template) + 1> operator-(
      const Vertex3<other_coord_bits>& other) const {
    return vector_from_origin() - other.vector_from_origin();
  }

 private:
  Vector<coord_bits> vector_from_origin_;
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
