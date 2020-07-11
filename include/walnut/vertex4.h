#ifndef WALNUT_VERTEX4_H__
#define WALNUT_VERTEX4_H__

#include "walnut/vector3.h"

namespace walnut {

// 3D vertex represented with homogeneous coordinates. The w coordinate acts
// like a divisor for the x, y, and z coordinates.
template <int num_bits_template = 32*7 + 9, int denom_bits_template = 32*6 + 7>
class Vertex4 {
 public:
  using VectorRep = Vector3<num_bits_template>;
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
    return vector_from_origin_.x();
  }

  const NumInt& x() const {
    return vector_from_origin_.x();
  }

  NumInt& y() {
    return vector_from_origin_.y();
  }

  const NumInt& y() const {
    return vector_from_origin_.y();
  }

  NumInt& z() {
    return vector_from_origin_.z();
  }

  const NumInt& z() const {
    return vector_from_origin_.z();
  }

  DenomInt& w() {
    return dist_denom_;
  }

  const DenomInt& w() const {
    return dist_denom_;
  }

  VectorRep& vector_from_origin() {
    return vector_from_origin_;
  }

  const VectorRep& vector_from_origin() const {
    return vector_from_origin_;
  }

  DenomInt& dist_denom() {
    return dist_denom_;
  }

  const DenomInt& dist_denom() const {
    return dist_denom_;
  }

  // Leaves the coordinates in an undefined state
  Vertex4() = default;

  template <int other_num_bits, int other_denom_bits>
  Vertex4(const Vertex4<other_num_bits, other_denom_bits>& other) :
    vector_from_origin_(other.vector_from_origin_), dist_denom_(other.dist_denom_) { }

  template <int other_num_bits, int other_denom_bits>
  Vertex4(const BigInt<other_num_bits>& x,
          const BigInt<other_num_bits>& y,
          const BigInt<other_num_bits>& z,
          const BigInt<other_denom_bits>& w) :
    vector_from_origin_(x, y, z), dist_denom_(w) { }

  Vertex4(int x, int y, int z, int w) :
    vector_from_origin_(x, y, z), dist_denom_(w) { }

 private:
  VectorRep vector_from_origin_;
  DenomInt dist_denom_;
};

}  // walnut

#endif // WALNUT_VERTEX4_H__
