#ifndef WALNUT_VERTEX3_H__
#define WALNUT_VERTEX3_H__

#include "walnut/vector3.h"

namespace walnut {

template <int coord_bits_template = 32>
class Vertex3 {
 public:
  using VectorRep = Vector3<coord_bits_template>;
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
  Vector3<std::max(other_coord_bits, coord_bits_template) + 1> operator-(
      const Vertex3<other_coord_bits>& other) const {
    return vector_from_origin() - other.vector_from_origin();
  }

 private:
  VectorRep vector_from_origin_;
};

}  // walnut

#endif // WALNUT_VERTEX3_H__
