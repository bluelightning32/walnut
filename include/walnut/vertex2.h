#ifndef WALNUT_VERTEX2_H__
#define WALNUT_VERTEX2_H__

#include "walnut/vector2.h"

namespace walnut {

template <int coord_bits_template = 32>
class Vertex2 {
 public:
  using VectorRep = Vector2<coord_bits_template>;
  using BigIntRep = typename VectorRep::BigIntRep;

  // The minimum number of bits to support for each coordinate.
  //
  // Note that the BigInt may round up the requested number of bits and end up
  // supporting more bits. Also note that the BigInt is faster when not all of
  // the requested bits are used.
  static constexpr int coord_bits = coord_bits_template;

  // Leaves the coordinates in an undefined state
  Vertex2() = default;

  template <int other_coord_bits>
  Vertex2(const Vertex2<other_coord_bits>& other) :
    Vertex2(other.coords()[0], other.coords()[1]) { }

  template <int other_coord_bits>
  Vertex2(const BigInt<other_coord_bits>& x,
          const BigInt<other_coord_bits>& y) :
    vector_from_origin_(x, y) { }

  Vertex2(int x, int y) : vector_from_origin_(x, y) { }

  std::array<BigIntRep, 2>& coords() { return vector_from_origin_.coords(); }

  const std::array<BigIntRep, 2>& coords() const { return vector_from_origin_.coords(); }

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

  template <int other_coord_bits>
  bool operator == (const Vertex2<other_coord_bits>& other) const {
    return vector_from_origin() == other.vector_from_origin();
  }

  template <int other_coord_bits>
  Vector2<std::max(other_coord_bits, coord_bits_template) + 1> operator-(
      const Vertex2<other_coord_bits>& other) const {
    return vector_from_origin() - other.vector_from_origin();
  }

  // Returns 0 if (p1, `this`, p3) are collinear.
  // Returns >0 if p3 is counter-clockwise from p1, with `this` as the center
  // point.
  // Returns <0 if p3 is clockwise from p1, with `this` as the center point.
  template <int other_coord_bits>
  BigIntWord GetTwistDir(const Vertex2<other_coord_bits>& p1,
                         const Vertex2<other_coord_bits>& p3) {
    return (p1 - *this).Cross(p3 - *this).GetSign();
  }

 private:
  VectorRep vector_from_origin_;
};

}  // walnut

#endif // WALNUT_VERTEX2_H__
