#ifndef WALNUT_POINT2_H__
#define WALNUT_POINT2_H__

#include "walnut/vector2.h"

namespace walnut {

template <size_t coord_bits_template = 32>
class Point2 {
 public:
  using VectorRep = Vector2<coord_bits_template>;

  // The minimum number of bits to support for each coordinate.
  //
  // Note that the BigInt may round up the requested number of bits and end up
  // supporting more bits. Also note that the BigInt is faster when not all of
  // the requested bits are used.
  static constexpr size_t coord_bits = coord_bits_template;

  // Leaves the coordinates in an undefined state
  Point2() = default;

  template <size_t other_coord_bits>
  Point2(const Point2<other_coord_bits>& other) :
    Point2(other.coords()[0], other.coords()[1]) { }

  Point2(const BigIntImpl& x, const BigIntImpl& y) :
    vector_from_origin_(x, y) { }

  Point2(int x, int y) : vector_from_origin_(x, y) { }

  std::array<BigIntImpl, 2>& coords() { return vector_from_origin_.coords(); }

  const std::array<BigIntImpl, 2>& coords() const { return vector_from_origin_.coords(); }

  const VectorRep& vector_from_origin() const {
    return vector_from_origin_;
  }

  BigIntImpl& x() {
    return vector_from_origin_.x();
  }

  const BigIntImpl& x() const {
    return vector_from_origin_.x();
  }

  BigIntImpl& y() {
    return vector_from_origin_.y();
  }

  const BigIntImpl& y() const {
    return vector_from_origin_.y();
  }

  template <size_t other_coord_bits>
  bool operator == (const Point2<other_coord_bits>& other) const {
    return vector_from_origin() == other.vector_from_origin();
  }

  template <size_t other_coord_bits>
  Vector2<std::max(other_coord_bits, coord_bits_template) + 1> operator-(
      const Point2<other_coord_bits>& other) const {
    return vector_from_origin() - other.vector_from_origin();
  }

  // Returns 0 if (p1, `this`, p3) are collinear.
  // Returns >0 if p3 is counter-clockwise from p1, with `this` as the center
  // point.
  // Returns <0 if p3 is clockwise from p1, with `this` as the center point.
  template <size_t other_coord_bits>
  BigIntWord GetTwistDir(const Point2<other_coord_bits>& p1,
                         const Point2<other_coord_bits>& p3) {
    return (p1 - *this).Cross(p3 - *this).GetSign();
  }

 private:
  VectorRep vector_from_origin_;
};

}  // walnut

#endif // WALNUT_POINT2_H__
