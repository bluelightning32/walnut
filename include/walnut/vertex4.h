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

  template <int other_num_bits>
  Vertex4(const Vertex3<other_num_bits>& other) :
    vector_from_origin_(other.vector_from_origin()), dist_denom_(1) { }

  template <int other_num_bits=num_bits, int other_denom_bits=denom_bits>
  static bool LexicographicallyLt(const Vertex4& a,
      const Vertex4<other_num_bits, other_denom_bits>& b) {
    auto a_scaled = a.vector_from_origin() * b.dist_denom();
    auto b_scaled = b.vector_from_origin() * a.dist_denom();
    return std::lexicographical_compare(a_scaled.coords().begin(),
                                        a_scaled.coords().end(),
                                        b_scaled.coords().begin(),
                                        b_scaled.coords().end());
  }

  // Returns 0 if (p1, `this`, p3) are collinear.
  // Returns >0 if p3 is counter-clockwise from p1, with `this` as the center
  // point.
  // Returns <0 if p3 is clockwise from p1, with `this` as the center point.
  //
  // The calculations are done in 2D by removing (treating it as 0)
  // `drop_dimension` from the vertex.
  template <int other_num_bits, int other_denom_bits>
  BigIntWord Get2DTwistDir(int drop_dimension,
      const Vertex4<other_num_bits, other_denom_bits>& p1,
      const Vertex4<other_num_bits, other_denom_bits>& p3) const {
    // We roughly want to calculate:
    //   sign( (p1 - *this) x (p3 - *this) )
    //
    // However, each Vertex4 is like a fraction. The fractions must have common
    // bases to perform subtraction. However, only the pairs that will be
    // subtracted need to have common bases. The trick is to make two copies of
    // *this, one for p1's denominator and one for p3's denominator.
    //
    // The vectors (p1 - *this) and (p3 - *this) will have different scales,
    // but that's okay since we're only looking at the sign of the cross
    // product.
    Vector2<other_num_bits + denom_bits - 1> p1_from_origin =
      p1.vector_from_origin().DropDimension(drop_dimension) * dist_denom();
    Vector2<num_bits + other_denom_bits - 1> p2_from_origin1 =
      vector_from_origin().DropDimension(drop_dimension) * p1.dist_denom();
    Vector2<num_bits + other_denom_bits - 1> p2_from_origin3 =
      vector_from_origin().DropDimension(drop_dimension) * p3.dist_denom();
    Vector2<other_num_bits + denom_bits - 1> p3_from_origin =
      p3.vector_from_origin().DropDimension(drop_dimension) * dist_denom();
    return (p1_from_origin - p2_from_origin1).Cross(
        p3_from_origin - p2_from_origin3).GetSign();
  }

  // Note that everything equals the 0 vertex with a 0 denominator.
  template <int other_num_bits, int other_denom_bits>
  bool operator==(
      const Vertex4<other_num_bits, other_denom_bits>& other) const {
    return vector_from_origin().Scale(other.w()) ==
      other.vector_from_origin().Scale(w());
  }

  // Note that everything equals the 0 vertex with a 0 denominator.
  template <int other_bits>
  bool operator==(const Vertex3<other_bits>& other) const {
    return vector_from_origin() == other.vector_from_origin().Scale(w());
  }

 private:
  VectorRep vector_from_origin_;
  DenomInt dist_denom_;
};

template <int a_bits, int b_num_bits, int b_denom_bits>
bool operator==(const Vertex3<a_bits>& a,
                const Vertex4<b_num_bits, b_denom_bits>& b) {
  return b == a;
}

}  // walnut

#endif // WALNUT_VERTEX4_H__
