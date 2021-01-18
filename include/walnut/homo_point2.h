#ifndef WALNUT_HOMO_POINT2_H__
#define WALNUT_HOMO_POINT2_H__

#include "walnut/point2.h"
#include "walnut/vector2.h"

namespace walnut {

// 2D point represented with homogeneous coordinates. The w coordinate acts
// like a divisor for the x and y coordinates.
template <size_t num_bits_template = 32, size_t denom_bits_template = 32>
class HomoPoint2 {
 public:
  using VectorRep = Vector2<num_bits_template>;
  using NumInt = typename VectorRep::BigIntRep;
  using DenomInt = BigInt<denom_bits_template>;

  // The minimum number of bits to support for each of the x and y coordinates.
  static constexpr size_t num_bits = num_bits_template;
  // The maximum number of bits supported for the x and y coordinates.
  static constexpr size_t max_num_bits = NumInt::max_bits;
  // The minimum number of bits to support for the w coordinate.
  static constexpr size_t denom_bits = denom_bits_template;
  // The maximum number of bits supported for the w coordinate.
  static constexpr size_t max_denom_bits = DenomInt::max_bits;

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
  HomoPoint2() = default;

  template <size_t other_num_bits, size_t other_denom_bits>
  HomoPoint2(const HomoPoint2<other_num_bits, other_denom_bits>& other) :
    vector_from_origin_(other.vector_from_origin_), dist_denom_(other.dist_denom_) { }

  template <size_t other_num_bits, size_t other_denom_bits>
  HomoPoint2(const BigInt<other_num_bits>& x,
          const BigInt<other_num_bits>& y,
          const BigInt<other_denom_bits>& w) :
    vector_from_origin_(x, y), dist_denom_(w) { }

  HomoPoint2(int x, int y, int w) :
    vector_from_origin_(x, y), dist_denom_(w) { }

  template <size_t other_num_bits>
  HomoPoint2(const Point2<other_num_bits>& other) :
    vector_from_origin_(other.vector_from_origin()), dist_denom_(1) { }

  template <size_t other_num_bits, size_t other_denom_bits>
  HomoPoint2(const Vector2<other_num_bits>& v,
             const BigInt<other_denom_bits>& w) :
    vector_from_origin_(v), dist_denom_(w) { }

  template <size_t other_num_bits=num_bits, size_t other_denom_bits=denom_bits>
  static bool LexicographicallyLt(const HomoPoint2& a,
      const HomoPoint2<other_num_bits, other_denom_bits>& b) {
    auto a_scaled = a.vector_from_origin() * b.dist_denom();
    auto b_scaled = b.vector_from_origin() * a.dist_denom();
    return std::lexicographical_compare(a_scaled.coords().begin(),
                                        a_scaled.coords().end(),
                                        b_scaled.coords().begin(),
                                        b_scaled.coords().end());
  }

  // Note that everything equals the 0 point with a 0 denominator.
  template <size_t other_num_bits, size_t other_denom_bits>
  bool operator==(
      const HomoPoint2<other_num_bits, other_denom_bits>& other) const {
    return vector_from_origin().Scale(other.w()) ==
      other.vector_from_origin().Scale(w());
  }

  // Note that everything equals the 0 point with a 0 denominator.
  template <size_t other_bits>
  bool operator==(const Point2<other_bits>& other) const {
    return vector_from_origin() == other.vector_from_origin().Scale(w());
  }

  // Note that everything equals the 0 point with a 0 denominator.
  template <size_t other_num_bits, size_t other_denom_bits>
  bool operator!=(
      const HomoPoint2<other_num_bits, other_denom_bits>& other) const {
    return !(*this == other);
  }

  // Note that everything equals the 0 point with a 0 denominator.
  template <size_t other_bits>
  bool operator!=(const Point2<other_bits>& other) const {
    return !(*this == other);
  }

 private:
  VectorRep vector_from_origin_;
  DenomInt dist_denom_;
};

template <size_t a_bits, size_t b_num_bits, size_t b_denom_bits>
bool operator==(const Point2<a_bits>& a,
                const HomoPoint2<b_num_bits, b_denom_bits>& b) {
  return b == a;
}

template <size_t num_bits, size_t denom_bits>
std::ostream& operator<<(std::ostream& out, const HomoPoint2<num_bits, denom_bits>& p) {
  return out << "{ ["
             << p.x() << ", "
             << p.y() << "] / "
             << p.w()
             << " }";
}

}  // walnut

#endif // WALNUT_HOMO_POINT2_H__
