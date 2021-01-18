#ifndef WALNUT_HOMO_POINT3_H__
#define WALNUT_HOMO_POINT3_H__

#include <sstream>

#include "walnut/homo_point2.h"
#include "walnut/point3.h"
#include "walnut/vector3.h"

namespace walnut {

// 3D point represented with homogeneous coordinates. The w component acts
// like a divisor for the x, y, and z component.
template <size_t num_bits_template = 32*7 + 9,
          size_t denom_bits_template = 32*6 + 7>
class HomoPoint3 {
 public:
  using VectorRep = Vector3<num_bits_template>;
  using NumInt = typename VectorRep::BigIntRep;
  using DenomInt = BigInt<denom_bits_template>;

  // The minimum number of bits to support for each of the x, y, and z
  // components.
  static constexpr size_t num_bits = num_bits_template;
  // The maximum number of bits supported for the x, y, and z components.
  static constexpr size_t max_num_bits = NumInt::max_bits;
  // The minimum number of bits to support for the w component.
  static constexpr size_t denom_bits = denom_bits_template;
  // The maximum number of bits supported for the w component.
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

  // Leaves the components in an undefined state
  HomoPoint3() = default;

  template <size_t other_num_bits, size_t other_denom_bits>
  HomoPoint3(const HomoPoint3<other_num_bits, other_denom_bits>& other) :
    vector_from_origin_(other.vector_from_origin()),
    dist_denom_(other.dist_denom()) { }

  template <size_t other_num_bits, size_t other_denom_bits>
  HomoPoint3(const BigInt<other_num_bits>& x,
          const BigInt<other_num_bits>& y,
          const BigInt<other_num_bits>& z,
          const BigInt<other_denom_bits>& w) :
    vector_from_origin_(x, y, z), dist_denom_(w) { }

  HomoPoint3(long x, long y, long z, long w) :
    vector_from_origin_(x, y, z), dist_denom_(w) { }

  template <size_t other_num_bits>
  HomoPoint3(const Point3<other_num_bits>& other) :
    vector_from_origin_(other.vector_from_origin()), dist_denom_(1) { }

  template <size_t other_num_bits, size_t other_denom_bits>
  HomoPoint3(const Vector3<other_num_bits>& v,
          const BigInt<other_denom_bits>& w) :
    vector_from_origin_(v), dist_denom_(w) { }

  template <size_t other_num_bits=num_bits, size_t other_denom_bits=denom_bits>
  static bool LexicographicallyLt(const HomoPoint3& a,
      const HomoPoint3<other_num_bits, other_denom_bits>& b) {
    // a.v / a.w <?> b.v / b.w
    // a.v <?> b.v * a.w / b.w (maybe flip sign)
    // a.v * b.w <?> b.v * a.w (maybe flip sign)
    int sign_flip = b.dist_denom().GetAbsMult(a.dist_denom());
    auto a_scaled = a.vector_from_origin() * (b.dist_denom() * sign_flip);
    auto b_scaled = b.vector_from_origin() * (a.dist_denom() * sign_flip);
    return std::lexicographical_compare(a_scaled.components().begin(),
                                        a_scaled.components().end(),
                                        b_scaled.components().begin(),
                                        b_scaled.components().end());
  }

  template <size_t other_num_bits=num_bits, size_t other_denom_bits=denom_bits>
  static bool TopnessLt(const HomoPoint3& a,
      const HomoPoint3<other_num_bits, other_denom_bits>& b) {
    // a.v / a.w <?> b.v / b.w
    // a.v <?> b.v * a.w / b.w (maybe flip sign)
    // a.v * b.w <?> b.v * a.w (maybe flip sign)
    int sign_flip = b.dist_denom().GetAbsMult(a.dist_denom());
    auto a_scaled = a.vector_from_origin() * (b.dist_denom() * sign_flip);
    auto b_scaled = b.vector_from_origin() * (a.dist_denom() * sign_flip);
    return std::lexicographical_compare(a_scaled.components().rbegin(),
                                        a_scaled.components().rend(),
                                        b_scaled.components().rbegin(),
                                        b_scaled.components().rend());
  }

  // Returns 0 if (p1, `this`, p3) are collinear.
  // Returns >0 if p3 is counter-clockwise from p1, with `this` as the center
  // point.
  // Returns <0 if p3 is clockwise from p1, with `this` as the center point.
  //
  // The calculations are done in 2D by removing (treating it as 0)
  // `drop_dimension` from the point.
  template <size_t other_num_bits, size_t other_denom_bits>
  BigIntWord Get2DTwistDir(int drop_dimension,
      const HomoPoint3<other_num_bits, other_denom_bits>& p1,
      const HomoPoint3<other_num_bits, other_denom_bits>& p3) const {
    // We roughly want to calculate:
    //   sign( (p1 - *this) x (p3 - *this) )
    //
    // However, each HomoPoint3 is like a fraction. The fractions must have
    // common bases to perform subtraction. However, only the pairs that will
    // be subtracted need to have common bases. The trick is to make two copies
    // of *this, one for p1's denominator and one for p3's denominator.
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

  HomoPoint2<num_bits, denom_bits> DropDimension(int drop_dimension) const {
    Vector2<num_bits> v = vector_from_origin().DropDimension(drop_dimension);
    return HomoPoint2<num_bits, denom_bits>(v, w());
  }

  // Note that everything equals the 0 point with a 0 denominator.
  template <size_t other_num_bits, size_t other_denom_bits>
  bool operator==(
      const HomoPoint3<other_num_bits, other_denom_bits>& other) const {
    return vector_from_origin().Scale(other.w()) ==
      other.vector_from_origin().Scale(w());
  }

  // Note that everything equals the 0 point with a 0 denominator.
  template <size_t other_bits>
  bool operator==(const Point3<other_bits>& other) const {
    return vector_from_origin() == other.vector_from_origin().Scale(w());
  }

  // Note that everything equals the 0 point with a 0 denominator.
  template <size_t other_num_bits, size_t other_denom_bits>
  bool operator!=(
      const HomoPoint3<other_num_bits, other_denom_bits>& other) const {
    return !(*this == other);
  }

  // Note that everything equals the 0 point with a 0 denominator.
  template <size_t other_bits>
  bool operator!=(const Point3<other_bits>& other) const {
    return !(*this == other);
  }

  bool IsValidState() const {
    return vector_from_origin_.IsValidState() && dist_denom_.IsValidState();
  }

  // Removes the common factor.
  //
  // After this function returns, the point may be stored in a more efficient
  // format, but the value of the point will be equivalent to the value from
  // before.
  void Reduce();

  // Return a string representation of the edge that uses decimal points to
  // approximate the vertex coordinates.
  std::string Approximate() const;

 private:
  VectorRep vector_from_origin_;
  DenomInt dist_denom_;
};

template <size_t num_bits, size_t denom_bits>
void HomoPoint3<num_bits, denom_bits>::Reduce() {
  bool dist_signed;
  auto common_factor = dist_denom_.GetUIntAbs(&dist_signed);

  bool unused;
  common_factor = common_factor.GetGreatestCommonDivisor(
      vector_from_origin_.x().GetUIntAbs(&unused));
  common_factor = common_factor.GetGreatestCommonDivisor(
      vector_from_origin_.y().GetUIntAbs(&unused));
  common_factor = common_factor.GetGreatestCommonDivisor(
      vector_from_origin_.z().GetUIntAbs(&unused));

  DenomInt signed_factor(common_factor);
  if (dist_signed) {
    bool overflowed = signed_factor.Negate();
    assert(!overflowed);
  }

  dist_denom_ /= signed_factor;
  vector_from_origin_.x() /= signed_factor;
  vector_from_origin_.y() /= signed_factor;
  vector_from_origin_.z() /= signed_factor;
}

template <size_t a_bits, size_t b_num_bits, size_t b_denom_bits>
bool operator==(const Point3<a_bits>& a,
                const HomoPoint3<b_num_bits, b_denom_bits>& b) {
  return b == a;
}

template <size_t num_bits, size_t denom_bits>
std::string HomoPoint3<num_bits, denom_bits>::Approximate() const {
  std::ostringstream out;
  double w_double(w());
  out << "{ "
      << (double)x()/w_double << ", "
      << (double)y()/w_double << ", "
      << (double)z()/w_double
      << " }";
  return out.str();
}

template <size_t num_bits, size_t denom_bits>
std::ostream& operator<<(std::ostream& out,
                         const HomoPoint3<num_bits, denom_bits>& p) {
  return out << "{ ["
             << p.x() << ", "
             << p.y() << ", "
             << p.z() << "] / "
             << p.w()
             << " }";
}

}  // walnut

#endif // WALNUT_HOMO_POINT3_H__
