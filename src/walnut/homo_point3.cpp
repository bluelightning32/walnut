#include "walnut/homo_point3.h"

// For std::min
#include <algorithm>

#include "walnut/double.h"

namespace walnut {

namespace {

BigInt SafeShiftLeft(int64_t value, int shift) {
  if (shift >= 0) return BigInt(value) << shift;

  if (-shift >= (int)BigInt::bits_per_word) return BigInt(0);

  return BigInt(value >> -shift);
}

} // namespace

HomoPoint3 HomoPoint3::FromDoubles(int min_exponent, double x, double y,
                                   double z) {
  int x_exp, y_exp, z_exp;
  int64_t x_mantissa = Decompose(x, &x_exp);
  int64_t y_mantissa = Decompose(y, &y_exp);
  int64_t z_mantissa = Decompose(z, &z_exp);

  int denom_exp = std::min(0,
                           std::max(min_exponent,
                                    std::min({x_exp, y_exp, z_exp})));
  return HomoPoint3(SafeShiftLeft(x_mantissa, x_exp - denom_exp),
                    SafeShiftLeft(y_mantissa, y_exp - denom_exp),
                    SafeShiftLeft(z_mantissa, z_exp - denom_exp),
                    BigInt(1) << -denom_exp);
}

HomoPoint3 HomoPoint3::FromDoublesExact(double x, double y, double z) {
  int x_exp, y_exp, z_exp;
  int64_t x_mantissa = Decompose(x, &x_exp);
  int64_t y_mantissa = Decompose(y, &y_exp);
  int64_t z_mantissa = Decompose(z, &z_exp);

  int denom_exp = std::min({0, x_exp, y_exp, z_exp});
  return HomoPoint3(BigInt(x_mantissa) << (x_exp - denom_exp),
                    BigInt(y_mantissa) << (y_exp - denom_exp),
                    BigInt(z_mantissa) << (z_exp - denom_exp),
                    BigInt(1) << -denom_exp);
}

bool HomoPoint3::LexicographicallyLt(const HomoPoint3& a,
                                     const HomoPoint3& b) {
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

bool HomoPoint3::TopnessLt(const HomoPoint3& a, const HomoPoint3& b) {
  // a.v / a.w <?> b.v / b.w
  // a.v <?> b.v * a.w / b.w (maybe flip sign)
  // a.v * b.w <?> b.v * a.w (maybe flip sign)
  auto a_scaled = a.vector_from_origin() * b.dist_denom();
  auto b_scaled = b.vector_from_origin() * a.dist_denom();
  BigInt::FlippableCompare compare(
      /*flip=*/b.dist_denom().HasDifferentSign(a.dist_denom()));
  return std::lexicographical_compare(a_scaled.components().rbegin(),
                                      a_scaled.components().rend(),
                                      b_scaled.components().rbegin(),
                                      b_scaled.components().rend(),
                                      compare);
}

BigIntWord HomoPoint3::Get2DTwistDir(int drop_dimension, const HomoPoint3& p1,
                                     const HomoPoint3& p3) const {
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
  Vector2 p1_from_origin =
    p1.vector_from_origin().DropDimension(drop_dimension) * dist_denom();
  Vector2 p2_from_origin1 =
    vector_from_origin().DropDimension(drop_dimension) * p1.dist_denom();
  Vector2 p2_from_origin3 =
    vector_from_origin().DropDimension(drop_dimension) * p3.dist_denom();
  Vector2 p3_from_origin =
    p3.vector_from_origin().DropDimension(drop_dimension) * dist_denom();
  return (p1_from_origin - p2_from_origin1).Cross(
      p3_from_origin - p2_from_origin3).GetSign() ^
    p1.dist_denom().SignExtension() ^ p3.dist_denom().SignExtension();
}

void HomoPoint3::Reduce() {
  auto common_factor = dist_denom_.GetGreatestCommonDivisor(
      vector_from_origin_.x());
  common_factor = common_factor.GetGreatestCommonDivisor(
      vector_from_origin_.y());
  common_factor = common_factor.GetGreatestCommonDivisor(
      vector_from_origin_.z());

  // Prioritize making the dist positive.
  if ((common_factor.GetSign() < 0) != (dist_denom_.GetSign() < 0)) {
    common_factor.Negate();
  }

  dist_denom_ /= common_factor;
  vector_from_origin_.x() /= common_factor;
  vector_from_origin_.y() /= common_factor;
  vector_from_origin_.z() /= common_factor;
}

std::string HomoPoint3::Approximate() const {
  std::ostringstream out;
  Approximate(out);
  return out.str();
}

std::ostream& HomoPoint3::Approximate(std::ostream& out) const {
  double w_double(w());
  out << "{ "
      << (double)x()/w_double << ", "
      << (double)y()/w_double << ", "
      << (double)z()/w_double
      << " }";
  return out;
}

std::ostream& operator<<(std::ostream& out, const HomoPoint3& p) {
  return out << "{ ["
             << p.x() << ", "
             << p.y() << ", "
             << p.z() << "] / "
             << p.w()
             << " }";
}

}  // walnut
