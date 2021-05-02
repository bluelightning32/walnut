#include "walnut/plucker_line.h"

namespace walnut {

PluckerLine::PluckerLine(const HalfSpace3& a, const HalfSpace3& b) :
  // d = a_xyz x b_xyz = (p^23, p^31, p^12)
  //
  // p^23 = | a.y  a.z |
  //        | b.y  b.z |
  //
  // p^31 = | a.z  a.x |
  //        | b.z  b.x |
  //
  // p^12 = | a.x  a.y |
  //        | b.x  b.y |
  //
  // m' = (p^01, p^02, p^03)
  //
  // p'^01 = | a.d  a.x |
  //         | b.d  b.x |
  //
  // p'^02 = | a.d  a.y |
  //         | b.d  b.y |
  //
  // p'^03 = | a.d  a.z |
  //         | b.d  b.z |
  //
  // However, the above equation for m only works if the plane equation is
  // in the form:
  //   p.x*a.x + p.y*a.y + p.z*a.z + a.d = 0
  //
  // However in Walnut the plane equations use this form instead:
  //   p.x*a.x + p.y*a.y + p.z*a.z = a.d
  //
  // So all of the distance components need to be negated:
  //
  // p^01 = | -a.d  a.x |
  //        | -b.d  b.x |
  //
  //      = | a.x a.d |
  //        | b.x b.d |
  //
  // p^02 = | -a.d  a.y |
  //        | -b.d  b.y |
  //
  //      = | a.y a.d |
  //        | b.y b.d |
  //
  // p^03 = | -a.d  a.z |
  //        | -b.d  b.z |
  //
  //      = | a.z a.d |
  //        | b.z b.d |
  //
  d_(a.normal().Cross(b.normal())),
  m_(BigInt::Determinant(a.x(), a.d(), b.x(), b.d()),
     BigInt::Determinant(a.y(), a.d(), b.y(), b.d()),
     BigInt::Determinant(a.z(), a.d(), b.z(), b.d())) { }

bool PluckerLine::operator==(const PluckerLine& other) const {
  BigInt scale_other;
  BigInt scale_mine;
  bool unused;
  if (!d().x().IsZero()) {
    scale_other = d().x().GetAbs(unused);
    scale_mine = other.d().x().GetAbs(unused);
  } else if (!d().y().IsZero()) {
    scale_other = d().y().GetAbs(unused);
    scale_mine = other.d().y().GetAbs(unused);
  } else {
    scale_other = d().z().GetAbs(unused);
    scale_mine = other.d().z().GetAbs(unused);
  }

  return
    d().x().Multiply(scale_mine) == other.d().x().Multiply(scale_other) &&
    d().y().Multiply(scale_mine) == other.d().y().Multiply(scale_other) &&
    d().z().Multiply(scale_mine) == other.d().z().Multiply(scale_other) &&
    m().x().Multiply(scale_mine) == other.m().x().Multiply(scale_other) &&
    m().y().Multiply(scale_mine) == other.m().y().Multiply(scale_other) &&
    m().z().Multiply(scale_mine) == other.m().z().Multiply(scale_other);
}

void PluckerLine::Reduce() {
  BigInt common_factor =
    d_.components()[0].GetGreatestCommonDivisor(d_.components()[1]);
  common_factor = common_factor.GetGreatestCommonDivisor(d_.components()[2]);

  common_factor = common_factor.GetGreatestCommonDivisor(m_.components()[0]);
  common_factor = common_factor.GetGreatestCommonDivisor(m_.components()[1]);
  common_factor = common_factor.GetGreatestCommonDivisor(m_.components()[2]);

  bool unused;
  BigInt abs_common_factor = common_factor.GetAbs(unused);

  d_.components()[0] /= abs_common_factor;
  d_.components()[1] /= abs_common_factor;
  d_.components()[2] /= abs_common_factor;

  m_.components()[0] /= abs_common_factor;
  m_.components()[1] /= abs_common_factor;
  m_.components()[2] /= abs_common_factor;
}

std::ostream& operator<<(std::ostream& out, const PluckerLine& line) {
  return out << "{ d=" << line.d() << " m=" << line.m() << " }";
}

}  // walnut
