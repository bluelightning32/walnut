#include "walnut/half_space3.h"

namespace walnut {

void HalfSpace3::Reduce() {
  auto common_factor = normal_.components()[0];
  common_factor = common_factor.GetGreatestCommonDivisor(
      normal_.components()[1]);
  common_factor = common_factor.GetGreatestCommonDivisor(
      normal_.components()[2]);

  common_factor = common_factor.GetGreatestCommonDivisor(dist_);

  bool unused;
  auto abs_factor = common_factor.GetAbs(unused);

  dist_ /= abs_factor;

  normal_.components()[0] /= abs_factor;
  normal_.components()[1] /= abs_factor;
  normal_.components()[2] /= abs_factor;
}

std::ostream& operator<<(std::ostream& out, const HalfSpace3& p) {
  return out << "{ x*" << p.x()
             << " + y*" << p.y()
             << " + z*" << p.z()
             << " = " << p.d()
             << " }";
}

}  // walnut
