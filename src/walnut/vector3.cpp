#include "walnut/vector3.h"

namespace walnut {

std::ostream& operator<<(std::ostream& out, const Vector3& v) {
  return out << "{ "
             << v.components()[0] << ", "
             << v.components()[1] << ", "
             << v.components()[2]
             << " }";
}

void Vector3::Reduce() {
  auto common_factor = components()[0];
  common_factor = common_factor.GetGreatestCommonDivisor(
      components()[1]);
  common_factor = common_factor.GetGreatestCommonDivisor(
      components()[2]);

  bool unused;
  auto abs_factor = common_factor.GetAbs(unused);

  components()[0] /= abs_factor;
  components()[1] /= abs_factor;
  components()[2] /= abs_factor;
}

}  // walnut
