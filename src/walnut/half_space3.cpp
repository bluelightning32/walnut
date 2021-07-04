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

HomoPoint3 HalfSpace3::ProjectOntoPlane(const HomoPoint2& p,
                                        int add_dimension) const {
  // p.x/p.w * n.x + p.y/p.w * n.y + p.z/p.w * n.z = d
  // p.x/p.w * n.x + p.y/p.w * n.y - d = p.z/p.w * -n.z
  // p.x * n.x + p.y * n.y - d*p.w = p.z * -n.z
  // (p.x * n.x + p.y * n.y - d*p.w)/-n.z = p.z
  HomoPoint3 result;
  result.GetComponentAfterDrop(0, /*drop_dimension=*/add_dimension) =
    p.x() * normal().components()[add_dimension];
  result.GetComponentAfterDrop(1, /*drop_dimension=*/add_dimension) =
    p.y() * normal().components()[add_dimension];
  result.vector_from_origin().components()[add_dimension] =
    d() * p.w() -
    p.x() * normal().GetComponentAfterDrop(0,
                                           /*drop_dimension=*/add_dimension) -
    p.y() * normal().GetComponentAfterDrop(1,
                                           /*drop_dimension=*/add_dimension);
  result.w() = p.w() * normal().components()[add_dimension];
  return result;
}

std::ostream& operator<<(std::ostream& out, const HalfSpace3& p) {
  return out << "{ x*" << p.x()
             << " + y*" << p.y()
             << " + z*" << p.z()
             << " = " << p.d()
             << " }";
}

}  // walnut
