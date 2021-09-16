#include "walnut/r_transformation.h"

namespace walnut {

int RXYCompareBivector(const Vector3& u, const Vector3& v) {
  int comparexy = (u.x()*v.y()).Compare(u.y()*v.x());
  if (comparexy != 0) {
    return comparexy;
  }
  int comparexz = (u.z()*v.x()).Compare(u.x()*v.z());
  if (comparexz != 0) {
    return comparexz;
  }
  int compareyz = (u.y()*v.z()).Compare(u.z()*v.y());
  return compareyz;
}

int RXYCompareBivector(const Vector3& u, const SplitSide& v) {
  return RXYCompareBivector(u, v.split->normal()) * (v.pos_side ? -1 : 1);
}

}  // walnut
