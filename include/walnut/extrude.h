#ifndef WALNUT_EXTRUDE_H__
#define WALNUT_EXTRUDE_H__

#include "walnut/big_int.h"
#include "walnut/homo_point3.h"
#include "walnut/mutable_convex_polygon.h"
#include "walnut/vector3.h"

namespace walnut {

std::vector<MutableConvexPolygon<>> Extrude(
    const std::vector<HomoPoint3>& bottom_cap,
    const Vector3& direction, const BigInt& direction_denom);

}  // walnut

#endif // WALNUT_EXTRUDE_H__
