#ifndef WALNUT_R_TRANFORMATION_H__
#define WALNUT_R_TRANFORMATION_H__

// In this context, if p is reverse lexicographically less than q, that means:
//   p.z < q.z ||
//   p.y < q.y ||
//   p.x < q.x
//
// R transformation is an R^3 shear transformation. The name is short for
// reverse lexicographical transformation. For to points, p and q, iff. p is
// reverse lexicographically less than q, then (R p).z < (R q).z.
//
// The R transform takes a parameter, 0 < epsilon < 1, which should be small
// enough that comparisons of R-transformed values stabilize.

#include "walnut/homo_point3.h"
#include "walnut/split_side.h"
#include "walnut/vector3.h"

namespace walnut {

// R transforms a point. Epsilon is specified by 1/eps_inv.
//
// Since this function is likely to produce a huge number of bits, this
// function should only be used for testing purposes.
inline HomoPoint3 RTransform(const HomoPoint3& p, const BigInt& eps_inv) {
  BigInt eps_inv_squared(eps_inv * eps_inv);
  // x and y stay the same. z' = z + y/eps + x/eps^2.
  return HomoPoint3(p.x()*eps_inv_squared,
                    p.y()*eps_inv_squared,
                    (p.z()*eps_inv + p.y())*eps_inv + p.x(),
                    p.w()*eps_inv_squared);
}

// Applies the R transform to two bivectors and compares the result in the XY
// plane.
//
// This function should not be used for the vector from the origin of a Point3,
// because that is a contravariant vector, not a bivector.
//
// This function can be used on HalfSpace3 normals, because they are bivectors.
//
// * Returns -1 if (v R) is clockwise from (u R)
// * Returns 0 if u points in the same or opposite direction as v (as do (u R)
//   and (v R)).
// * Returns 1 if (v R) is counter-clockwise from (u R)
//
// If either vector is 0, then 0 is returned.
int RXYCompareBivector(const Vector3& u, const Vector3& v);

// Applies the R transform to a bivector and a split normal and compares the
// result in the XY plane.
//
// The split normal is taken from `v`. If `v.pos_side` is true, the split
// normal is flipped such that the normal always points away from the point
// that the `SplitSide` is relative to.
//
// * Returns -1 if (v R) is clockwise from (u R)
// * Returns 0 if u points in the same or opposite direction as v (as do (u R)
//   and (v R)).
// * Returns 1 if (v R) is counter-clockwise from (u R)
//
// If either vector is 0, then 0 is returned.
int RXYCompareBivector(const Vector3& u, const SplitSide& v);

}  // walnut

#endif // WALNUT_R_TRANFORMATION_H__
