#ifndef WALNUT_PLUCKER_LINE_H__
#define WALNUT_PLUCKER_LINE_H__

#include "walnut/half_space2.h"
#include "walnut/half_space3.h"
#include "walnut/point3.h"
#include "walnut/vector3.h"

namespace walnut {

// Plücker coordinates are a way to store a R^3 line with homogeneous
// coordinates.
class PluckerLine {
 public:
  // Returns the direction vector for the line.
  //
  // If the line was constructed using points, the direction vector points from
  // p1 to p2.
  //
  // Some definitions use the p^ab or p_ab notation.
  // d = (p_01, p_02, p_03)
  // d = (p^23, p^31, p^12)
  const Vector3& d() const {
    return d_;
  }

  Vector3& d() {
    return d_;
  }

  // Returns the moment of the line.
  //
  // The moment indirectly indicates how far the direction vector is from the
  // origin. The moment is zero if and only if the line goes through the
  // origin.
  //
  // Some definitions use the p^ab or p_ab notation.
  // m = (p_23, p_31, p_12)
  // m = (p^01, p^02, p^03)
  const Vector3& m() const {
    return m_;
  }

  Vector3& m() {
    return m_;
  }

  // Returns true if this line was correctly constructed.
  bool IsValid() const {
    // m_ may be 0 if the line goes through the origin.
    return !d_.IsZero();
  }

  // Leaves the vectors in an undefined state
  PluckerLine() = default;

  PluckerLine(const Vector3& d, const Vector3& m) :
    d_(d), m_(m) { }

  PluckerLine(const PluckerLine&) = default;
  PluckerLine(PluckerLine&&) = default;

  PluckerLine(const Point3& p1, const Point3& p2) :
    d_(p2 - p1), m_(p1.vector_from_origin().Cross(p2.vector_from_origin())) { }

  PluckerLine(const HomoPoint3& p1, const HomoPoint3& p2) :
    d_((p2.vector_from_origin().Scale(p1.w()) -
        p1.vector_from_origin().Scale(p2.w())) * p1.w().GetAbsMult(p2.w())),
    m_(p1.vector_from_origin().Cross(p2.vector_from_origin()) *
       (p1.w().GetAbsMult(p2.w()))) { }

  // Constructs the line for the intersection of the two planes, `a` and `b`.
  //
  // Both `a` and `b` must be valid (have non-zero normals), and they must be
  // non-equal, otherwise `IsValid` will return false on the constructed line.
  PluckerLine(const HalfSpace3& a, const HalfSpace3& b);

  PluckerLine& operator=(const PluckerLine&) = default;
  PluckerLine& operator=(PluckerLine&&) = default;

  // Returns true if `p` is on the line.
  bool IsCoincident(const Point3& p) const {
    /*
     * p x (p + d) == m
     * p x p + p x d == m
     * p x d == m
     */
    return p.vector_from_origin().Cross(d_) == m();
  }

  // Returns true if `p` is on the line.
  bool IsCoincident(const HomoPoint3& p) const {
    /*
     * p x (p + p.dist*d) == m
     * p x p + p x (p.dist*d) == m
     * p x (p.dist*d) == m
     */
    return p.vector_from_origin().Cross(d_) == m() * p.dist_denom();
  }

  // Returns true if the line is on the plane.
  bool IsCoincident(const HalfSpace3& p) const {
    return d().Dot(p.normal()).IsZero();
  }

  // Returns true when the lines match
  //
  // Two plucker lines are considered equal if they describe the same set
  // of points, and their d vectors point the same direction. Notably the lines
  // may have a different common scale factors.
  //
  // The lines are considered not equal if their directions are opposite (one
  // scale factor is negative).
  bool operator==(const PluckerLine& other) const;

  bool operator!=(const PluckerLine& other) const {
    return !(*this == other);
  }

  // Calculate the intersection between this line and a plane.
  HomoPoint3 Intersect(const HalfSpace3& p) const {
    auto vector = p.normal().Cross(m()) + d().Scale(p.d());
    auto w = p.normal().Dot(d());
    return HomoPoint3(vector, w);
  }

  // Project the line into a HalfSpace2 by dropping one of the dimensions.
  //
  // Say the plucker line was created from p1 to p2, then the projected versions
  // of p1 and p2 will be coincident on the projected half-space. The normal of
  // the half-space will point a quarter turn from the vector (p2-p1). That
  // means the positive half-space will be counter-clockwise from the line p1
  // to p2. Note that when dimension 1 is dropped, the vector components go
  // from (x, y, z) to (z, x). The quarter turn is performed after the
  // projection.
  //
  // At least one of the remaining components of d() must be non-zero for the
  // resulting HalfSpace2 to be valid.
  HalfSpace2 Project2D(int drop_dimension) const {
    return HalfSpace2(
        /*normal=*/d().DropDimension(drop_dimension).GetPerpendicular(),
        /*dist=*/-m_.components()[drop_dimension]);
  }

  // This could overflow. It is the caller's responsibility to ensure that none
  // of the components are equal to their min_value.
  PluckerLine operator-() const {
    return PluckerLine(-d(), -m());
  }

  // Removes the common factor.
  //
  // After this function returns, the line may be stored in a more efficient
  // format, but the value of the point will be equivalent to the value from
  // before.
  void Reduce();

 private:
  Vector3 d_;
  Vector3 m_;
};

std::ostream& operator<<(std::ostream& out, const PluckerLine& line);

}  // walnut

#endif // WALNUT_PLUCKER_LINE_H__
