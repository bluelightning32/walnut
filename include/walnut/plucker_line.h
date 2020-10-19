#ifndef WALNUT_PLUCKER_LINE_H__
#define WALNUT_PLUCKER_LINE_H__

#include "walnut/half_space3.h"
#include "walnut/point3.h"
#include "walnut/vector3.h"

namespace walnut {

// Plücker coordinates are a way to store a R^3 line with homogeneous coordinates.
template <int d_bits_template = 31*2 + 1, int m_bits_template = 31*2 + 3>
class PluckerLine {
 public:
  using DVector = Vector3<d_bits_template>;
  using MVector = Vector3<m_bits_template>;

  // The minimum number of bits to support for each of the components of d.
  static constexpr int d_bits = d_bits_template;
  // The minimum number of bits to support for each of the components of m.
  static constexpr int m_bits = m_bits_template;

  // Returns the direction vector for the line.
  //
  // If the line was constructed using points, the direction vector points from
  // p1 to p2.
  //
  // Some definitions use the p^ab or p_ab notation.
  // d = (p_01, p_02, p_03)
  // d = (p^23, p^31, p^12)
  const DVector& d() const {
    return d_;
  }

  DVector& d() {
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
  const MVector& m() const {
    return m_;
  }

  MVector& m() {
    return m_;
  }

  // Returns true if this line was correctly constructed.
  bool IsValid() const {
    // m_ may be 0 if the line goes through the origin.
    return !d_.IsZero();
  }

  // Leaves the vectors in an undefined state
  PluckerLine() = default;

  template <int other_d_bits, int other_m_bits>
  PluckerLine(const Vector3<other_d_bits>& d, const Vector3<other_m_bits>& m) :
    d_(d), m_(m) { }

  template <int other_d_bits, int other_m_bits>
  PluckerLine(const PluckerLine<other_d_bits, other_m_bits>& other) :
    PluckerLine(other.d(), other.m()) { }

  template <int point_bits>
  PluckerLine(const Point3<point_bits>& p1,
              const Point3<point_bits>& p2) :
    d_(p2 - p1), m_(p1.vector_from_origin().Cross(p2.vector_from_origin())) { }

  // Constructs the line for the intersection of the two planes, `a` and `b`.
  //
  // Both `a` and `b` must be valid (have non-zero normals), and they must be
  // non-equal.
  template <int vector_bits, int dist_bits>
  PluckerLine(const HalfSpace3<vector_bits, dist_bits>& a,
              const HalfSpace3<vector_bits, dist_bits>& b) :
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
    // m = (p^01, p^02, p^03)
    //
    // p^01 = | a.d  a.x |
    //        | b.d  b.x |
    //
    // p^02 = | a.d  a.y |
    //        | b.d  b.y |
    //
    // p^03 = | a.d  a.z |
    //        | b.d  b.z |
    d_(a.normal().Cross(b.normal())),
    m_(MVector::BigIntRep::Determinant(-a.d(), a.x(), -b.d(), b.x()),
       MVector::BigIntRep::Determinant(-a.d(), a.y(), -b.d(), b.y()),
       MVector::BigIntRep::Determinant(-a.d(), a.z(), -b.d(), b.z())) { }

  // Returns true if `p` is on the line.
  template <int v_bits>
  bool IsOnLine(const Point3<v_bits>& p) const {
    /*
     * p x (p + d) == m
     * p x p + p x d == m
     * p x d == m
     */
    return p.vector_from_origin().Cross(d_) == m();
  }

  // Returns true if `p` is on the line.
  template <int num_bits, int denom_bits>
  bool IsOnLine(const HomoPoint3<num_bits, denom_bits>& p) const {
    /*
     * p x (p + p.dist*d) == m
     * p x p + p x (p.dist*d) == m
     * p x (p.dist*d) == m
     */
    return p.vector_from_origin().Cross(d_ * p.dist_denom()) == m();
  }

  // Returns true when the lines match
  //
  // Two plucker lines are still considered equal if they describe the same set
  // of lines. Notably the lines may have different common scale factors.
  template <int other_d_bits, int other_m_bits>
  bool operator==(const PluckerLine<other_d_bits, other_m_bits>& other) const {
    using OtherDVector =
      typename PluckerLine<other_d_bits, other_m_bits>::DVector;
    typename DVector::BigIntRep scale_other;
    typename OtherDVector::BigIntRep scale_mine;
    if (d().x() != 0) {
      scale_other = d().x();
      scale_mine = other.d().x();
    } else if (d().y() != 0) {
      scale_other = d().y();
      scale_mine = other.d().y();
    } else {
      scale_other = d().z();
      scale_mine = other.d().z();
    }

    return
      d().x().Multiply(scale_mine) == other.d().x().Multiply(scale_other) &&
      d().y().Multiply(scale_mine) == other.d().y().Multiply(scale_other) &&
      d().z().Multiply(scale_mine) == other.d().z().Multiply(scale_other) &&
      m().x().Multiply(scale_mine) == other.m().x().Multiply(scale_other) &&
      m().y().Multiply(scale_mine) == other.m().y().Multiply(scale_other) &&
      m().z().Multiply(scale_mine) == other.m().z().Multiply(scale_other);
  }

  template <int other_d_bits, int other_m_bits>
  bool operator!=(const PluckerLine<other_d_bits, other_m_bits>& other) const {
    return !(*this == other);
  }

  // Verifies the fields are in their supported ranges.
  //
  // The BigInts can sometimes internally support a larger range than what is
  // requested in the template parameters. This function returns true if all of
  // the fields are in their supported range.
  //
  // This function exists for testing purposes. It should always return true.
  bool IsValidState() const {
    return d().IsValidState() && m().IsValidState();
  }

  // Calculate the intersection between this line and a plane.
  template <int vector_bits, int dist_bits>
  HomoPoint3<std::max(vector_bits + m_bits, d_bits + dist_bits) + 1,
          vector_bits + d_bits + 1>
  Intersect(const HalfSpace3<vector_bits, dist_bits>& p) const {
    auto vector = p.normal().Cross(m()) + d().Scale(p.d());
    auto w = p.normal().Dot(d());
    return HomoPoint3<decltype(vector)::coord_bits, decltype(w)::bits>(
                       /*p=*/Point3<decltype(vector)::coord_bits>(vector),
                       w);
  }

 private:
  DVector d_;
  MVector m_;
};

// This is a wrapper around the PluckerLine constructor that takes 2 Point3's.
// The only reason to use this wrapper is that it figures out how many bits are
// necessary in the worst case for the PluckerLine d and m vector components,
// given the number of bits in each Point3.
template <int point3_bits_template = 32>
class PluckerLineFromPoint3sBuilder {
 public:
  using Point3Rep = Point3<point3_bits_template>;
  using PluckerLineRep = PluckerLine<point3_bits_template + 1,
                                     (point3_bits_template - 1)*2 + 2>;
  using DInt = typename PluckerLineRep::DVector::BigIntRep;
  using MInt = typename PluckerLineRep::MVector::BigIntRep;

  static constexpr DInt d_component_min() {
    DInt n = Point3Rep::BigIntRep::max_value() + DInt(1);
    DInt two_n_1 = n + n - BigInt<2>(1);
    return -two_n_1;
  }
  static constexpr DInt d_component_max() {
    return -d_component_min();
  }
  static constexpr MInt m_component_min() {
    MInt n = Point3Rep::BigIntRep::max_value() + MInt(1);
    return d_component_min() * n;
  }
  static constexpr MInt m_component_max() {
    return -m_component_min();
  }

  static PluckerLineRep Build(const Point3Rep& p1, const Point3Rep& p2) {
    return PluckerLineRep(p1, p2);
  }
};

// This is a wrapper around the PluckerLine constructor that takes 2 Planes.
// The only reason to use this wrapper is that it figures out how many bits are
// necessary in the worst case for the PluckerLine d and m vector components,
// given that the Plane components are all within the bounds defined by
// HalfSpace3FromPoint3Builder<point3_bits>.
template <int point3_bits_template = 32>
class PluckerLineFromPlanesFromPoint3sBuilder {
 public:
  static_assert(point3_bits_template >= 3,
      "The bit formulas are only correct for point3_bits_template >= 3");
  using Point3Rep = Point3<point3_bits_template>;
  using HalfSpace3Builder = HalfSpace3FromPoint3Builder<point3_bits_template>;
  using HalfSpace3Rep = typename HalfSpace3Builder::HalfSpace3Rep;
  using PluckerLineRep = PluckerLine<(point3_bits_template - 1)*4 + 6,
                                     (point3_bits_template - 1)*5 + 6>;
  using DInt = typename PluckerLineRep::DVector::BigIntRep;
  using MInt = typename PluckerLineRep::MVector::BigIntRep;

  static constexpr DInt d_component_min() {
    DInt n = Point3Rep::BigIntRep::max_value() + DInt(1);
    DInt two_n_1 = n + n - BigInt<2>(1);
    return -two_n_1*two_n_1*two_n_1*two_n_1*DInt(2);
  }
  static constexpr DInt d_component_max() {
    return -d_component_min();
  }
  static constexpr MInt m_component_min() {
    MInt n = Point3Rep::BigIntRep::max_value() + MInt(1);
    return d_component_min() * (n + MInt(1));
  }
  static constexpr MInt m_component_max() {
    return -m_component_min();
  }

  static PluckerLineRep Build(const HalfSpace3Rep& p1, const HalfSpace3Rep& p2) {
    return PluckerLineRep(p1, p2);
  }
};

}  // walnut

#endif // WALNUT_PLANE_H__
