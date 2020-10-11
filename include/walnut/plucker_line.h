#ifndef WALNUT_PLUCKER_LINE_H__
#define WALNUT_PLUCKER_LINE_H__

#include "walnut/vector3.h"
#include "walnut/vertex3.h"

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

  template <int vertex_bits>
  PluckerLine(const Vertex3<vertex_bits>& p1,
              const Vertex3<vertex_bits>& p2) :
    d_(p2 - p1), m_(p1.vector_from_origin().Cross(p2.vector_from_origin())) { }

  // Returns true if `v` is on the line.
  template <int v_bits>
  bool IsOnLine(const Vertex3<v_bits>& v) const {
    return v.vector_from_origin().Cross((v + d_).vector_from_origin()) == m();
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

 private:
  DVector d_;
  MVector m_;
};

// This is a wrapper around the PluckerLine constructor that takes 2 Vertex3's.
// The only reason to use this wrapper is that it figures out how many bits are
// necessary in the worst case for the PluckerLine d and m vector components,
// given the number of bits in each Vertex3.
template <int vertex3_bits_template = 32>
class PluckerLineFromVertex3Builder {
 public:
  using Vertex3Rep = Vertex3<vertex3_bits_template>;
  using PluckerLineRep = PluckerLine<vertex3_bits_template + 1,
                                     (vertex3_bits_template - 1)*2 + 2>;
  using DInt = typename PluckerLineRep::DVector::BigIntRep;
  using MInt = typename PluckerLineRep::MVector::BigIntRep;

  static constexpr DInt d_component_min() {
    DInt n = Vertex3Rep::BigIntRep::max_value() + DInt(1);
    DInt two_n_1 = n + n - BigInt<2>(1);
    return -two_n_1;
  }
  static constexpr DInt d_component_max() {
    return -d_component_min();
  }
  static constexpr MInt m_component_min() {
    MInt n = Vertex3Rep::BigIntRep::max_value() + MInt(1);
    return d_component_min() * n;
  }
  static constexpr MInt m_component_max() {
    return -m_component_min();
  }

  static PluckerLineRep Build(const Vertex3Rep& p1, const Vertex3Rep& p2) {
    return PluckerLineRep(p1, p2);
  }
};

}  // walnut

#endif // WALNUT_PLANE_H__
