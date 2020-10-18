#ifndef WALNUT_PLANE_H__
#define WALNUT_PLANE_H__

#include "walnut/vector3.h"
#include "walnut/vertex3.h"
#include "walnut/vertex4.h"

namespace walnut {

template <int vector_bits_template = 31*2 + 3, int dist_bits_template = 31*3 + 3>
class Plane {
 public:
  using VectorRep = Vector3<vector_bits_template>;
  using VectorInt = typename VectorRep::BigIntRep;
  using DistInt = BigInt<dist_bits_template>;

  // The minimum number of bits to support for each of the x, y, and z coordinates.
  static constexpr int vector_bits = vector_bits_template;
  // The minimum number of bits to support for the d coordinate.
  static constexpr int dist_bits = dist_bits_template;

  const VectorInt& x() const {
    return normal_.x();
  }

  const VectorInt& y() const {
    return normal_.y();
  }

  const VectorInt& z() const {
    return normal_.z();
  }

  const VectorRep& normal() const {
    return normal_;
  }

  DistInt& d() {
    return dist_;
  }

  const DistInt& d() const {
    return dist_;
  }

  bool IsValid() const {
    return !normal_.IsZero();
  }

  // Leaves the coordinates in an undefined state
  Plane() = default;

  template <int other_vector_bits>
  Plane(const Vector3<other_vector_bits>& normal, const DistInt& dist) :
    normal_(normal), dist_(dist) { }

  Plane(const VectorInt& x, const VectorInt& y, const VectorInt& z,
        const DistInt& dist) :
    normal_(x, y, z), dist_(dist) { }

  Plane(int x, int y, int z, int dist) : normal_(x, y, z), dist_(dist) { }

  template <int other_vector_bits, int other_dist_bits>
  Plane(const Plane<other_vector_bits, other_dist_bits>& other) :
    Plane(other.normal(), other.d()) { }

  template <int vertex_bits>
  Plane(const Vertex3<vertex_bits>& p1,
        const Vertex3<vertex_bits>& p2,
        const Vertex3<vertex_bits>& p3) :
    // Use p2 as the center point, because if p1, p2, and p3 are from a polygon
    // with more than 3 points, (p3 - p2) and (p1 - p2) are likely to be
    // shorter than (p2 - p1) and (p3 - p1).
    normal_((p3 - p2).Cross(p1 - p2)),
    dist_(normal_.Dot(p2.vector_from_origin())) { }

  // Returns >0 if `v` is in the half space, 0 if `v` is coincident with the
  // plane, or <0 if `v` is outside of the half space.
  template <int v_bits>
  int Compare(const Vertex3<v_bits>& v) {
    return dist_.Compare(normal_.Dot(v.vector_from_origin()));
  }

  // Returns >0 if `v` is in the half space, 0 if `v` is coincident with the
  // plane, or <0 if `v` is outside of the half space.
  template <int v_num_bits, int v_denom_bits>
  int Compare(const Vertex4<v_num_bits, v_denom_bits>& v) {
    return (v.dist_denom() * dist_).Compare(normal_.Dot(v.vector_from_origin()));
  }

  // Returns a plane with an invalid 0 normal vector and a 0 distance.
  //
  // All vertices are coincident with the returned plane. `IsValid` will report
  // false for the returned plane.
  static Plane Zero() {
    return Plane(/*normal=*/VectorRep::Zero(), /*dist=*/DistInt(0));
  }

  // Note that everything equals the zero vector.
  template <int other_vector_bits, int other_dist_bits>
  bool operator==(
      const Plane<other_vector_bits, other_dist_bits>& other) const {
    return normal().Scale(other.d()) == other.normal().Scale(d());
  }

  // Note that everything equals the zero vector.
  template <int other_vector_bits, int other_dist_bits>
  bool operator!=(
      const Plane<other_vector_bits, other_dist_bits>& other) const {
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
    return normal_.IsValidState() && dist_.IsValidState();
  }

 private:
  VectorRep normal_;
  DistInt dist_;
};

// This is a wrapper around the Plane constructor that takes 3 Vertex3's. The
// only reason to use this wrapper is that it figures out how many bits are
// necessary in the worst case for the plane numerator and denominator, given
// the number of bits in each Vertex3.
template <int vertex3_bits_template = 32>
class PlaneFromVertex3Builder {
 public:
  using Vertex3Rep = Vertex3<vertex3_bits_template>;
  using PlaneRep = Plane<(vertex3_bits_template - 1)*2 + 3,
                         (vertex3_bits_template - 1)*3 + 3>;
  using VectorInt = typename PlaneRep::VectorInt;
  using DistInt = typename PlaneRep::DistInt;

  static constexpr VectorInt normal_component_min() {
    VectorInt n = Vertex3Rep::BigIntRep::max_value() + VectorInt(1);
    VectorInt two_n_1 = n + n - BigInt<2>(1);
    return -two_n_1 * two_n_1;
  }
  static constexpr VectorInt normal_component_max() {
    return -normal_component_min();
  }
  static constexpr DistInt dist_min() {
    DistInt n = Vertex3Rep::BigIntRep::max_value() + DistInt(1);
    return normal_component_min() * (n + DistInt(1));
  }
  static constexpr DistInt dist_max() {
    return -dist_min();
  }

  static PlaneRep Build(const Vertex3Rep& p1,
                        const Vertex3Rep& p2,
                        const Vertex3Rep& p3) {
    return PlaneRep(p1, p2, p3);
  }
};

template <int vector_bits, int dist_bits>
std::ostream& operator<<(std::ostream& out,
                         const Plane<vector_bits, dist_bits>& p) {
  return out << "{ x*" << p.x()
             << " + y*" << p.y()
             << " + z*" << p.z()
             << " = " << p.d()
             << " }";
}

}  // walnut

#endif // WALNUT_PLANE_H__
