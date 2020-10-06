#ifndef WALNUT_PLANE_H__
#define WALNUT_PLANE_H__

#include "walnut/vector3.h"
#include "walnut/vertex3.h"
#include "walnut/vertex4.h"

namespace walnut {

template <int vector_bits_template = 4 + 2*31, int dist_bits_template = 99>
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
    Plane(other.normal, other.dist) { }

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

 private:
  VectorRep normal_;
  DistInt dist_;
};

}  // walnut

#endif // WALNUT_PLANE_H__