#ifndef WALNUT_PLANE_H__
#define WALNUT_PLANE_H__

#include "walnut/vertex.h"

namespace walnut {

template <int vector_bits_template = 4 + 2*31, int dist_bits_template = 131>
class Plane {
 public:
  using Vector3Rep = Vector3<vector_bits_template>;
  using VectorInt = typename Vector3Rep::BigIntRep;
  using DistInt = BigInt<dist_bits_template>;

  // The minimum number of bits to support for each of the x, y, and z coordinates.
  static constexpr int vector_bits = vector_bits_template;
  // The minimum number of bits to support for the d coordinate.
  static constexpr int dist_bits = dist_bits_template;

  VectorInt& x() {
    return normal_.x();
  }

  VectorInt& y() {
    return normal_.y();
  }

  VectorInt& z() {
    return normal_.z();
  }

  DistInt& d() {
    return dist_;
  }

  // Leaves the coordinates in an undefined state
  Plane() = default;

  template <int other_vector_bits>
  Plane(const Vector3<other_vector_bits>& normal, const DistInt& dist) :
    normal_(normal), dist_(dist) { }

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
    normal_((p3 - p2).Cross(p1 - p2)), dist_(normal_.Dot(p2)) { }

  // Returns >0 if `v` is in the half space, 0 if `v` is coincident with the
  // plane, or <0 if `v` is outside of the half space.
  template <int v_bits>
  int Compare(const Vector3<v_bits>& v);

 private:
  Vector3Rep normal_;
  DistInt dist_;
};

template <int vector_bits_template, int dist_bits_template>
template <int v_bits>
inline int Plane<vector_bits_template, dist_bits_template>::Compare(const Vector3<v_bits>& v) {
  return dist_.Compare(normal_.Dot(v));
  return 0;
}

}  // walnut

#endif // WALNUT_PLANE_H__
