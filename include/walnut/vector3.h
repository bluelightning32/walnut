#ifndef WALNUT_VECTOR3_H__
#define WALNUT_VECTOR3_H__

#include <array>

#include "walnut/big_int.h"
#include "walnut/vector2.h"

namespace walnut {

template <size_t component_bits_template = 33>
class Vector3 {
  template <size_t other_component_bits>
  friend class Vector3;

 public:
  using BigIntRep = BigInt<component_bits_template>;

  // The minimum number of bits to support for each coordinate.
  //
  // Note that the BigInt may round up the requested number of bits and end up
  // supporting more bits. Also note that BigInts are faster when fewer bits
  // are requested, and of BigInts that have the same requested bits, instances
  // with fewer used bits are faster.
  static constexpr size_t component_bits = component_bits_template;

  // Leaves the coordinates in an undefined state
  Vector3() = default;

  template <size_t other_component_bits>
  Vector3(const Vector3<other_component_bits>& other) :
    components_{other.components()[0], other.components()[1],
                other.components()[2]} { }

  Vector3(const BigIntRep& x, const BigIntRep& y, const BigIntRep& z) :
    components_{x, y, z} { }

  Vector3(long x, long y, long z) :
    components_{BigIntRep(x), BigIntRep(y), BigIntRep(z)} { }

  static Vector3 Zero() {
    return Vector3(/*x=*/0, /*y=*/0, /*z=*/0);
  }

  BigIntRep& x() {
    return components_[0];
  }

  const BigIntRep& x() const {
    return components_[0];
  }

  BigIntRep& y() {
    return components_[1];
  }

  const BigIntRep& y() const {
    return components_[1];
  }

  BigIntRep& z() {
    return components_[2];
  }

  const BigIntRep& z() const {
    return components_[2];
  }

  std::array<BigIntRep, 3>& components() {
    return components_;
  }

  const std::array<BigIntRep, 3>& components() const {
    return components_;
  }

  template <size_t other_component_bits>
  bool operator ==(const Vector3<other_component_bits>& other) const {
    return x() == other.x() &&
           y() == other.y() &&
           z() == other.z();
  }

  template <size_t other_component_bits>
  bool operator !=(const Vector3<other_component_bits>& other) const {
    return x() != other.x() ||
           y() != other.y() ||
           z() != other.z();
  }

  template <size_t other_component_bits>
  Vector3<std::max(other_component_bits, component_bits_template) + 1>
  operator-(const Vector3<other_component_bits>& other) const {
    return Vector3<std::max(other_component_bits, component_bits_template) + 1>(
        /*x=*/x() - other.x(),
        /*y=*/y() - other.y(),
        /*z=*/z() - other.z());
  }

  template <size_t other_component_bits>
  Vector3<std::max(other_component_bits, component_bits_template) + 1>
  operator+(const Vector3<other_component_bits>& other) const {
    return Vector3<std::max(other_component_bits, component_bits_template) + 1>(
        /*x=*/x() + other.x(),
        /*y=*/y() + other.y(),
        /*z=*/z() + other.z());
  }

  // Return true if the vectors have the same direction and only differ in
  // magnitude.
  //
  // Vectors with opposite magnitude are considered to have opposite
  // directions, and this function returns false for such vectors.
  template <size_t other_component_bits>
  bool IsSameDir(const Vector3<other_component_bits>& other) const;

  // Return true if the vectors have the same direction or opposite directions
  // and only differ in magnitude.
  template <size_t other_component_bits>
  bool IsSameOrOppositeDir(const Vector3<other_component_bits>& other) const;

  // Get the square of the scale of this vector
  BigInt<component_bits*2 + 2> GetScaleSquared() const {
    return Dot(*this);
  }

  // Compute the dot product
  template <size_t other_component_bits>
  BigInt<component_bits + other_component_bits + 1>
  Dot(const Vector3<other_component_bits>& other) const {
    BigInt<component_bits + other_component_bits + 1> result = x() * other.x();
    result += y() * other.y();
    result += z() * other.z();
    return result;
  }

  // Compute the cross product
  template <size_t other_component_bits>
  Vector3<component_bits + other_component_bits>
  Cross(const Vector3<other_component_bits>& other) const {
    return Vector3<component_bits + other_component_bits>(
        /*x=*/y()*other.z() - z()*other.y(),
        /*y=*/z()*other.x() - x()*other.z(),
        /*z=*/x()*other.y() - y()*other.x());
  }

  template <size_t other_bits>
  Vector3<component_bits + other_bits>
  Scale(const BigInt<other_bits>& scale) const {
    return Vector3<component_bits + other_bits>(x() * scale,
                                                y() * scale,
                                                z() * scale);
  }

  Vector3<component_bits + sizeof(int)*8> Scale(int scale) const {
    return Scale<sizeof(int)*8>(BigInt<sizeof(int)*8>(scale));
  }

  template <size_t other_bits>
  Vector3<component_bits + other_bits> operator*(
      const BigInt<other_bits>& scale) const {
    return Scale(scale);
  }

  Vector3<component_bits + sizeof(int)*8> operator*(int scale) const {
    return Scale(scale);
  }

  bool IsZero() const {
    return components_[0] == 0 && components_[1] == 0 && components_[2] == 0;
  }

  int GetFirstNonzeroDimension() const {
    for (int i = 0; i < 3; ++i) {
      if (!components()[i].IsZero()) return i;
    }
    return -1;
  }

  Vector2<component_bits> DropDimension(int drop_dimension) const {
    return Vector2<component_bits>(components()[(drop_dimension + 1) % 3],
                                   components()[(drop_dimension + 2) % 3]);
  }

  // Verifies the fields are in their supported ranges.
  //
  // The BigInts can sometimes internally support a larger range than what is
  // requested in the template parameters. This function returns true if all of
  // the fields are in their supported range.
  //
  // This function exists for testing purposes. It should always return true.
  bool IsValidState() const {
    return components_[0].IsValidState() &&
           components_[1].IsValidState() &&
           components_[2].IsValidState();
  }

  // This function could potentially overflow. The caller must ensure there is
  // sufficient bitspace.
  void Negate() {
    for (BigIntRep& coord : components_) {
      bool overflowed = coord.Negate();
      assert(!overflowed);
    }
  }

  // This could overflow. It is the caller's responsibility to ensure that none
  // of the components are equal to their min_value.
  Vector3 operator-() const {
    return Vector3(-x(), -y(), -z());
  }

 private:
  std::array<BigIntRep, 3> components_;
};

template <size_t component_bits_template>
template <size_t other_component_bits>
inline bool Vector3<component_bits_template>::IsSameDir(
    const Vector3<other_component_bits>& other) const {
  BigInt<component_bits> scale_other;
  BigInt<other_component_bits> scale_mine;
  if (x() != 0) {
    scale_other = x().abs();
    scale_mine = other.x().abs();
  } else if (y() != 0) {
    scale_other = y().abs();
    scale_mine = other.y().abs();
  } else {
    scale_other = z().abs();
    scale_mine = other.z().abs();
  }

  return x().Multiply(scale_mine) == other.x().Multiply(scale_other) &&
         y().Multiply(scale_mine) == other.y().Multiply(scale_other) &&
         z().Multiply(scale_mine) == other.z().Multiply(scale_other);
}

template <size_t component_bits_template>
template <size_t other_component_bits>
inline bool Vector3<component_bits_template>::IsSameOrOppositeDir(
    const Vector3<other_component_bits>& other) const {
  BigInt<component_bits> scale_other;
  BigInt<other_component_bits> scale_mine;
  if (x() != 0) {
    scale_other = x();
    scale_mine = other.x();
  } else if (y() != 0) {
    scale_other = y();
    scale_mine = other.y();
  } else {
    scale_other = z();
    scale_mine = other.z();
  }

  return x().Multiply(scale_mine) == other.x().Multiply(scale_other) &&
         y().Multiply(scale_mine) == other.y().Multiply(scale_other) &&
         z().Multiply(scale_mine) == other.z().Multiply(scale_other);
}

template <size_t component_bits>
std::ostream& operator<<(std::ostream& out, const Vector3<component_bits>& v) {
  return out << "{ "
             << v.components()[0] << ", "
             << v.components()[1] << ", "
             << v.components()[2]
             << " }";
}

}  // walnut

#endif // WALNUT_VECTOR3_H__
