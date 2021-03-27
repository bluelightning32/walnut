#ifndef WALNUT_VECTOR3_H__
#define WALNUT_VECTOR3_H__

#include <array>

#include "walnut/big_int.h"
#include "walnut/vector2.h"

namespace walnut {

class Vector3 {
 public:
  // Leaves the coordinates in an undefined state
  Vector3() = default;

  Vector3(const Vector3& other) :
    components_{other.components()[0], other.components()[1],
                other.components()[2]} { }

  Vector3(const BigIntImpl& x, const BigIntImpl& y, const BigIntImpl& z) :
    components_{x, y, z} { }

  Vector3(long x, long y, long z) :
    components_{BigIntImpl(x), BigIntImpl(y), BigIntImpl(z)} { }

  static Vector3 Zero() {
    return Vector3(/*x=*/0, /*y=*/0, /*z=*/0);
  }

  BigIntImpl& x() {
    return components_[0];
  }

  const BigIntImpl& x() const {
    return components_[0];
  }

  BigIntImpl& y() {
    return components_[1];
  }

  const BigIntImpl& y() const {
    return components_[1];
  }

  BigIntImpl& z() {
    return components_[2];
  }

  const BigIntImpl& z() const {
    return components_[2];
  }

  std::array<BigIntImpl, 3>& components() {
    return components_;
  }

  const std::array<BigIntImpl, 3>& components() const {
    return components_;
  }

  bool operator ==(const Vector3& other) const {
    return x() == other.x() &&
           y() == other.y() &&
           z() == other.z();
  }

  bool operator !=(const Vector3& other) const {
    return x() != other.x() ||
           y() != other.y() ||
           z() != other.z();
  }

  Vector3 operator-(const Vector3& other) const {
    return Vector3(/*x=*/x() - other.x(), /*y=*/y() - other.y(),
                   /*z=*/z() - other.z());
  }

  Vector3 operator+(const Vector3& other) const {
    return Vector3(/*x=*/x() + other.x(), /*y=*/y() + other.y(),
                   /*z=*/z() + other.z());
  }

  // Return true if the vectors have the same direction and only differ in
  // magnitude.
  //
  // Vectors with opposite magnitude are considered to have opposite
  // directions, and this function returns false for such vectors.
  bool IsSameDir(const Vector3& other) const;

  // Return true if the vectors have the same direction or opposite directions
  // and only differ in magnitude.
  bool IsSameOrOppositeDir(const Vector3& other) const;

  // Get the square of the scale of this vector
  BigIntImpl GetScaleSquared() const {
    return Dot(*this);
  }

  // Compute the dot product
  BigIntImpl Dot(const Vector3& other) const {
    BigIntImpl result = x() * other.x();
    result += y() * other.y();
    result += z() * other.z();
    return result;
  }

  // Compute the cross product
  Vector3 Cross(const Vector3& other) const {
    return Vector3(/*x=*/y()*other.z() - z()*other.y(),
                   /*y=*/z()*other.x() - x()*other.z(),
                   /*z=*/x()*other.y() - y()*other.x());
  }

  Vector3 Scale(const BigIntImpl& scale) const {
    return Vector3(x() * scale, y() * scale, z() * scale);
  }

  Vector3 Scale(int scale) const {
    return Scale(BigIntImpl(scale));
  }

  Vector3 operator*(const BigIntImpl& scale) const {
    return Scale(scale);
  }

  Vector3 operator*(int scale) const {
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

  Vector2 DropDimension(int drop_dimension) const {
    return Vector2(components()[(drop_dimension + 1) % 3],
                   components()[(drop_dimension + 2) % 3]);
  }

  // This function could potentially overflow. The caller must ensure there is
  // sufficient bitspace.
  void Negate() {
    for (BigIntImpl& coord : components_) {
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
  std::array<BigIntImpl, 3> components_;
};

inline bool Vector3::IsSameDir(const Vector3& other) const {
  BigIntImpl scale_other;
  BigIntImpl scale_mine;
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

inline bool Vector3::IsSameOrOppositeDir(const Vector3& other) const {
  BigIntImpl scale_other;
  BigIntImpl scale_mine;
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

inline std::ostream& operator<<(std::ostream& out, const Vector3& v) {
  return out << "{ "
             << v.components()[0] << ", "
             << v.components()[1] << ", "
             << v.components()[2]
             << " }";
}

}  // walnut

#endif // WALNUT_VECTOR3_H__
