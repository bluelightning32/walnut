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

  Vector3(const Vector3&) = default;
  Vector3(Vector3&&) = default;

  Vector3(const BigInt& x, const BigInt& y, const BigInt& z) :
    components_{x, y, z} { }

  Vector3(BigInt&& x, BigInt&& y, BigInt&& z) :
    components_{std::move(x), std::move(y), std::move(z)} { }

  Vector3(int64_t x, int64_t y, int64_t z) :
    components_{BigInt(x), BigInt(y), BigInt(z)} { }

  Vector3& operator=(const Vector3&) = default;
  Vector3& operator=(Vector3&&) = default;

  static Vector3 Zero() {
    return Vector3(/*x=*/0, /*y=*/0, /*z=*/0);
  }

  BigInt& x() {
    return components_[0];
  }

  const BigInt& x() const {
    return components_[0];
  }

  BigInt& y() {
    return components_[1];
  }

  const BigInt& y() const {
    return components_[1];
  }

  BigInt& z() {
    return components_[2];
  }

  const BigInt& z() const {
    return components_[2];
  }

  std::array<BigInt, 3>& components() {
    return components_;
  }

  const std::array<BigInt, 3>& components() const {
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

  Vector3& operator+=(const Vector3& other) {
    x() += other.x();
    y() += other.y();
    z() += other.z();
    return *this;
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
  BigInt GetScaleSquared() const {
    return Dot(*this);
  }

  // Compute the dot product
  BigInt Dot(const Vector3& other) const {
    BigInt result = x() * other.x();
    result += y() * other.y();
    result += z() * other.z();
    return result;
  }

  // Compute the cross product
  Vector3 Cross(const Vector3& other) const {
    return Vector3(
        /*x=*/std::move((y()*other.z()).SubtractMultiply(z(), other.y())),
        /*y=*/std::move((z()*other.x()).SubtractMultiply(x(), other.z())),
        /*z=*/std::move((x()*other.y()).SubtractMultiply(y(), other.x())));
  }

  // Compute the hadamard product
  Vector3 Hadamard(const Vector3& other) const {
    return Vector3(/*x=*/x() * other.x(),
                   /*y=*/y() * other.y(),
                   /*z=*/z() * other.z());
  }

  // Compute the hadamard product with itself
  Vector3 HadamardSquared() const {
    return Hadamard(*this);
  }

  Vector3 Scale(const BigInt& scale) const {
    return Vector3(x() * scale, y() * scale, z() * scale);
  }

  Vector3 Scale(int scale) const {
    return Scale(BigInt(scale));
  }

  Vector3 operator*(const BigInt& scale) const {
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

  const BigInt& GetComponentAfterDrop(int component,
                                      int drop_dimension) const {
    return components()[(component + drop_dimension + 1) % 3];
  }

  BigInt& GetComponentAfterDrop(int component, int drop_dimension) {
    return components()[(component + drop_dimension + 1) % 3];
  }

  // This function could potentially overflow. The caller must ensure there is
  // sufficient bitspace.
  void Negate() {
    for (BigInt& coord : components_) {
      coord.Negate();
    }
  }

  // This could overflow. It is the caller's responsibility to ensure that none
  // of the components are equal to their min_value.
  Vector3 operator-() const {
    return Vector3(-x(), -y(), -z());
  }

  // Reduce the vector to the lowest vector that points in the same direction.
  //
  // The resulting vector will point in the same direction, but have a lower
  // magnitude.
  void Reduce();

 private:
  std::array<BigInt, 3> components_;
};

inline bool Vector3::IsSameDir(const Vector3& other) const {
  BigInt scale_other;
  BigInt scale_mine;
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
  BigInt scale_other;
  BigInt scale_mine;
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

std::ostream& operator<<(std::ostream& out, const Vector3& v);

inline Vector3 operator*(const BigInt& scale, const Vector3& v) {
  return v.Scale(scale);
}

}  // walnut

#endif // WALNUT_VECTOR3_H__
