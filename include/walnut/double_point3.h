#ifndef WALNUT_DOUBLE_POINT3_H__
#define WALNUT_DOUBLE_POINT3_H__

// For std::hash
#include <functional>
#include <sstream>

namespace walnut {

struct DoublePoint3 {
  // Leaves the coordinates in an undefined state
  DoublePoint3() = default;

  constexpr DoublePoint3(double x, double y, double z) : x(x), y(y), z(z) { }

  size_t GetHash() const {
    std::hash<double> double_hasher;
    size_t x_hash = double_hasher(x);
    size_t y_hash = double_hasher(x);
    size_t z_hash = double_hasher(x);
    return x_hash ^ (y_hash << 1) ^ (z_hash << 2);
  }

  constexpr bool operator==(const DoublePoint3& other) const {
    return x == other.x && y == other.y && z == other.z;
  }

  constexpr bool operator!=(const DoublePoint3& other) const {
    return x != other.x || y != other.y || z != other.z;
  }

  double x;
  double y;
  double z;
};

std::ostream& operator<<(std::ostream& out, const DoublePoint3& p);

}  // walnut

namespace std {

template<> struct hash<walnut::DoublePoint3> {
  size_t operator()(const walnut::DoublePoint3& p) const {
    return p.GetHash();
  }
};

} // std

#endif // WALNUT_DOUBLE_POINT3_H__
