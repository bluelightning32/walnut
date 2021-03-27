#ifndef WALNUT_HOMO_POINT2_H__
#define WALNUT_HOMO_POINT2_H__

#include "walnut/point2.h"
#include "walnut/vector2.h"

namespace walnut {

// 2D point represented with homogeneous coordinates. The w coordinate acts
// like a divisor for the x and y coordinates.
class HomoPoint2 {
 public:
  BigIntImpl& x() {
    return vector_from_origin_.x();
  }

  const BigIntImpl& x() const {
    return vector_from_origin_.x();
  }

  BigIntImpl& y() {
    return vector_from_origin_.y();
  }

  const BigIntImpl& y() const {
    return vector_from_origin_.y();
  }

  BigIntImpl& w() {
    return dist_denom_;
  }

  const BigIntImpl& w() const {
    return dist_denom_;
  }

  Vector2& vector_from_origin() {
    return vector_from_origin_;
  }

  const Vector2& vector_from_origin() const {
    return vector_from_origin_;
  }

  BigIntImpl& dist_denom() {
    return dist_denom_;
  }

  const BigIntImpl& dist_denom() const {
    return dist_denom_;
  }

  // Leaves the coordinates in an undefined state
  HomoPoint2() = default;

  HomoPoint2(const HomoPoint2& other) :
    vector_from_origin_(other.vector_from_origin_), dist_denom_(other.dist_denom_) { }

  HomoPoint2(const BigIntImpl& x, const BigIntImpl& y, const BigIntImpl& w) :
    vector_from_origin_(x, y), dist_denom_(w) { }

  HomoPoint2(int x, int y, int w) :
    vector_from_origin_(x, y), dist_denom_(w) { }

  HomoPoint2(const Point2& other) :
    vector_from_origin_(other.vector_from_origin()), dist_denom_(1) { }

  HomoPoint2(const Vector2& v, const BigIntImpl& w) :
    vector_from_origin_(v), dist_denom_(w) { }

  static bool LexicographicallyLt(const HomoPoint2& a, const HomoPoint2& b) {
    auto a_scaled = a.vector_from_origin() * b.dist_denom();
    auto b_scaled = b.vector_from_origin() * a.dist_denom();
    return std::lexicographical_compare(a_scaled.coords().begin(),
                                        a_scaled.coords().end(),
                                        b_scaled.coords().begin(),
                                        b_scaled.coords().end());
  }

  // Note that everything equals the 0 point with a 0 denominator.
  bool operator==(const HomoPoint2& other) const {
    return vector_from_origin().Scale(other.w()) ==
      other.vector_from_origin().Scale(w());
  }

  // Note that everything equals the 0 point with a 0 denominator.
  bool operator==(const Point2& other) const {
    return vector_from_origin() == other.vector_from_origin().Scale(w());
  }

  // Note that everything equals the 0 point with a 0 denominator.
  bool operator!=(const HomoPoint2& other) const {
    return !(*this == other);
  }

  // Note that everything equals the 0 point with a 0 denominator.
  bool operator!=(const Point2& other) const {
    return !(*this == other);
  }

 private:
  Vector2 vector_from_origin_;
  BigIntImpl dist_denom_;
};

inline bool operator==(const Point2& a, const HomoPoint2& b) {
  return b == a;
}

inline std::ostream& operator<<(std::ostream& out, const HomoPoint2& p) {
  return out << "{ ["
             << p.x() << ", "
             << p.y() << "] / "
             << p.w()
             << " }";
}

}  // walnut

#endif // WALNUT_HOMO_POINT2_H__
