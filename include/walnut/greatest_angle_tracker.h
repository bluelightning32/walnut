#ifndef WALNUT_GREATEST_ANGLE_TRACKER_H__
#define WALNUT_GREATEST_ANGLE_TRACKER_H__

#include "walnut/r_transformation.h"
#include "walnut/vector3.h"

namespace walnut {

// Receives Vector3s and remembers the one that points the most
// counter-clockwise or clockwise (depending on the template parameter)
// relative to the first received Vector3.
//
// The vector comparisons are done using `RXYCompareBivector`.
//
// The behavior is undefined if the total range of received vectors is a
// half-turn or more. For example, receiving a vector to the right then one to
// the left is undefined, because they are a half-turn apart. Receiving a
// vector to the right, one up, then one to the left is also undefined.
template <bool most_ccw = true, int component_bits_template = 33>
class GreatestAngleTracker {
 public:
  using Vector3Rep = Vector3<component_bits_template>;

  static constexpr int component_bits = component_bits_template;

  void Receive(const Vector3Rep& v);

  const Vector3Rep& current() const {
    return current_;
  }

  bool AnyReceived() const {
    return !current_.IsZero();
  }

 private:
  Vector3Rep current_;
};

template <bool most_ccw, int component_bits>
void GreatestAngleTracker<most_ccw, component_bits>::Receive(
    const Vector3Rep& v) {
  int comparison = RXYCompareBivector(current_, v);
  if (most_ccw) {
    if (comparison >= 0) {
      current_ = v;
    }
  } else {
    if (comparison <= 0) {
      current_ = v;
    }
  }
}

}  // walnut

#endif // WALNUT_GREATEST_ANGLE_TRACKER_H__
