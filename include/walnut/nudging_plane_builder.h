#ifndef WALNUT_NUDGING_PLANE_BUILDER_H__
#define WALNUT_NUDGING_PLANE_BUILDER_H__

#include <array>

#include "walnut/half_space3.h"
#include "walnut/homo_point3.h"

namespace walnut {

// Builds a `HalfSpace3` from a combination of vertices that can be adjusted so
// that they are on a common plane, and from vertices that cannot be adjusted.
class NudgingPlaneBuilder {
 public:
  // Adds an unconstrained vertex (one that can be nudged) to the plane.
  void AddUnconstrained(const HomoPoint3* vertex);

  // Tries to add a constrained vertex to the plane.
  //
  // Returns false if the new vertex does not match the current plane, and the
  // plane is already fully constrained.
  bool TryAddConstrained(const HomoPoint3* vertex);

  // Use the accepted vertices to build a plane.
  //
  // This may only be called after 3 or more calls to `AddUnconstrained` and
  // `TryAddConstrained`.
  HalfSpace3 Build() const& {
    if (plane_set_) {
      return plane_;
    } else {
      return BuildFromPlaneInfo();
    }
  }

  // Use the accepted vertices to build a plane.
  //
  // This may only be called after 3 or more calls to `AddUnconstrained` and
  // `TryAddConstrained`.
  HalfSpace3 Build() && {
    if (plane_set_) {
      return std::move(plane_);
    } else {
      return BuildFromPlaneInfo();
    }
  }

  bool HasLargePlane();

 private:
  // This may only be called if `plane_info_used_` >= 3.
  HalfSpace3 BuildFromPlaneInfo() const;

  struct VertexInfo {
    bool constrained;
    const HomoPoint3* vertex;
  };
  std::array<VertexInfo, 6> plane_info_;
  size_t plane_info_used_ = 0;
  size_t constrained_count_ = 0;
  // This is only initialized if constrained_count_ == 3.
  HalfSpace3 plane_;
  bool plane_set_ = false;
};

inline void NudgingPlaneBuilder::AddUnconstrained(const HomoPoint3* vertex) {
  if (plane_info_used_ < 3) {
    plane_info_[plane_info_used_].constrained = false;
    plane_info_[plane_info_used_].vertex = vertex;
    ++plane_info_used_;
  }
}

inline bool NudgingPlaneBuilder::TryAddConstrained(const HomoPoint3* vertex) {
  if (constrained_count_ < 3) {
    plane_info_[plane_info_used_].constrained = true;
    plane_info_[plane_info_used_].vertex = vertex;
    ++plane_info_used_;
    ++constrained_count_;

    if (constrained_count_ == 3) {
      plane_ = BuildFromPlaneInfo();
      plane_set_ = true;
    }
    return true;
  } else {
    return plane_.IsCoincident(*vertex);
  }
}

inline bool NudgingPlaneBuilder::HasLargePlane() {
  if (plane_info_used_ < 3) return false;

  if (!plane_set_) {
    plane_ = BuildFromPlaneInfo();
    plane_set_ = true;
  }
  return !BigInt::AllHalfWords(plane_.x(), plane_.y(), plane_.z()) ||
         !plane_.d().IsHalfWord();
}

}  // walnut

#endif // WALNUT_NUDGING_PLANE_BUILDER_H__
