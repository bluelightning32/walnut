#include "walnut/nudging_plane_builder.h"

namespace walnut {

HalfSpace3 NudgingPlaneBuilder::BuildFromPlaneInfo() const {
  std::array<const HomoPoint3*, 3> filtered_vertices;
  // The number of remaining unconstrained vertices to accept.
  size_t accept_unconstrained = 3 - constrained_count_;

  for (size_t i = 0, accepted = 0; accepted < 3; ++i) {
    const VertexInfo& info = plane_info_[i];
    bool accept;
    if (info.constrained) {
      accept = true;
    } else if (accept_unconstrained > 0) {
      accept = true;
      --accept_unconstrained;
    } else {
      accept = false;
    }
    if (accept) {
      filtered_vertices[accepted] = info.vertex;
      ++accepted;
    }
  }
  HalfSpace3 result(*filtered_vertices[0], *filtered_vertices[1],
                    *filtered_vertices[2]);
  result.Reduce();
  return result;
}

}  // walnut
