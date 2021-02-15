#ifndef WALNUT_CONVEX_POLYGON_SPLIT_INFO_H__
#define WALNUT_CONVEX_POLYGON_SPLIT_INFO_H__

#include <sstream>
// for std::pair
#include <utility>

#include "walnut/homo_point3.h"
#include "walnut/plucker_line.h"

namespace walnut {

// Stores information about which vertex indexes should be included in the
// child ConvexPolygons created from a split.
struct ConvexPolygonSplitRanges {
  bool IsValid(size_t vertex_count) const {
    if (!(neg_range.first <= neg_range.second)) return false;

    if (!(neg_range.second <= neg_range.first + vertex_count)) return false;

    if (!(pos_range.first <= pos_range.second)) return false;

    if (!(pos_range.second <= pos_range.first + vertex_count)) return false;

    if (ShouldEmitNegativeChild() && ShouldEmitPositiveChild()) {
      // pos_range must start either exactly at the end of neg_range, or one
      // after the end of neg_range.
      if (neg_range.second % vertex_count != pos_range.first % vertex_count &&
          (neg_range.second + 1)% vertex_count !=
          pos_range.first % vertex_count) {
        return false;
      }

      // neg_range must start either exactly at the end of pos_range, or one
      // after the end of pos_range.
      if (pos_range.second % vertex_count != neg_range.first % vertex_count &&
          (pos_range.second + 1)% vertex_count !=
          neg_range.first % vertex_count) {
        return false;
      }
    }

    return true;
  }

  bool ShouldEmitNegativeChild() const {
    return neg_range.first != neg_range.second;
  }

  bool ShouldEmitPositiveChild() const {
    return pos_range.first != pos_range.second;
  }

  // [neg_range.first, neg_range.second) should be part of only the negative
  // child.
  //
  // This is the index of the first vertex that should be included in the
  // negative child. If neg_range.first == neg_range.second, then no negative
  // child should be emitted.
  std::pair<size_t, size_t> neg_range{0, 0};

  // [pos_range.first, pos_range.second) should be part of only the positive
  // child.
  //
  // This is the index of the first vertex that should be included in the
  // positive child. If pos_range.first == pos_range.second, then no positive
  // child should be emitted.
  std::pair<size_t, size_t> pos_range{0, 0};
};

// Stores information about how to build the ConvexPolygons on both sides of a
// plane.
template <size_t point3_bits_template = 32>
struct ConvexPolygonSplitInfo {
  using HomoPoint3Rep = HomoPoint3<(point3_bits_template - 1)*7 + 10,
                             (point3_bits_template - 1)*6 + 10>;
  using LineRep = typename PluckerLineFromPlanesFromPoint3sBuilder<
    point3_bits_template>::PluckerLineRep;

  bool IsValid(size_t vertex_count) const {
    if (!ranges.IsValid(vertex_count)) return false;

    if (ShouldEmitNegativeChild() && ShouldEmitPositiveChild()) {
      if ((neg_range().second % vertex_count ==
           pos_range().first % vertex_count) !=
          has_new_shared_point2) return false;

      if ((pos_range().second % vertex_count ==
           neg_range().first % vertex_count) !=
          has_new_shared_point1) return false;
    } else {
      if (has_new_shared_point1) return false;
      if (has_new_shared_point2) return false;
    }

    return true;
  }

  bool ShouldEmitNegativeChild() const {
    return ranges.ShouldEmitNegativeChild();
  }

  bool ShouldEmitPositiveChild() const {
    return ranges.ShouldEmitPositiveChild();
  }

  bool ShouldEmitOnPlane() const {
    return !ranges.ShouldEmitNegativeChild() &&
           !ranges.ShouldEmitPositiveChild();
  }

  const std::pair<size_t, size_t>& neg_range() const {
    return ranges.neg_range;
  }

  const std::pair<size_t, size_t>& pos_range() const {
    return ranges.pos_range;
  }

  ConvexPolygonSplitRanges ranges;

  // A line that should used for the new edge in the negative child. -new_line
  // should be used for the positive child.
  //
  // This field is only initialized if both children are emitted.
  LineRep new_line;

  bool has_new_shared_point1 = false;
  // If `has_new_shared_point1` is set, this point should be inserted before
  // neg_range and after pos_range.
  //
  // Otherwise, if `ShouldEmitNegativeChild` and `ShouldEmitPositiveChild` both
  // return true, then the vertex at pos_range.second should be inserted before
  // neg_range and after pos_range. In this case:
  //   neg_range.first % vertex_count == (pos_range.second + 1) % vertex_count
  HomoPoint3Rep new_shared_point1;

  bool has_new_shared_point2 = false;
  // If `has_new_shared_point2` is set, this point should be inserted before
  // pos_range and after neg_range.
  //
  // Otherwise, if `ShouldEmitNegativeChild` and `ShouldEmitPositiveChild` both
  // return true, then the vertex at neg_range.second should be inserted before
  // pos_range and after neg_range. In this case:
  //   pos_range.first % vertex_count == (neg_range.second + 1) % vertex_count
  HomoPoint3Rep new_shared_point2;
};

std::ostream& operator<<(
    std::ostream& out, const ConvexPolygonSplitRanges& ranges) {
  out << "neg_range=[" << ranges.neg_range.first
      << ", " << ranges.neg_range.second << ")";
  out << ", pos_range=[" << ranges.pos_range.first
      << ", " << ranges.pos_range.second << ")";
  return out;
}

template <size_t point3_bits>
std::ostream& operator<<(
    std::ostream& out, const ConvexPolygonSplitInfo<point3_bits>& info) {
  out << info.ranges;
  if (info.has_new_shared_point1) {
    out << ", new_shared_point1=" << info.new_shared_point1;
  }
  if (info.has_new_shared_point2) {
    out << ", new_shared_point2=" << info.new_shared_point2;
  }
  return out;
}

}  // walnut

#endif // WALNUT_CONVEX_POLYGON_SPLIT_INFO_H__
