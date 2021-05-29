#include "walnut/convex_polygon_split_info.h"

namespace walnut {

std::ostream& operator<<(std::ostream& out,
                         const ConvexPolygonSplitRanges& ranges) {
  out << "neg_range=[" << ranges.neg_range.first
      << ", " << ranges.neg_range.second << ")";
  out << ", pos_range=[" << ranges.pos_range.first
      << ", " << ranges.pos_range.second << ")";
  return out;
}

std::ostream& operator<<(std::ostream& out,
                         const ConvexPolygonSplitInfo& info) {
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
