#include "walnut/point3.h"

// For std::min
#include <algorithm>

namespace walnut {

bool Point3::LexicographicallyLt(const Point3& a, const Point3& b) {
  return std::lexicographical_compare(a.components().begin(),
                                      a.components().end(),
                                      b.components().begin(),
                                      b.components().end());
}

std::ostream& operator<<(std::ostream& out, const Point3& v) {
  return out << v.vector_from_origin();
}

}  // walnut
