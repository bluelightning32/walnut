#include "walnut/double_point3.h"

namespace walnut {

std::ostream& operator<<(std::ostream& out, const DoublePoint3& p) {
  return out << "{ "
             << p.x << ", "
             << p.y << ", "
             << p.z
             << " }";
}

}  // walnut
