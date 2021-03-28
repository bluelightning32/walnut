#include "walnut/vector3.h"

namespace walnut {

std::ostream& operator<<(std::ostream& out, const Vector3& v) {
  return out << "{ "
             << v.components()[0] << ", "
             << v.components()[1] << ", "
             << v.components()[2]
             << " }";
}

}  // walnut
