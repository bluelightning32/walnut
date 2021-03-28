#include "walnut/half_space2.h"

namespace walnut {

std::ostream& operator<<(std::ostream& out, const HalfSpace2& p) {
  return out << "{ x*" << p.x()
             << " + y*" << p.y()
             << " = " << p.d()
             << " }";
}

}  // walnut
