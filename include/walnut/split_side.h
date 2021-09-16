#ifndef WALNUT_SPLIT_SIDE_H__
#define WALNUT_SPLIT_SIDE_H__

// for std::ostream
#include <ostream>

#include "walnut/half_space3.h"

namespace walnut {

struct SplitSide {
  const HalfSpace3* split = nullptr;
  bool pos_side = false;

  bool operator==(const SplitSide& other) const {
    return split == other.split && pos_side == other.pos_side;
  }

  bool operator!=(const SplitSide& other) const {
    return split != other.split || pos_side != other.pos_side;
  }
};

inline std::ostream& operator<<(std::ostream& out, const SplitSide& info) {
  if (info.split == nullptr) {
    out << "none";
  } else {
    out << "[";
    if (info.pos_side) {
      out << "pos";
    } else {
      out << "neg";
    }
    out << " " << *info.split << "]";
  }
  return out;
}

}  // walnut

#endif // WALNUT_SPLIT_SIDE_H__
