#ifndef WALNUT_ASSIGNABLE_WRAPPER_H__
#define WALNUT_ASSIGNABLE_WRAPPER_H__

#include "walnut/homo_point3.h"
#include "walnut/plucker_line.h"

namespace walnut {

// Exposes the protected constructors and assignment operators from `T`.
template <typename T>
struct AssignableWrapper : public T {
  using T::T;
  using T::operator=;
};

}  // walnut

#endif // WALNUT_ASSIGNABLE_WRAPPER_H__
