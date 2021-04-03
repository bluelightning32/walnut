#ifndef WALNUT_ASSIGNABLE_WRAPPER_H__
#define WALNUT_ASSIGNABLE_WRAPPER_H__

// For std::enable_if
#include <type_traits>
// For std::move
#include <utility>

namespace walnut {

// Exposes the protected constructors and assignment operators from `T`.
template <typename T>
struct AssignableWrapper : public T {
  using T::T;
  using T::operator=;

  template <typename B = bool,
            std::enable_if_t<std::is_copy_constructible<T>::value, B> = true>
  AssignableWrapper(const T& other)
    noexcept(std::is_nothrow_copy_constructible<T>::value) : T(other) { }

  template <typename B = bool,
            std::enable_if_t<std::is_move_constructible<T>::value, B> = true>
  AssignableWrapper(T&& other)
    noexcept(std::is_nothrow_move_constructible<T>::value) :
      T(std::move(other)) { }
};

}  // walnut

#endif // WALNUT_ASSIGNABLE_WRAPPER_H__
