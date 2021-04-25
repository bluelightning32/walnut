#ifndef WALNUT_ASSIGNABLE_WRAPPER_H__
#define WALNUT_ASSIGNABLE_WRAPPER_H__

// For std::enable_if
#include <type_traits>
// For std::move
#include <utility>

namespace walnut {

template <typename T>
struct RValueKey {
  template <typename U,
            std::enable_if_t<std::is_base_of<T, U>::value, bool> = true>
  constexpr RValueKey(RValueKey<U> other) : value(other.value) { }

  constexpr T& get() const {
    return *value;
  }

 private:
  friend T;
  template <typename U>
  friend struct RValueKey;

  explicit constexpr RValueKey(T&& value) : value(&value) { }

  T* value;
};

// Exposes protected and public constructors and assignment operators from T.
template <typename T>
struct ExposeProtectedConstructAssign : public T {
  using T::T;
  using T::operator=;
};

// Exposes the RValueKey constructor and assignment from T.
template <typename T>
struct AssignableWrapper : public T {
  using T::T;
  using T::operator=;

  template <typename B = bool,
            std::enable_if_t<std::is_copy_constructible<T>::value, B> = true>
  AssignableWrapper(const T& other)
    noexcept(std::is_nothrow_copy_constructible<T>::value) : T(other) { }

  AssignableWrapper(const AssignableWrapper<T>& other)
    noexcept(std::is_nothrow_copy_constructible<T>::value) : T(other) { }

  AssignableWrapper(AssignableWrapper<T>&& other)
      noexcept(std::is_nothrow_constructible<T, RValueKey<T>>::value) :
    T(std::move(other).GetRValueKey()) { }

  AssignableWrapper& operator=(AssignableWrapper<T>&& other)
      noexcept(std::is_nothrow_assignable<ExposeProtectedConstructAssign<T>,
                                          RValueKey<T>>::value) {
    *this = std::move(other).GetRValueKey();
    return *this;
  }

  AssignableWrapper& operator=(const AssignableWrapper<T>& other) = default;
};

}  // walnut

#endif // WALNUT_ASSIGNABLE_WRAPPER_H__
