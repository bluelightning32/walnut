#ifndef WALNUT_MEMBER_FORWARD_H__
#define WALNUT_MEMBER_FORWARD_H__

#include <type_traits>
#include <utility>

namespace walnut {

// Even though MemberForward takes 2 template parameters, only one should be
// explicitly specified. For example:
//   MemberForward<Parent>(value)
//
// If Parent is an rvalue type, then MemberForward will convert an argument of
// another type into an rvalue reference.
//
// Else if Parent is not an rvalue type, then MemberForward will return t as
// is.

template <typename Parent, typename T>
typename std::enable_if_t<
  !std::is_lvalue_reference<Parent>::value &&
    !std::is_const<std::remove_reference_t<T>>::value,
  std::remove_reference_t<T>&& >
MemberForward(T&& t) {
  return std::move(t);
}

template <typename Parent, typename T>
std::enable_if_t<std::is_lvalue_reference<Parent>::value ||
                 std::is_const<std::remove_reference_t<T>>::value,
                 T&& >
MemberForward(T&& t) {
  return std::forward<T>(t);
}

}  // walnut

#endif // WALNUT_MEMBER_FORWARD_H__
