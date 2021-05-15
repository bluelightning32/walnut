#ifndef WALNUT_REDIRECTABLE_VALUE_H__
#define WALNUT_REDIRECTABLE_VALUE_H__

// For aligned_union
#include <type_traits>
// For forward
#include <utility>

namespace walnut {

template <typename Value>
class RedirectableValue {
 public:
  RedirectableValue() : redirect_(this) {
    new(&value_) Value();
  }

  template<typename ...Args>
  RedirectableValue(Args&&... args) : redirect_(this) {
    new(&value_) Value(std::forward<Args...>(args...));
  }

  RedirectableValue(RedirectableValue& other) : redirect_(&other) { }

  ~RedirectableValue() {
    if (redirect_ == this) {
      reinterpret_cast<Value *>(&value_)->~Value();
    }
  }

  bool operator!=(const RedirectableValue& other) const {
    return &primary() != &other.primary();
  }

  bool operator==(const RedirectableValue& other) const {
    return &primary() == &other.primary();
  }

  Value& operator*() {
    return *reinterpret_cast<Value *>(&primary().value_);
  }

  const Value& operator*() const {
    return *reinterpret_cast<const Value *>(&primary().value_);
  }

  Value* operator->() {
    return reinterpret_cast<Value *>(&primary().value_);
  }

  const Value* operator->() const {
    return reinterpret_cast<const Value *>(&primary().value_);
  }

  bool IsPrimary() const {
    return &primary() == this;
  }

  const RedirectableValue& primary() const {
    RedirectableValue* redirect = redirect_;
    while (redirect != redirect->redirect_) {
      redirect = redirect->redirect_;
    }
    return *redirect;
  }

  RedirectableValue& primary() {
    while (redirect_ != redirect_->redirect_) {
      redirect_ = redirect_->redirect_;
    }
    return *redirect_;
  }

  // Change this entry so that when it is dereferenced the value at `to` is
  // accessed instead.
  void Redirect(RedirectableValue& to) {
    RedirectableValue& final_to = to.primary();
    if (&final_to == redirect_) return;

    if (redirect_ == this) {
      reinterpret_cast<Value *>(&value_)->~Value();
    }
    redirect_ = &to.primary();
  }

 private:
  RedirectableValue* redirect_;
  typename std::aligned_union<0, Value>::type value_;
};

}  // walnut

#endif // WALNUT_REDIRECTABLE_VALUE_H__
