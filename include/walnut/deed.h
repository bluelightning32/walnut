// `Deed` is a pointer to an object where the pointer gets updated if the
// object is moved. The deed must be destructed or returned before the object
// it points to is destructed.
//
// At most one deed can point to the object at any point. Although the deed can
// be temporarily lent to a subdeed. The subdeed must be destructed (which
// restores the parent deed) before the parent deed is destructed. When the
// parent deed is lent out, the underlying object cannot be accessed through
// the parent deed, only through the subdeed.
#ifndef WALNUT_DEED_H__
#define WALNUT_DEED_H__

#include <cassert>
// For std::swap
#include <utility>

namespace walnut {

class GenericDeed;

class DeedObject {
 public:
  DeedObject() = default;

  // Copies the object without copying or transferring ownership.
  //
  // The object can be copied, even it if is owned.
  DeedObject(const DeedObject& other) { }

  // Transfers the object and ownership.
  DeedObject(DeedObject&& other) noexcept;

  // Copies the object without copying or transferring ownership.
  //
  // The object can be copied, even it if is owned.
  DeedObject& operator=(const DeedObject& other) {
    return *this;
  }

  // Transfers the object and ownership
  DeedObject& operator=(DeedObject&& other);

  ~DeedObject() {
    // The deed must be returned before the object is destructed.
    assert(!owner_);
  }

 private:
  friend GenericDeed;

  GenericDeed* owner_ = nullptr;
};

class GenericDeed {
 public:
  // Initializes the deed to nullptr.
  GenericDeed() = default;

  GenericDeed(DeedObject* object) : object_(object) {
    if (object != nullptr && object->owner_ != nullptr) {
      lender_ = object->owner_;
      lender_->MarkLent();
    }
    object->owner_ = this;
  }

  GenericDeed(const GenericDeed&) = delete;

  GenericDeed(GenericDeed&& other) :
      object_(other.object_), lender_(other.lender_) {
    // Borrowed deeds must be returned before they are moved.
    assert(!other.is_lender());
    other.object_ = nullptr;
    other.lender_ = nullptr;
  }

  ~GenericDeed() {
    // Borrowed deeds must be returned before they are destructed.
    assert(!is_lender());
    if (lender_) {
      lender_->object_ = object_;
      assert(!lender_->is_lender());
    }
    if (object_) {
      object_->owner_ = lender_;
    }
  }

  GenericDeed& operator=(GenericDeed&& other) {
    assert(!other.is_lender());
    assert(!is_lender());
    if (object_) {
      object_->owner_ = &other;
    }
    std::swap(object_, other.object_);
    if (object_) {
      object_->owner_ = this;
    }
    std::swap(lender_, other.lender_);
    return *this;
  }

  bool is_lender() const {
    return is_lender_ == this;
  }

  void Return() {
    assert(!is_lender());
    if (lender_) {
      lender_->object_ = object_;
      assert(!lender_->is_lender());
    }
    if (object_) {
      object_->owner_ = lender_;
    }
    object_ = nullptr;
    lender_ = nullptr;
  }

  explicit operator bool() const {
    return get() != nullptr;
  }

  bool operator==(std::nullptr_t) const {
    return get() == nullptr;
  }

  bool operator!=(std::nullptr_t) const {
    return get() != nullptr;
  }

  GenericDeed Lend() {
    assert(!is_lender());
    return GenericDeed(object_, this);
  }

  DeedObject* get() const {
    assert(!is_lender());
    return object_;
  }

 private:
  friend DeedObject;

  GenericDeed(DeedObject* object, GenericDeed* borrowed_from) :
      object_(object), lender_(borrowed_from) {
    lender_->MarkLent();
  }

  void MarkLent() {
    is_lender_ = this;
  }

  union {
    DeedObject* object_ = nullptr;
    // Points to `this` if the pointer was stolen from `this`.
    GenericDeed* is_lender_;
  };
  GenericDeed* lender_ = nullptr;
};

template <typename T>
class Deed : public GenericDeed {
 public:
  static_assert(std::is_base_of<DeedObject, T>::value,
                "The object must inherit from DeedObject.");

  // Initializes the deed to nullptr.
  Deed() = default;

  Deed(T* object) : GenericDeed(object) { }

  Deed(Deed&& other) : GenericDeed(std::move(other)) { }

  Deed& operator=(Deed&& other) = default;

  T* get() const {
    return static_cast<T*>(GenericDeed::get());
  }

  T& operator*() const {
    return *get();
  }

  T* operator->() const {
    return get();
  }

  Deed Lend() {
    return Deed(GenericDeed::Lend());
  }

 private:
  Deed(GenericDeed&& base) : GenericDeed(std::move(base)) { }
};

// Transfers the object and ownership.
inline DeedObject::DeedObject(DeedObject&& other) noexcept :
    owner_(other.owner_) {
  other.owner_ = nullptr;
  if (owner_) {
    owner_->object_ = this;
  }
}

inline DeedObject& DeedObject::operator=(DeedObject&& other) {
  if (owner_) {
    owner_->object_ = &other;
  }
  std::swap(owner_, other.owner_);
  if (owner_) {
    owner_->object_ = this;
  }
  return *this;
}

}  // walnut

#endif // WALNUT_DEED_H__
