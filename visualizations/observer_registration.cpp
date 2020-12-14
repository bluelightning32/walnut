#include "observer_registration.h"

namespace walnut {

ObserverRegistration::ObserverRegistration(vtkObject* object,
                                           unsigned long tag) :
  object_(object), tag_(tag) {
}

ObserverRegistration::~ObserverRegistration() {
  Clear();
}

ObserverRegistration::ObserverRegistration(ObserverRegistration&& other) :
    object_(std::move(other.object_)), tag_(other.tag_) {
  other.object_ = nullptr;
}

ObserverRegistration& ObserverRegistration::operator=(
    ObserverRegistration&& other) {
  object_ = other.object_;
  tag_ = other.tag_;
  other.object_ = nullptr;
  return *this;
}

void ObserverRegistration::Clear() {
  if (object_) {
    object_->RemoveObserver(tag_);
    object_ = nullptr;
  }
}

} // walnut
