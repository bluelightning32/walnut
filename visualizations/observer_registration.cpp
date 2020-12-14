#include "observer_registration.h"

namespace walnut {

ObserverRegistration::ObserverRegistration(vtkObject* object,
                                           unsigned long tag) :
  object_(object), tag_(tag) {
}

ObserverRegistration::~ObserverRegistration() {
  Clear();
}

void ObserverRegistration::Clear() {
  if (object_) {
    object_->RemoveObserver(tag_);
    object_ = nullptr;
  }
}

} // walnut
