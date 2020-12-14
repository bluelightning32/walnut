#ifndef WALNUT_VISUALIZATIONS_OBSERVER_REGISTRATION_H__
#define WALNUT_VISUALIZATIONS_OBSERVER_REGISTRATION_H__

#include <vtkObject.h>
#include <vtkSmartPointer.h>

namespace walnut {

// Unregisters a VTK observer when destructed.
class ObserverRegistration {
 public:
  ObserverRegistration(vtkObject* object, unsigned long tag);
  ~ObserverRegistration();

  void Clear();

 private:
  vtkSmartPointer<vtkObject> object_;
  unsigned long tag_;
};

} // walnut

#endif // WALNUT_VISUALIZATIONS_OBSERVER_REGISTRATION_H__
