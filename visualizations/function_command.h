#ifndef WALNUT_VISUALIZATIONS_FUNCTION_COMMAND_H__
#define WALNUT_VISUALIZATIONS_FUNCTION_COMMAND_H__

#include <functional>

#include <vtkCommand.h>
#include <vtkSmartPointer.h>

namespace walnut {

class FunctionCommand : public vtkCommand {
 public:
  using FunctionType = std::function<void (vtkObject *caller,
                                           unsigned long event_id,
                                           void *call_data)>;

  void set_function(FunctionType function) {
    function_ = function;
  }

  // Implements the vtkCommand interface
  void Execute (vtkObject *caller, unsigned long event_id,
                void *call_data) override {
    function_(caller, event_id, call_data);
  }

  // Creates a new heap allocted FunctionCommand.
  //
  // This is necessary for vtkSmartPointer<FunctionCommand> to work.
  static FunctionCommand* New() {
    return new FunctionCommand;
  }

 private:
  FunctionType function_;
};

inline vtkSmartPointer<FunctionCommand> MakeFunctionCommand(
    FunctionCommand::FunctionType function) {
  auto result = vtkSmartPointer<FunctionCommand>::New();
  result->set_function(function);
  return result;
}

} // walnut

#endif // WALNUT_VISUALIZATIONS_FUNCTION_COMMAND_H__
