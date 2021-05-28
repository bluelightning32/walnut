#ifndef WALNUT_VTK_WALNUT_BOOLEAN_FILTER_H__
#define WALNUT_VTK_WALNUT_BOOLEAN_FILTER_H__

#include <limits>

#include <vtkBooleanOperationPolyDataFilter.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkSetGet.h>

namespace walnut {

class VTK_EXPORT vtkWalnutBooleanFilter : public vtkPolyDataAlgorithm {
 public:
  using OperationType = vtkBooleanOperationPolyDataFilter::OperationType;

  static vtkWalnutBooleanFilter* New ();

  vtkSetClampMacro(/*name=*/Operation, /*type=*/OperationType,
                   /*min=*/OperationType::VTK_UNION,
                   /*max=*/OperationType::VTK_DIFFERENCE);
  vtkGetMacro(/*name=*/Operation, /*type=*/OperationType);

  vtkSetClampMacro(/*name=*/MinExponent, /*type=*/int,
                   /*min=*/std::numeric_limits<int>::min(), /*max=*/0);
  vtkGetMacro(/*name=*/MinExponent, /*type=*/int);

  void SetOperationToUnion() {
    SetOperation(OperationType::VTK_UNION );
  }
  void SetOperationToIntersection() {
    SetOperation(OperationType::VTK_INTERSECTION );
  }
  void SetOperationToDifference() {
    SetOperation(OperationType::VTK_DIFFERENCE );
  }

 protected:
  int FillInputPortInformation(int port, vtkInformation* info) override;

  int RequestData(vtkInformation* request, vtkInformationVector** input_vector,
                  vtkInformationVector* output_vector) override;

 private:
  OperationType Operation = OperationType::VTK_UNION;
  int MinExponent = std::numeric_limits<int>::min();
};

}  // walnut

#endif // WALNUT_VTK_WALNUT_BOOLEAN_FILTER_H__
