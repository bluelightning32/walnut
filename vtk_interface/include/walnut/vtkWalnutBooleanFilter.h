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


  // Quantize the input VTK coordinates to 2^min_exponent. By default
  // `min_exponent` is set to `INT_MIN` to disable quantization.
  //
  // This is used for converting from VTK’s double floating point to Walnut’s
  // internal exact rational format. This does not affect output conversion
  // from Walnut to VTK.
  //
  // By default quantization is disabled and the doubles are converted exactly.
  // However, often the input data already has small rounding errors, and
  // representing these exactly in Walnut slows down the algorithm.
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
