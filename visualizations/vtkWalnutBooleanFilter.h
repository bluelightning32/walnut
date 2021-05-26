#ifndef WALNUT_VISUALIZATIONS_VTK_WALNUT_BOOLEAN_FILTER_H__
#define WALNUT_VISUALIZATIONS_VTK_WALNUT_BOOLEAN_FILTER_H__

#include <chrono>

#include <vtkBooleanOperationPolyDataFilter.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkSetGet.h>

#include "mesh_adapter.h"
#include "walnut/boolean_operation_filter.h"
#include "walnut/bsp_tree.h"

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
                  vtkInformationVector* output_vector) override {
    auto start = std::chrono::steady_clock::now();

    int num_connections = input_vector[0]->GetNumberOfInformationObjects();
    walnut::BSPTree<> tree;
    std::vector<walnut::BSPContentId> ids;

    for (int i = 0; i < num_connections; ++i) {
      ids.push_back(tree.AllocateId());
      const bool flip = Operation == OperationType::VTK_DIFFERENCE && i > 0;
      tree.AddContents(ids.back(),
                       walnut::VTKToWalnutMesh(
                        vtkPolyData::GetData(input_vector[0], i), MinExponent,
                        flip));
    }

#if 1
    auto added = std::chrono::steady_clock::now();
    std::cerr << "Converting and adding the shapes to the BSP took: "
              << std::chrono::duration<double>(added - start).count()
              << std::endl;
#endif

    bool errored = false;
    auto error_log = [&errored, this](const std::string& error) {
      vtkErrorMacro(<< error);
      errored = true;
    };
    walnut::UnionIdsFilter union_filter;
    walnut::IntersectIdsFilter intersect_filter;
    walnut::SubtractIdsFilter subtract_filter;
    walnut::BooleanOperationFilter* selected_filter;

    switch (Operation) {
      default:
        assert(false);
      case OperationType::VTK_UNION:
        union_filter.SetIds(std::move(ids));
        selected_filter = &union_filter;
        break;

      case OperationType::VTK_INTERSECTION:
        intersect_filter.SetIds(std::move(ids));
        selected_filter = &intersect_filter;
        break;

      case OperationType::VTK_DIFFERENCE:
        subtract_filter.SetIds(std::move(ids));
        selected_filter = &subtract_filter;
        break;
    }

    walnut::ConnectingVisitor<walnut::BooleanOperationFilter> visitor(
        *selected_filter, error_log);
    tree.Traverse(visitor);

    if (errored) return 0;

    visitor.FilterEmptyPolygons();
    vtkPolyData* output = vtkPolyData::GetData(output_vector, 0);
#if 1
    auto operation_done = std::chrono::steady_clock::now();
    std::cerr << "The boolean operation took: "
              << std::chrono::duration<double>(operation_done - added).count()
              << std::endl;
#endif

    walnut::SaveWalnutMeshToVTK(visitor.TakePolygons(), output);

#if 1
    auto end = std::chrono::steady_clock::now();
    std::cerr << "Converting back to VTK took: "
              << std::chrono::duration<double>(end - operation_done).count()
              << std::endl;
#endif

    return 1;
  }

 private:
  OperationType Operation = OperationType::VTK_UNION;
  int MinExponent = std::numeric_limits<int>::min();
};

vtkStandardNewMacro(vtkWalnutBooleanFilter);

inline int vtkWalnutBooleanFilter::FillInputPortInformation(int port,
                                                     vtkInformation* info) {
  if (!vtkPolyDataAlgorithm::FillInputPortInformation(port, info)) {
    return 0;
  }
  info->Set(vtkAlgorithm::INPUT_IS_REPEATABLE(), 1);
  return 1;
}

#endif // WALNUT_VISUALIZATIONS_VTK_WALNUT_BOOLEAN_FILTER_H__
