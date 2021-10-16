#include "walnut/vtkWalnutBooleanFilter.h"

#include <chrono>
#include <vtkInformation.h>
#include <vtkInformationVector.h>

#include "walnut/boolean_operation_filter.h"
#include "walnut/bsp_tree.h"
#include "walnut/vtk_to_walnut_mesh.h"
#include "walnut/walnut_to_vtk_mesh.h"

#define PRINT_TIMING 0

namespace walnut {

vtkStandardNewMacro(vtkWalnutBooleanFilter);

inline int vtkWalnutBooleanFilter::FillInputPortInformation(int port,
                                                     vtkInformation* info) {
  if (!vtkPolyDataAlgorithm::FillInputPortInformation(port, info)) {
    return 0;
  }
  info->Set(vtkAlgorithm::INPUT_IS_REPEATABLE(), 1);
  return 1;
}

int vtkWalnutBooleanFilter::RequestData(vtkInformation* request,
                                        vtkInformationVector** input_vector,
                                        vtkInformationVector* output_vector) {
#if PRINT_TIMING
  auto start = std::chrono::steady_clock::now();
#endif

  int num_connections = input_vector[0]->GetNumberOfInformationObjects();
  BSPTree<> tree;
  std::vector<BSPContentId> ids;

  for (int i = 0; i < num_connections; ++i) {
    ids.push_back(tree.AllocateId());
    const bool flip = Operation == OperationType::VTK_DIFFERENCE && i > 0;
    tree.AddContents(ids.back(),
                     VTKToWalnutMesh(
                      vtkPolyData::GetData(input_vector[0], i), MinExponent,
                      flip));
  }

#if PRINT_TIMING
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
  UnionIdsFilter union_filter;
  IntersectIdsFilter intersect_filter;
  SubtractIdsFilter subtract_filter;
  BooleanOperationFilter* selected_filter;

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

  ConnectingVisitor<BooleanOperationFilter> visitor(*selected_filter,
                                                    error_log);
  tree.Traverse(visitor);

  if (errored) return 0;

  visitor.FilterEmptyPolygons();
  vtkPolyData* output = vtkPolyData::GetData(output_vector, 0);
#if PRINT_TIMING
  auto operation_done = std::chrono::steady_clock::now();
  std::cerr << "The boolean operation took: "
            << std::chrono::duration<double>(operation_done - added).count()
            << std::endl;
#endif

  SaveWalnutMeshToVTK(visitor.TakePolygons(), output);

#if PRINT_TIMING
  auto end = std::chrono::steady_clock::now();
  std::cerr << "Converting back to VTK took: "
            << std::chrono::duration<double>(end - operation_done).count()
            << std::endl;
#endif

  return 1;
}

}  // walnut
