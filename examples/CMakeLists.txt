cmake_minimum_required(VERSION 3.10)
project(examples)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
  set(GLEW_LIBRARIES "")
else()
  find_package(GLEW REQUIRED)
endif()

find_package(VTK REQUIRED COMPONENTS
    vtkCommonCore
    vtkFiltersGeneral
    vtkFiltersHybrid
    vtkFiltersModeling
    vtkFiltersParallel
    vtkFiltersSources
    vtkInteractionStyle
    vtkIOGeometry
    vtkIOImage
    vtkIOPLY
    vtkIOXML
    vtkRenderingAnnotation
    vtkRenderingCore
    vtkRenderingFreeType
    vtkRenderingLabel
    vtkRenderingOpenGL2
    vtkViewsInfovis
    NO_MODULE)

if (VTK_VERSION VERSION_LESS "8.90.0")
    include(${VTK_USE_FILE})
endif()

add_library(examples_helper STATIC
            bsp_visualization.cpp
            normals_actor.cpp
            observer_registration.cpp
            points_actor.cpp
            visualization_window.cpp)
target_link_libraries(examples_helper walnut_vtk)

# On Linux, executables can include an RPATH option to specify where to find
# their dependent dynamic libraries. Windows does not have such an option.
# Furthermore, there is no official VTK installer for Windows. The DLLs are
# likely sitting in some directory that's not in the path.
#
# To workaround that issue, on Windows the VTK DLLs are copied to the build
# directory. That way the examples can be launched from the build directory,
# because the executable's directory is part of the DLL search path.
if(MSVC)
  list (GET VTK_LIBRARIES 0 VTK_FIRST_LIBRARY)
  get_target_property(VTK_FIRST_LIBRARY_PATH ${VTK_FIRST_LIBRARY} LOCATION)
  get_filename_component(VTK_LIB_DIR ${VTK_FIRST_LIBRARY_PATH} DIRECTORY)
  message(STATUS "Copying VTK dlls from ${VTK_LIB_DIR} to the build directory.")
  file(GLOB VTK_DLLS "${VTK_LIB_DIR}/*.dll")
  foreach(VTK_DLL ${VTK_DLLS})
    get_filename_component(BASENAME ${VTK_DLL} NAME)
    configure_file(${VTK_DLL} ${BASENAME} COPYONLY)
  endforeach()
endif()

add_executable(kerf_effect1 kerf_effect1.cpp)
target_link_libraries(kerf_effect1
                      examples_helper
                      ${GLEW_LIBRARIES} ${VTK_LIBRARIES} )

add_executable(kerf_effect2 kerf_effect2.cpp)
target_link_libraries(kerf_effect2
                      examples_helper
                      ${GLEW_LIBRARIES} ${VTK_LIBRARIES} )

add_executable(bottom_top_check bottom_top_check.cpp)
target_link_libraries(bottom_top_check
                      examples_helper
                      ${GLEW_LIBRARIES} ${VTK_LIBRARIES} )

add_executable(union union.cpp)
target_link_libraries(union
                      examples_helper
                      ${GLEW_LIBRARIES} ${VTK_LIBRARIES} )

add_executable(zig_zag_border zig_zag_border.cpp)
target_link_libraries(zig_zag_border
                      examples_helper
                      ${GLEW_LIBRARIES} ${VTK_LIBRARIES} )

add_executable(self_intersection self_intersection.cpp)
target_link_libraries(self_intersection
                      examples_helper
                      ${GLEW_LIBRARIES} ${VTK_LIBRARIES} )

add_executable(r_transformation_effect r_transformation_effect.cpp)
target_link_libraries(r_transformation_effect
                      examples_helper
                      ${GLEW_LIBRARIES} ${VTK_LIBRARIES} )

add_executable(redundant_faces redundant_faces.cpp)
target_link_libraries(redundant_faces
                      examples_helper
                      ${GLEW_LIBRARIES} ${VTK_LIBRARIES} )

add_executable(simple_entrance simple_entrance.cpp)
target_link_libraries(simple_entrance
                      examples_helper
                      ${GLEW_LIBRARIES} ${VTK_LIBRARIES} )

add_executable(split_before_after_crossing split_before_after_crossing.cpp)
target_link_libraries(split_before_after_crossing
                      examples_helper
                      ${GLEW_LIBRARIES} ${VTK_LIBRARIES} )

if (VTK_VERSION VERSION_GREATER_EQUAL "8.90.0")
  vtk_module_autoinit(
    TARGETS
    kerf_effect1
    kerf_effect2
    bottom_top_check
    union
    zig_zag_border
    self_intersection
    r_transformation_effect
    redundant_faces
    simple_entrance
    split_before_after_crossing
    MODULES ${VTK_LIBRARIES}
    )
endif()
