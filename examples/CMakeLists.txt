cmake_minimum_required(VERSION 3.10)
project(examples)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GLEW REQUIRED)

find_package(VTK REQUIRED COMPONENTS
    vtkCommonCore
    vtkFiltersGeneral
    vtkFiltersHybrid
    vtkFiltersModeling
    vtkFiltersSources
    vtkInteractionStyle
    vtkIOGeometry
    vtkIOImage
    vtkIOPLY
    vtkIOXML
    vtkRenderingCore
    vtkRenderingFreeType
    vtkRenderingOpenGL2
    vtkViewsInfovis
    NO_MODULE)

include(${VTK_USE_FILE})

# Make the top level walnut include files visible to all targets in this file.
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/vtk_interface/include)

add_library(examples_helper STATIC
            normals_actor.cpp
            observer_registration.cpp
            points_actor.cpp
            visualization_window.cpp)
target_link_libraries(examples_helper walnut_vtk)

add_executable(union union.cpp)
target_link_libraries(union
                      examples_helper
                      ${GLEW_LIBRARIES} ${VTK_LIBRARIES} )

add_executable(zig_zag_border zig_zag_border.cpp)
target_link_libraries(zig_zag_border
                      examples_helper
                      ${GLEW_LIBRARIES} ${VTK_LIBRARIES} )
