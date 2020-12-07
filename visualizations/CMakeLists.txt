cmake_minimum_required(VERSION 3.10)
project(visualizations)

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

add_library(visualization STATIC
            visualization_window.cpp)

add_executable(extrusion extrusion.cpp)
target_link_libraries(extrusion
                      visualization
                      ${GLEW_LIBRARIES} ${VTK_LIBRARIES} )

add_executable(corner_cell_boundary corner_cell_boundary.cpp)
target_link_libraries(corner_cell_boundary
                      visualization
                      ${GLEW_LIBRARIES} ${VTK_LIBRARIES} )

add_executable(zig_zag_border zig_zag_border.cpp)
target_link_libraries(zig_zag_border
                      visualization
                      ${GLEW_LIBRARIES} ${VTK_LIBRARIES} )
