cmake_minimum_required(VERSION 3.10)
project(visualizations)

set(CMAKE_CXX_STANDARD 11)
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

add_library(visualization STATIC visualization_window.cpp)

add_executable(extrusion extrusion.cpp)
target_link_libraries(extrusion
                      visualization
                      ${GLEW_LIBRARIES} ${VTK_LIBRARIES} )
