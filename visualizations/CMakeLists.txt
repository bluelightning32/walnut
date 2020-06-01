cmake_minimum_required(VERSION 3.1)
project(examples)

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

add_executable(extrusion extrusion.cxx)
target_link_libraries(extrusion ${GLEW_LIBRARIES} ${VTK_LIBRARIES} )
