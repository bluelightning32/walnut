cmake_minimum_required(VERSION 3.10)
project(vtk_interface
        VERSION 1.0
        DESCRIPTION "Walnut to VTK interface"
        )

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/vtk_interface/include)

add_library(walnut_vtk STATIC
            src/vtkWalnutBooleanFilter.cpp)
target_link_libraries(walnut_vtk walnut ${VTK_LIBRARIES})

set(WALNUT_VTK_PUBLIC_HEADERS
    "include/walnut/vtkWalnutBooleanFilter.h"
    "include/walnut/vtk_to_walnut_mesh.h"
    "include/walnut/walnut_to_vtk_mesh.h"
    )
set_target_properties(walnut_vtk PROPERTIES
                      PUBLIC_HEADER "${WALNUT_VTK_PUBLIC_HEADERS}")
install(TARGETS walnut_vtk PUBLIC_HEADER DESTINATION "include/walnut")
