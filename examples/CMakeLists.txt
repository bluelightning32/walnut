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
    vtkFiltersParallel
    vtkFiltersSources
    vtkImagingOpenGL2
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

include(${VTK_USE_FILE})

add_library(examples_helper STATIC
            bsp_visualization.cpp
            normals_actor.cpp
            observer_registration.cpp
            points_actor.cpp
            visualization_window.cpp)
target_link_libraries(examples_helper walnut_vtk)

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
