cmake_minimum_required(VERSION 3.10)
project(walnut
        VERSION 1.0
        DESCRIPTION "Exact 3D boolean library"
        )

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Werror)
endif()

add_subdirectory(visualizations EXCLUDE_FROM_ALL)
get_property(visualization_targets
             DIRECTORY visualizations
             PROPERTY BUILDSYSTEM_TARGETS)
add_custom_target(visualizations DEPENDS ${visualization_targets})

add_custom_target(extras DEPENDS visualizations)

option(PACKAGE_TESTS "Build the tests" ON)
if(PACKAGE_TESTS)
    enable_testing()
    include(GoogleTest)
endif()
 
add_subdirectory("${PROJECT_SOURCE_DIR}/extern/googletest" "extern/googletest"
                 EXCLUDE_FROM_ALL)

mark_as_advanced(
    BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
    gmock_build_tests gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)

set_target_properties(gtest PROPERTIES FOLDER extern)
set_target_properties(gtest_main PROPERTIES FOLDER extern)
set_target_properties(gmock PROPERTIES FOLDER extern)
set_target_properties(gmock_main PROPERTIES FOLDER extern)

option(BUILD_TESTS "Build the tests as part of the default target." ON)
option(RUN_TESTS
"Whenever a test file is changed (or any of its dependencies), run the test \
as part of the default build target. Set this to OFF to disable building and \
running the unit tests." ON)

AUX_SOURCE_DIRECTORY(src/walnut SOURCES)
add_library(walnut STATIC ${SOURCES})
target_include_directories(walnut PUBLIC include)

if(BUILD_TESTS)
    enable_testing()
    include(GoogleTest)
  macro(package_add_test TESTNAME)
      # create an exectuable in which the tests will be stored
      add_executable(${TESTNAME} ${ARGN})
      # link the Google test infrastructure, mocking library, and a default
      # main fuction to the test executable.  Remove g_test_main if writing
      # your own main function.
      target_link_libraries(${TESTNAME} gtest gmock gtest_main walnut)
      # gtest_discover_tests replaces gtest_add_tests, see
      # https://cmake.org/cmake/help/v3.10/module/GoogleTest.html for more
      # options to pass to it
      gtest_discover_tests(${TESTNAME}
          # set a working directory so your project root so that you can find
          # test data via paths relative to the project root
          WORKING_DIRECTORY ${PROJECT_DIR}
          PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_DIR}"
      )
      set_target_properties(${TESTNAME} PROPERTIES FOLDER tests)
      add_custom_command(
          OUTPUT ${TESTNAME}_results.json
          COMMAND ${TESTNAME} "--gtest_output=json:${TESTNAME}_results.json"
          DEPENDS ${TESTNAME}
          USES_TERMINAL
      )
      if(RUN_TESTS)
        add_custom_target(run_${TESTNAME} ALL DEPENDS ${TESTNAME}_results.json)
      endif()
  endmacro()
else()
  macro(package_add_test TESTNAME)
  endmacro()
endif()

package_add_test(aabb_convex_polygon_test test/aabb_convex_polygon_test.cpp)
package_add_test(aabb_test test/aabb_test.cpp)
package_add_test(assignable_wrapper_test test/assignable_wrapper_test.cpp)
package_add_test(big_int_test test/big_int_test.cpp)
package_add_test(big_int_words_test test/big_int_words_test.cpp)
package_add_test(big_uint_word_test test/big_uint_word_test.cpp)
package_add_test(bsp_traverser_test test/bsp_traverser_test.cpp)
package_add_test(bsp_tree_test test/bsp_tree_test.cpp)
package_add_test(bsp_tree_pwn_test test/bsp_tree_pwn_test.cpp)
package_add_test(concat_range_test test/concat_range_test.cpp)
package_add_test(connected_polygon_test test/connected_polygon_test.cpp)
package_add_test(convex_polygon_factory_test test/convex_polygon_factory_test.cpp)
package_add_test(convex_polygon_test test/convex_polygon_test.cpp)
package_add_test(convex_polygon_split_info_test test/convex_polygon_split_info_test.cpp)
package_add_test(convex_polygon_vertex_iterator_test test/convex_polygon_vertex_iterator_test.cpp)
package_add_test(convex_vertex_aabb_tracker_test test/convex_vertex_aabb_tracker_test.cpp)
package_add_test(deed_test test/deed_test.cpp)
package_add_test(double_test test/double_test.cpp)
package_add_test(edge_line_connector_test test/edge_line_connector_test.cpp)
package_add_test(half_space2_sideless_compare_test test/half_space2_sideless_compare_test.cpp)
package_add_test(half_space2_test test/half_space2_test.cpp)
package_add_test(half_space3_test test/half_space3_test.cpp)
package_add_test(homo_point3_test test/homo_point3_test.cpp)
package_add_test(member_forward_test test/member_forward_test.cpp)
package_add_test(monotone_decomposer_test test/monotone_decomposer_test.cpp)
package_add_test(monotone_range_test test/monotone_range_test.cpp)
package_add_test(monotone_triangulator_test test/monotone_triangulator_test.cpp)
package_add_test(orienting_monotone_decomposer_test test/orienting_monotone_decomposer_test.cpp)
package_add_test(planar_range_test test/planar_range_test.cpp)
package_add_test(plane_partitioner_test test/plane_partitioner_test.cpp)
package_add_test(plucker_line_test test/plucker_line_test.cpp)
package_add_test(point2_test test/point2_test.cpp)
package_add_test(point3_test test/point3_test.cpp)
package_add_test(r_transformation_test test/r_transformation_test.cpp)
package_add_test(redirectable_value_test test/redirectable_value_test.cpp)
package_add_test(rational_test test/rational_test.cpp)
package_add_test(vector2_test test/vector2_test.cpp)
package_add_test(vector3_test test/vector3_test.cpp)
