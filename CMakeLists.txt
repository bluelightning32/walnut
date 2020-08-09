cmake_minimum_required(VERSION 3.10)
project(walnut
        VERSION 1.0
        DESCRIPTION "Exact 3D boolean library"
        )

option(PACKAGE_TESTS "Build the tests" ON)
if(PACKAGE_TESTS)
    enable_testing()
    include(GoogleTest)
endif()
 
add_subdirectory("${PROJECT_SOURCE_DIR}/extern/googletest" "extern/googletest")

mark_as_advanced(
    BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
    gmock_build_tests gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)

set_target_properties(gtest PROPERTIES FOLDER extern)
set_target_properties(gtest_main PROPERTIES FOLDER extern)
set_target_properties(gmock PROPERTIES FOLDER extern)
set_target_properties(gmock_main PROPERTIES FOLDER extern)

macro(package_add_test TESTNAME)
    # create an exectuable in which the tests will be stored
    add_executable(${TESTNAME} ${ARGN})
    # link the Google test infrastructure, mocking library, and a default main fuction to
    # the test executable.  Remove g_test_main if writing your own main function.
    target_link_libraries(${TESTNAME} gtest gmock gtest_main)
    target_include_directories(${TESTNAME} PUBLIC include)
    # gtest_discover_tests replaces gtest_add_tests,
    # see https://cmake.org/cmake/help/v3.10/module/GoogleTest.html for more options to pass to it
    gtest_discover_tests(${TESTNAME}
        # set a working directory so your project root so that you can find test data via paths relative to the project root
        WORKING_DIRECTORY ${PROJECT_DIR}
        PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_DIR}"
    )
    set_target_properties(${TESTNAME} PROPERTIES FOLDER tests)
endmacro()

package_add_test(big_int_test test/big_int_test.cpp)
package_add_test(big_uint_test test/big_uint_test.cpp)
package_add_test(big_uint_word_test test/big_uint_word_test.cpp)
package_add_test(plane_test test/plane_test.cpp)
package_add_test(convex_polygon_test test/convex_polygon_test.cpp)
package_add_test(monotone_decomposer_test test/monotone_decomposer_test.cpp)
package_add_test(monotone_range_test test/monotone_range_test.cpp)
package_add_test(monotone_triangulator_test test/monotone_triangulator_test.cpp)
package_add_test(orienting_monotone_decomposer_test test/orienting_monotone_decomposer_test.cpp)
package_add_test(vector2_test test/vector2_test.cpp)
package_add_test(vector3_test test/vector3_test.cpp)
package_add_test(vertex2_test test/vertex2_test.cpp)
package_add_test(vertex3_test test/vertex3_test.cpp)
