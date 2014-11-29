set (current_project "convex_hull")
project ("${current_project}")
cmake_minimum_required(VERSION 2.8)

set (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11")

set (common_directory "${PROJECT_SOURCE_DIR}/../common")

set (MCBSP_DIR "${PROJECT_SOURCE_DIR}/../MulticoreBSP-for-C")
set (MCBSP_LIBDIR "${MCBSP_DIR}/lib")
set (MCBSP_INCDIR "${MCBSP_DIR}/include")

include_directories ("${PROJECT_SOURCE_DIR}/include")
include_directories ("${MCBSP_INCDIR}" "${common_directory}")

file (GLOB source ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable (${current_project}
                "${common_directory}/bspedupack.c"
                ${source})

target_link_libraries (${current_project}
                       "${MCBSP_LIBDIR}/libmcbsp1.2.0.a"
                       pthread rt)

################################################################
enable_testing ()
set (test_project "${current_project}_test")

find_package (GTest REQUIRED)
message (STATUS "GTest libs: ${GTEST_LIBRARIES}")
find_package (Qhull REQUIRED)
include_directories (${QHULL_INCLUDE_DIRS})

file (GLOB test_source ${PROJECT_SOURCE_DIR}/test/*.cpp)

add_executable (${test_project} ${test_source})
set_target_properties (${test_project}
                       PROPERTIES COMPILE_DEFINITIONS
                       "TESTING")

target_link_libraries (${test_project}
                       ${GTEST_LIBRARY}
                       ${GTEST_MAIN_LIBRARY}
                       /usr/lib64/libqhullcpp.a
                       ${QHULL_LIBRARY})

