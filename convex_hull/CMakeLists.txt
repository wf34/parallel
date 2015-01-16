set (current_project "convex_hull")
set (rbox_project "normal_rbox")
project ("${current_project}")
cmake_minimum_required(VERSION 2.8)

if (NOT WIN32)
    set (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
else ()
    set (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/win")
endif ()

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11")
if (WIN32)
    add_definitions(-DFREEIMAGE_LIB)
endif ()

find_package (FreeImage REQUIRED)

set (common_directory "${PROJECT_SOURCE_DIR}/../common")
set (MCBSP_DIR "${PROJECT_SOURCE_DIR}/../MulticoreBSP-for-C")
set (MCBSP_LIBDIR "${MCBSP_DIR}/lib")
set (MCBSP_INCDIR "${MCBSP_DIR}")

include_directories (${MCBSP_INCDIR}
                     ${common_directory}
                     "${PROJECT_SOURCE_DIR}/include"
                     ${FreeImage_INCLUDE_DIRS}
                    )

add_executable (${current_project}
                "${common_directory}/bspedupack.c"
                "${PROJECT_SOURCE_DIR}/src/convex_hull.cpp"
               )
add_executable (${rbox_project}
                "${PROJECT_SOURCE_DIR}/src/normal_rbox.cpp"
               )
target_link_libraries (${current_project}
                       "${MCBSP_LIBDIR}/libmcbsp1.2.0.a"
                       pthread rt
                       ${FreeImage_LIBRARIES}
                      )

################################################################
#enable_testing ()
#set (test_project "${current_project}_test")
#
#find_package (GTest REQUIRED)
#message (STATUS "GTest libs: ${GTEST_LIBRARIES}")
#find_package (Qhull REQUIRED)
#include_directories (${QHULL_INCLUDE_DIRS})
#
#file (GLOB test_source ${PROJECT_SOURCE_DIR}/test/*.cpp)
#
#add_executable (${test_project} ${test_source})
#set_target_properties (${test_project}
#                       PROPERTIES COMPILE_DEFINITIONS
#                       "TESTING")
#
#target_link_libraries (${test_project}
#                       ${GTEST_LIBRARY}
#                       ${GTEST_MAIN_LIBRARY}
#                       /usr/local/lib/libqhullcpp.a
#                       ${QHULL_LIBRARY})

