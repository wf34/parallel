set(PRJ "bspbench")
project("${PRJ}")
cmake_minimum_required(VERSION 2.8)

set(COMMON_DIR "${PROJECT_SOURCE_DIR}/../common")
include_directories("${PROJECT_SOURCE_DIR}/../MulticoreBSP-for-C/include"
		    "${COMMON_DIR}")
#link_directories("${PROJECT_SOURCE_DIR}/../MulticoreBSP-for-C/lib")
add_executable("${PRJ}" "${COMMON_DIR}/bspedupack.c" "${PROJECT_SOURCE_DIR}/bspbench.c")
target_link_libraries("${PRJ}" "${PROJECT_SOURCE_DIR}/../MulticoreBSP-for-C/lib/libmcbsp1.2.0.a" pthread rt)
