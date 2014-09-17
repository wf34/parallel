set(PRJ "lcs-bsp")
project("${PRJ}")
cmake_minimum_required(VERSION 2.8)

set(COMMON_DIR "${PROJECT_SOURCE_DIR}/../../common")
set(MCBSP_DIR "${PROJECT_SOURCE_DIR}/../../MulticoreBSP-for-C")
set(MCBSP_LIBDIR "${MCBSP_DIR}/lib")
set(MCBSP_INCDIR "${MCBSP_DIR}/include")

include_directories("${MCBSP_INCDIR}" "${COMMON_DIR}")

add_executable("${PRJ}" "${COMMON_DIR}/bspedupack.c"
                        "${PROJECT_SOURCE_DIR}/lcs_bsp.cpp")
target_link_libraries("${PRJ}" "${MCBSP_LIBDIR}/libmcbsp1.2.0.a"
                                pthread rt)
