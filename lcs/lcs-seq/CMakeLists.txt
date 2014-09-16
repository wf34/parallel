set(PRJ "lcs-seq")
project("${PRJ}")
cmake_minimum_required(VERSION 2.8)

set(MCBSP_DIR "${PROJECT_SOURCE_DIR}/../../MulticoreBSP-for-C")
set(MCBSP_LIBDIR "${MCBSP_DIR}/lib")
set(MCBSP_INCDIR "${MCBSP_DIR}/include")

include_directories("${MCBSP_INCDIR}")

add_executable("${PRJ}" "${PROJECT_SOURCE_DIR}/lcs.cpp")
target_link_libraries("${PRJ}" "${MCBSP_LIBDIR}/libmcbsp1.2.0.a"
                                pthread rt)
