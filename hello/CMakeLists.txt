set(PRJ "helloBsp")
project("${PRJ}")
cmake_minimum_required(VERSION 2.8)

include_directories("${PROJECT_SOURCE_DIR}/../MulticoreBSP-for-C")
#link_directories("${PROJECT_SOURCE_DIR}/../MulticoreBSP-for-C/lib")
add_executable("${PRJ}" "${PROJECT_SOURCE_DIR}/hello.cpp")
target_link_libraries("${PRJ}" "${PROJECT_SOURCE_DIR}/../MulticoreBSP-for-C/lib/libmcbsp1.2.0.a" pthread rt)
