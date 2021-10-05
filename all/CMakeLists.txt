# this script adds all subprojects to a single build to allow IDEs understand the full project
# structure.

cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(BuildAll LANGUAGES CXX)

include(../cmake/tools.cmake)

# needed to generate test target
enable_testing()

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../test ${CMAKE_BINARY_DIR}/test)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../documentation ${CMAKE_BINARY_DIR}/documentation)

# Examples
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../examples/extended_kalman_filter ${CMAKE_BINARY_DIR}/ekf)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../examples/infinite_horizon_lqr ${CMAKE_BINARY_DIR}/lqr)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../examples/infinite_horizon_lqr_plot ${CMAKE_BINARY_DIR}/lqr_plot)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../examples/pid ${CMAKE_BINARY_DIR}/pid)